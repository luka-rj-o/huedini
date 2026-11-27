"""
Xbox Series X|S Controller Reader (Linux evdev) & Configurable Teleop Mapper.

- On Raspberry Pi Zero W (Linux), discovers Bluetooth BLE HOGP or wired USB-C
  Xbox controllers via `evdev` using a lightweight `select()` loop (< 2% CPU).
- On non-Linux hosts (Windows/macOS), `evdev` is not installed and the reader
  stays cleanly dormant while the Browser Gamepad API bridge can feed snapshots
  for `--mock-hardware` testing.
- Supports dynamic button remapping, axis presets, inversions, deadzone/curve
  tuning, precision creep scaling, and haptic rumble feedback.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import json
import logging
from pathlib import Path
import re
import select
import subprocess
import sys
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

try:
    import evdev  # type: ignore
    from evdev import ecodes, ff  # type: ignore
    EVDEV_AVAILABLE = True
except ImportError:
    evdev = None  # type: ignore
    ecodes = None  # type: ignore
    ff = None  # type: ignore
    EVDEV_AVAILABLE = False

logger = logging.getLogger("gantry.gamepad")

CANONICAL_BUTTONS = (
    "A",
    "B",
    "X",
    "Y",
    "LB",
    "RB",
    "LT",
    "RT",
    "VIEW",
    "MENU",
    "L3",
    "R3",
)

VALID_BUTTON_CHOICES = CANONICAL_BUTTONS + ("NONE",)

AXIS_KEYS = ("x", "y", "z", "a")

AXIS_SOURCES = (
    "ls_x",
    "ls_y",
    "rs_x",
    "rs_y",
    "bumpers",
    "triggers",
    "dpad_x",
    "dpad_y",
    "none",
)

AXIS_PRESETS: Dict[str, Dict[str, str]] = {
    "bumper_servo": {"x": "ls_x", "y": "ls_y", "z": "rs_y", "a": "bumpers"},
    "sticks": {"x": "ls_x", "y": "ls_y", "z": "rs_y", "a": "rs_x"},
    "triggers_bumpers": {"x": "ls_x", "y": "ls_y", "z": "triggers", "a": "bumpers"},
}

ACTION_KEYS = (
    "grip_open",
    "precision_hold",
    "weigh",
    "home_all",
    "soft_estop",
    "toggle_mode",
)

DEFAULT_GAMEPAD_MAPPING: Dict[str, object] = {
    "schema_version": 2,
    "axis_preset": "bumper_servo",  # "bumper_servo" | "sticks" | "triggers_bumpers" | "custom"
    "axes": {
        "x": "ls_x",
        "y": "ls_y",
        "z": "rs_y",
        "a": "bumpers",
    },
    "invert_x": False,
    "invert_y": False,
    "invert_z": False,
    "invert_a": False,
    "gripper_mode": "hold",  # "hold" | "toggle"
    "buttons": {
        "grip_open": "A",
        "precision_hold": "LT",
        "weigh": "X",
        "home_all": "Y",
        "soft_estop": "B",
        "toggle_mode": "MENU",
    },
    "speed_scale": 0.65,
    "deadzone": 0.12,
    "curve_exp": 2.2,
    "precision_scale": 0.25,
    "max_vx": 60.0,
    "max_vy": 100.0,
    "max_vz_up": 35.0,
    "max_vz_down": 22.0,
    "max_va": 92.0,
    "z_guard_enabled": True,
    "z_guard_force_g": 150.0,
    "telemetry_mode": "suspended",  # "suspended" | "low_rate"
}


def normalize_gamepad_mapping(raw: Optional[dict]) -> Dict[str, object]:
    """Validates and fills missing fields in a gamepad mapping configuration."""
    out = copy.deepcopy(DEFAULT_GAMEPAD_MAPPING)
    if not isinstance(raw, dict):
        return out

    try:
        raw_ver = int(raw.get("schema_version", 1))
    except (TypeError, ValueError):
        raw_ver = 1

    preset = str(raw.get("axis_preset", out["axis_preset"])).lower()
    if preset in ("bumper_servo", "sticks", "triggers_bumpers", "custom"):
        out["axis_preset"] = preset

    raw_axes = raw.get("axes")
    axes_cfg: Dict[str, str] = dict(out["axes"])  # type: ignore[arg-type]
    if isinstance(raw_axes, dict):
        for ax in AXIS_KEYS:
            cand = str(raw_axes.get(ax, axes_cfg[ax])).lower().strip()
            if cand in AXIS_SOURCES:
                axes_cfg[ax] = cand
    elif preset in AXIS_PRESETS:
        axes_cfg = dict(AXIS_PRESETS[preset])
    out["axes"] = axes_cfg

    # Derive matching preset tag automatically if axes match a known preset
    matched_preset = "custom"
    for p_name, p_axes in AXIS_PRESETS.items():
        if axes_cfg == p_axes:
            matched_preset = p_name
            break
    out["axis_preset"] = matched_preset

    for k in ("invert_x", "invert_y", "invert_z", "invert_a", "z_guard_enabled"):
        if k in raw:
            out[k] = bool(raw[k])

    grip_mode = str(raw.get("gripper_mode", out["gripper_mode"])).lower()
    if grip_mode in ("hold", "toggle"):
        out["gripper_mode"] = grip_mode

    t_mode = str(raw.get("telemetry_mode", out["telemetry_mode"])).lower()
    if t_mode in ("suspended", "low_rate"):
        out["telemetry_mode"] = t_mode

    raw_btns = raw.get("buttons")
    if isinstance(raw_btns, dict):
        btns: Dict[str, str] = dict(out["buttons"])  # type: ignore[arg-type]
        for action in ACTION_KEYS:
            cand = str(raw_btns.get(action, btns[action])).upper().strip()
            if cand in VALID_BUTTON_CHOICES:
                btns[action] = cand
        out["buttons"] = btns

    def _clamp_float(val: object, lo: float, hi: float, default: float) -> float:
        try:
            f = float(val)  # type: ignore[arg-type]
            if f != f:  # NaN
                return default
            return round(max(lo, min(hi, f)), 3)
        except (TypeError, ValueError):
            return default

    if raw_ver < 2:
        # Migrate v1 configs so existing speeds (~45/65/14/60) correspond to 65% of the new ceilings
        prev_spd = _clamp_float(raw.get("speed_scale"), 0.10, 1.00, 1.0)
        out["speed_scale"] = round(max(0.10, min(1.00, prev_spd * 0.65)), 2)
    else:
        out["speed_scale"] = _clamp_float(raw.get("speed_scale"), 0.10, 1.00, 0.65)
        out["max_vx"] = _clamp_float(raw.get("max_vx"), 5.0, 60.0, 60.0)
        out["max_vy"] = _clamp_float(raw.get("max_vy"), 5.0, 100.0, 100.0)
        out["max_vz_up"] = _clamp_float(raw.get("max_vz_up"), 5.0, 35.0, 35.0)
        out["max_vz_down"] = _clamp_float(raw.get("max_vz_down"), 3.0, 30.0, 22.0)
        out["max_va"] = _clamp_float(raw.get("max_va"), 10.0, 95.0, 92.0)

    out["schema_version"] = 2
    out["deadzone"] = _clamp_float(raw.get("deadzone"), 0.02, 0.40, 0.12)
    out["curve_exp"] = _clamp_float(raw.get("curve_exp"), 1.0, 3.5, 2.2)
    out["precision_scale"] = _clamp_float(raw.get("precision_scale"), 0.10, 0.80, 0.25)
    out["z_guard_force_g"] = _clamp_float(raw.get("z_guard_force_g"), 40.0, 350.0, 150.0)
    return out


def load_gamepad_mapping(path: Path) -> Dict[str, object]:
    try:
        if path.is_file():
            raw = json.loads(path.read_text(encoding="utf-8"))
            clean = normalize_gamepad_mapping(raw)
            if not isinstance(raw, dict) or int(raw.get("schema_version", 1)) < 2:
                save_gamepad_mapping(path, clean)
            return clean
    except Exception as exc:
        logger.warning("Failed to load gamepad mapping from %s: %s", path, exc)
    return copy.deepcopy(DEFAULT_GAMEPAD_MAPPING)


def save_gamepad_mapping(path: Path, mapping: dict) -> Dict[str, object]:
    clean = normalize_gamepad_mapping(mapping)
    try:
        path.write_text(json.dumps(clean, indent=2) + "\n", encoding="utf-8")
    except Exception as exc:
        logger.warning("Failed to save gamepad mapping to %s: %s", path, exc)
    return clean


@dataclass
class GamepadSnapshot:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    lt: float = 0.0
    rt: float = 0.0
    dpad_x: float = 0.0
    dpad_y: float = 0.0
    buttons: Dict[str, bool] = field(default_factory=dict)
    timestamp_mono: float = 0.0
    source: str = "none"


@dataclass
class TeleopCommand:
    vx_mm_s: float = 0.0
    vy_mm_s: float = 0.0
    vz_mm_s: float = 0.0
    va_deg_s: float = 0.0
    precision_active: bool = False
    grip_open_desired: bool = False
    trigger_weigh: bool = False
    trigger_home: bool = False
    trigger_soft_estop: bool = False
    trigger_toggle_mode: bool = False
    any_motion: bool = False


def _shape_axis(raw_val: float, deadzone: float, exponent: float) -> float:
    mag = abs(raw_val)
    if mag <= deadzone:
        return 0.0
    scaled = (mag - deadzone) / max(1e-4, 1.0 - deadzone)
    shaped = min(1.0, max(0.0, scaled)) ** exponent
    return shaped if raw_val >= 0.0 else -shaped


class TeleopInputMapper:
    """Stateful mapper converting raw controller snapshots into gantry velocities & edges."""

    def __init__(self) -> None:
        self._prev_buttons: Dict[str, bool] = {}
        self._grip_toggle_open: bool = False
        self._grip_latched_off_until_release: bool = False

    def reset(self, preserve_buttons: bool = True) -> None:
        if not preserve_buttons:
            self._prev_buttons.clear()
        self._grip_toggle_open = False
        self._grip_latched_off_until_release = False

    def latch_grip_closed_until_next_input(self) -> None:
        """Forces gripper OFF after a thermal watchdog trip until a fresh button press."""
        self._grip_toggle_open = False
        self._grip_latched_off_until_release = True

    def map_input(self, snap: GamepadSnapshot, mapping: Dict[str, object]) -> TeleopCommand:
        btns_cfg: Dict[str, str] = mapping.get("buttons", {})  # type: ignore[assignment]
        cur_btns: Dict[str, bool] = dict(snap.buttons)
        if "LT" not in cur_btns:
            cur_btns["LT"] = snap.lt >= 0.5
        if "RT" not in cur_btns:
            cur_btns["RT"] = snap.rt >= 0.5

        def is_pressed(action: str) -> bool:
            bname = btns_cfg.get(action, "")
            return bool(cur_btns.get(bname, False))

        def rising_edge(action: str) -> bool:
            bname = btns_cfg.get(action, "")
            now_on = bool(cur_btns.get(bname, False))
            was_on = bool(self._prev_buttons.get(bname, False))
            return now_on and not was_on

        dz = float(mapping.get("deadzone", 0.12))  # type: ignore[arg-type]
        exp = float(mapping.get("curve_exp", 2.2))  # type: ignore[arg-type]
        prec_scale = float(mapping.get("precision_scale", 0.25))  # type: ignore[arg-type]
        speed_scale = float(mapping.get("speed_scale", 1.0))  # type: ignore[arg-type]

        sources = {
            "ls_x": _shape_axis(snap.lx, dz, exp),
            # Sticks push forward/up -> negative Y in evdev & W3C Gamepad API; map forward to +Y.
            "ls_y": _shape_axis(-snap.ly, dz, exp),
            "rs_x": _shape_axis(snap.rx, dz, exp),
            # Right stick up (-ry) lifts EE (-Z_down); down (+ry) lowers EE (+Z_down).
            "rs_y": _shape_axis(snap.ry, dz, exp),
            "bumpers": (1.0 if cur_btns.get("RB", False) else 0.0) - (1.0 if cur_btns.get("LB", False) else 0.0),
            "triggers": _shape_axis(snap.rt, dz * 0.5, exp) - _shape_axis(snap.lt, dz * 0.5, exp),
            "dpad_x": max(-1.0, min(1.0, float(snap.dpad_x))),
            "dpad_y": max(-1.0, min(1.0, float(snap.dpad_y))),
            "none": 0.0,
        }

        axes_cfg: Dict[str, str] = mapping.get("axes", {})  # type: ignore[assignment]
        x_norm = sources.get(str(axes_cfg.get("x", "ls_x")), 0.0)
        y_norm = sources.get(str(axes_cfg.get("y", "ls_y")), 0.0)
        z_norm = sources.get(str(axes_cfg.get("z", "rs_y")), 0.0)
        a_norm = sources.get(str(axes_cfg.get("a", "bumpers")), 0.0)

        if mapping.get("invert_x"):
            x_norm = -x_norm
        if mapping.get("invert_y"):
            y_norm = -y_norm
        if mapping.get("invert_z"):
            z_norm = -z_norm
        if mapping.get("invert_a"):
            a_norm = -a_norm

        precision_active = is_pressed("precision_hold")
        eff_scale = speed_scale * (prec_scale if precision_active else 1.0)

        max_vx = float(mapping.get("max_vx", 60.0))  # type: ignore[arg-type]
        max_vy = float(mapping.get("max_vy", 100.0))  # type: ignore[arg-type]
        max_vz_up = float(mapping.get("max_vz_up", 35.0))  # type: ignore[arg-type]
        max_vz_down = float(mapping.get("max_vz_down", 22.0))  # type: ignore[arg-type]
        max_va = float(mapping.get("max_va", 92.0))  # type: ignore[arg-type]

        vx = x_norm * max_vx * eff_scale
        vy = y_norm * max_vy * eff_scale
        vz = (z_norm * max_vz_down if z_norm >= 0.0 else z_norm * max_vz_up) * eff_scale
        va = a_norm * max_va * eff_scale

        if str(mapping.get("gripper_mode", "hold")) == "toggle":
            if rising_edge("grip_open"):
                if self._grip_latched_off_until_release:
                    self._grip_latched_off_until_release = False
                    self._grip_toggle_open = True
                else:
                    self._grip_toggle_open = not self._grip_toggle_open
            grip_open = False if self._grip_latched_off_until_release else self._grip_toggle_open
        else:
            held = is_pressed("grip_open")
            if self._grip_latched_off_until_release:
                if not held:
                    self._grip_latched_off_until_release = False
                grip_open = False
            else:
                grip_open = held

        cmd = TeleopCommand(
            vx_mm_s=vx,
            vy_mm_s=vy,
            vz_mm_s=vz,
            va_deg_s=va,
            precision_active=precision_active,
            grip_open_desired=grip_open,
            trigger_weigh=rising_edge("weigh"),
            trigger_home=rising_edge("home_all"),
            trigger_soft_estop=rising_edge("soft_estop"),
            trigger_toggle_mode=rising_edge("toggle_mode"),
            any_motion=(abs(vx) > 0.2 or abs(vy) > 0.2 or abs(vz) > 0.2 or abs(va) > 0.5),
        )
        self._prev_buttons = cur_btns
        return cmd


class XboxGamepadReader:
    """
    Background Linux `evdev` reader supporting Xbox Series X|S over Bluetooth BLE
    and wired USB-C on the Raspberry Pi Zero W.
    """

    def __init__(
        self,
        on_connect: Optional[Callable[[str, str], None]] = None,
        on_disconnect: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._snap = GamepadSnapshot()
        self._connected = False
        self._device_name = ""
        self._transport = "none"
        self._device = None
        self._rumble_id: Optional[int] = None
        self._last_button = ""
        self._last_button_seq = 0

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def evdev_supported(self) -> bool:
        return EVDEV_AVAILABLE

    def status_dict(self) -> dict:
        with self._lock:
            return {
                "evdev_supported": EVDEV_AVAILABLE,
                "connected": self._connected,
                "device_name": self._device_name,
                "transport": self._transport,
                "last_button": self._last_button,
                "last_button_seq": self._last_button_seq,
            }

    def get_snapshot(self) -> GamepadSnapshot:
        with self._lock:
            return GamepadSnapshot(
                lx=self._snap.lx,
                ly=self._snap.ly,
                rx=self._snap.rx,
                ry=self._snap.ry,
                lt=self._snap.lt,
                rt=self._snap.rt,
                dpad_x=self._snap.dpad_x,
                dpad_y=self._snap.dpad_y,
                buttons=dict(self._snap.buttons),
                timestamp_mono=self._snap.timestamp_mono,
                source=self._snap.source,
            )

    def start(self) -> None:
        if not EVDEV_AVAILABLE:
            logger.info("Linux evdev not available on this host; relying on Browser Gamepad API if connected.")
            return
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._reader_loop,
            name="xbox-evdev-reader",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._close_device()

    def rumble(self, strong: float = 0.8, weak: float = 0.4, duration_ms: int = 180) -> None:
        if not EVDEV_AVAILABLE:
            return
        dev = self._device
        if dev is None:
            return
        try:
            rumble = ff.Rumble(
                strong_magnitude=int(max(0.0, min(1.0, strong)) * 0xFFFF),
                weak_magnitude=int(max(0.0, min(1.0, weak)) * 0xFFFF),
            )
            effect = ff.Effect(
                ecodes.FF_RUMBLE,
                -1 if self._rumble_id is None else self._rumble_id,
                0,
                ff.Trigger(0, 0),
                ff.Replay(int(max(40, min(1500, duration_ms))), 0),
                ff.EffectType(ff_rumble_effect=rumble),
            )
            self._rumble_id = dev.upload_effect(effect)
            dev.write(ecodes.EV_FF, self._rumble_id, 1)
        except Exception:
            pass

    def _close_device(self) -> None:
        dev = self._device
        self._device = None
        self._rumble_id = None
        if dev is not None:
            try:
                dev.close()
            except Exception:
                pass

    def _find_xbox_device(self):
        if not EVDEV_AVAILABLE:
            return None
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                name_l = (dev.name or "").lower()
                caps = dev.capabilities()
                has_abs = ecodes.EV_ABS in caps
                has_key = ecodes.EV_KEY in caps
                abs_codes = {c[0] if isinstance(c, tuple) else c for c in caps.get(ecodes.EV_ABS, [])}
                key_codes = set(caps.get(ecodes.EV_KEY, []))
                is_gamepad = has_abs and has_key and ecodes.ABS_X in abs_codes and ecodes.ABS_Y in abs_codes and (
                    ecodes.BTN_SOUTH in key_codes or ecodes.BTN_GAMEPAD in key_codes
                )
                is_xbox_name = any(tok in name_l for tok in ("xbox", "x-box", "microsoft", "controller", "gamepad"))
                if is_gamepad or (is_xbox_name and has_abs and has_key):
                    return dev
                dev.close()
            except Exception:
                continue
        return None

    def _build_key_map(self) -> Dict[int, str]:
        m: Dict[int, str] = {}
        for code_name, btn in (
            ("BTN_SOUTH", "A"),
            ("BTN_A", "A"),
            ("BTN_EAST", "B"),
            ("BTN_B", "B"),
            ("BTN_NORTH", "X"),
            ("BTN_X", "X"),
            ("BTN_WEST", "Y"),
            ("BTN_Y", "Y"),
            ("BTN_TL", "LB"),
            ("BTN_TR", "RB"),
            ("BTN_SELECT", "VIEW"),
            ("KEY_BACK", "VIEW"),
            ("BTN_START", "MENU"),
            ("KEY_MENU", "MENU"),
            ("BTN_THUMBL", "L3"),
            ("BTN_THUMBR", "R3"),
        ):
            code = getattr(ecodes, code_name, None)
            if code is not None:
                m[code] = btn
        return m

    def _normalize_signed(self, value: int, lo: int, hi: int) -> float:
        if hi <= lo:
            return 0.0
        mid = 0.5 * (lo + hi)
        half = 0.5 * (hi - lo)
        return max(-1.0, min(1.0, (value - mid) / half))

    def _normalize_trigger(self, value: int, lo: int, hi: int) -> float:
        if hi <= lo:
            return 0.0
        return max(0.0, min(1.0, (value - lo) / float(hi - lo)))

    def _reader_loop(self) -> None:
        key_map = self._build_key_map()
        abs_stick_map = {
            ecodes.ABS_X: "lx",
            ecodes.ABS_Y: "ly",
            getattr(ecodes, "ABS_RX", 3): "rx",
            getattr(ecodes, "ABS_RY", 4): "ry",
        }
        # Some BLE HID descriptors expose right stick on ABS_Z / ABS_RZ and triggers on ABS_BRAKE / ABS_GAS
        trig_codes = {
            getattr(ecodes, "ABS_BRAKE", 10): "lt",
            getattr(ecodes, "ABS_GAS", 9): "rt",
        }
        hat_codes = {
            getattr(ecodes, "ABS_HAT0X", 16): "dpad_x",
            getattr(ecodes, "ABS_HAT0Y", 17): "dpad_y",
        }

        while self._running:
            dev = self._find_xbox_device()
            if dev is None:
                time.sleep(1.0)
                continue

            name = dev.name or "Xbox Controller"
            bustype = getattr(dev.info, "bustype", 0)
            transport = "bluetooth" if bustype == 0x05 else "usb"
            caps_abs = dict(dev.capabilities().get(ecodes.EV_ABS, []))
            has_brake_gas = getattr(ecodes, "ABS_BRAKE", 10) in caps_abs or getattr(ecodes, "ABS_GAS", 9) in caps_abs

            stick_map = dict(abs_stick_map)
            local_trig_map = dict(trig_codes)
            if has_brake_gas:
                # BLE HOGP mapping: ABS_Z=RX, ABS_RZ=RY, ABS_BRAKE=LT, ABS_GAS=RT
                stick_map[ecodes.ABS_Z] = "rx"
                stick_map[ecodes.ABS_RZ] = "ry"
            else:
                # USB xpad mapping: ABS_Z=LT, ABS_RZ=RT, ABS_RX=RX, ABS_RY=RY
                local_trig_map[ecodes.ABS_Z] = "lt"
                local_trig_map[ecodes.ABS_RZ] = "rt"

            self._device = dev
            with self._lock:
                self._connected = True
                self._device_name = name
                self._transport = transport
                self._snap = GamepadSnapshot(timestamp_mono=time.monotonic(), source=transport)

            logger.info("Connected Xbox controller via %s: %s (%s)", transport.upper(), name, dev.path)
            if self._on_connect:
                self._on_connect(name, transport)

            try:
                while self._running:
                    r, _, _ = select.select([dev.fd], [], [], 0.25)
                    if not r:
                        continue
                    for event in dev.read():
                        now_m = time.monotonic()
                        if event.type == ecodes.EV_ABS:
                            info = caps_abs.get(event.code)
                            lo = info.min if info else -32768
                            hi = info.max if info else 32767
                            if event.code in local_trig_map:
                                attr = local_trig_map[event.code]
                                val = self._normalize_trigger(event.value, lo, hi)
                                with self._lock:
                                    setattr(self._snap, attr, val)
                                    btn_name = "LT" if attr == "lt" else "RT"
                                    prev_on = self._snap.buttons.get(btn_name, False)
                                    now_on = val >= 0.5
                                    self._snap.buttons[btn_name] = now_on
                                    self._snap.timestamp_mono = now_m
                                    if now_on and not prev_on:
                                        self._last_button = btn_name
                                        self._last_button_seq += 1
                            elif event.code in hat_codes:
                                attr = hat_codes[event.code]
                                val = 1.0 if event.value > 0 else (-1.0 if event.value < 0 else 0.0)
                                with self._lock:
                                    setattr(self._snap, attr, val)
                                    self._snap.timestamp_mono = now_m
                            elif event.code in stick_map:
                                attr = stick_map[event.code]
                                val = self._normalize_signed(event.value, lo, hi)
                                with self._lock:
                                    setattr(self._snap, attr, val)
                                    self._snap.timestamp_mono = now_m
                        elif event.type == ecodes.EV_KEY and event.code in key_map:
                            btn_name = key_map[event.code]
                            pressed = event.value != 0
                            with self._lock:
                                prev_on = self._snap.buttons.get(btn_name, False)
                                self._snap.buttons[btn_name] = pressed
                                self._snap.timestamp_mono = now_m
                                if pressed and not prev_on:
                                    self._last_button = btn_name
                                    self._last_button_seq += 1
            except OSError as exc:
                logger.warning("Xbox controller disconnected (%s): %s", name, exc)
            except Exception as exc:
                logger.warning("Xbox controller reader error: %s", exc)
            finally:
                self._close_device()
                with self._lock:
                    self._connected = False
                    self._device_name = ""
                    self._transport = "none"
                    self._snap = GamepadSnapshot()
                if self._on_disconnect:
                    self._on_disconnect(name)
                time.sleep(1.0)


class XboxBluetoothManager:
    """
    Non-blocking BlueZ (`bluetoothctl`) helper allowing the PC Dashboard to
    reconnect a previously paired Xbox controller or re-pair one that was
    recently paired with a console/PC.
    """

    def __init__(self, on_status_change: Optional[Callable[[dict], None]] = None) -> None:
        self._on_status_change = on_status_change
        self._lock = threading.Lock()
        self._busy = False
        self._status: Dict[str, object] = {
            "state": "idle",
            "message": "",
            "target_mac": "",
            "paired_mac": "",
            "updated_ts": 0.0,
        }

    def status_dict(self) -> Dict[str, object]:
        with self._lock:
            return dict(self._status)

    def _set_status(
        self,
        state: str,
        message: str,
        target_mac: Optional[str] = None,
        paired_mac: Optional[str] = None,
    ) -> None:
        with self._lock:
            self._status["state"] = state
            self._status["message"] = message
            if target_mac is not None:
                self._status["target_mac"] = target_mac
            if paired_mac is not None:
                self._status["paired_mac"] = paired_mac
            self._status["updated_ts"] = round(time.time(), 2)
            snap = dict(self._status)
        if self._on_status_change:
            try:
                self._on_status_change(snap)
            except Exception:
                pass

    @staticmethod
    def _run_bt(args: List[str], timeout_s: float = 10.0) -> Tuple[int, str]:
        try:
            proc = subprocess.run(
                ["bluetoothctl", *args],
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
            return proc.returncode, (proc.stdout or "") + (proc.stderr or "")
        except Exception as exc:
            return 1, str(exc)

    def _get_xbox_devices(self) -> List[Tuple[str, str]]:
        rc, out = self._run_bt(["devices"], timeout_s=5.0)
        if rc != 0:
            return []
        found: List[Tuple[str, str]] = []
        for line in out.splitlines():
            m = re.match(r"^\s*Device\s+([0-9A-Fa-f:]{17})\s+(.+?)\s*$", line)
            if m:
                mac, name = m.group(1).upper(), m.group(2).strip()
                if any(tok in name.lower() for tok in ("xbox", "x-box", "wireless controller")):
                    found.append((mac, name))
        return found

    def _is_mac_connected(self, mac: str) -> bool:
        rc, out = self._run_bt(["info", mac], timeout_s=5.0)
        return rc == 0 and "Connected: yes" in out

    def reconnect_saved(self) -> bool:
        if sys.platform != "linux":
            self._set_status("error", "Pi Bluetooth manager only runs on Linux/Raspberry Pi.")
            return False
        with self._lock:
            if self._busy:
                return False
            self._busy = True
        threading.Thread(target=self._reconnect_worker, name="xbox-bt-reconnect", daemon=True).start()
        return True

    def pair_new_xbox(self, scan_timeout_s: float = 22.0) -> bool:
        if sys.platform != "linux":
            self._set_status("error", "Pi Bluetooth manager only runs on Linux/Raspberry Pi.")
            return False
        with self._lock:
            if self._busy:
                return False
            self._busy = True
        threading.Thread(
            target=self._pair_worker,
            args=(scan_timeout_s,),
            name="xbox-bt-pair",
            daemon=True,
        ).start()
        return True

    def _reconnect_worker(self) -> None:
        try:
            self._set_status("reconnecting", "Waking Pi Bluetooth and reconnecting saved Xbox controller...")
            self._run_bt(["power", "on"], timeout_s=5.0)
            devs = self._get_xbox_devices()
            if not devs:
                self._set_status(
                    "error",
                    "No saved Xbox controller found on Pi. Hold controller Pair button (3s) and click Pair / Re-Pair.",
                )
                return
            for mac, name in devs:
                self._set_status("reconnecting", f"Connecting {name} ({mac})...", target_mac=mac)
                self._run_bt(["trust", mac], timeout_s=5.0)
                self._run_bt(["connect", mac], timeout_s=12.0)
                time.sleep(0.8)
                if self._is_mac_connected(mac):
                    self._set_status("connected", f"Connected {name} ({mac}).", target_mac=mac, paired_mac=mac)
                    return
            self._set_status(
                "error",
                "Could not reconnect (controller asleep or paired elsewhere). Hold Pair button 3s & click Pair / Re-Pair.",
            )
        except Exception as exc:
            self._set_status("error", f"Bluetooth reconnect error: {exc}")
        finally:
            with self._lock:
                self._busy = False

    def _pair_worker(self, scan_timeout_s: float) -> None:
        proc = None
        try:
            self._set_status(
                "scanning",
                "Hold Xbox Pair button (3s until fast blink) — scanning for controller...",
                target_mac="",
            )
            self._run_bt(["power", "on"], timeout_s=5.0)
            for mac, _name in self._get_xbox_devices():
                if not self._is_mac_connected(mac):
                    self._run_bt(["remove", mac], timeout_s=5.0)

            proc = subprocess.Popen(
                ["bluetoothctl"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert proc.stdin is not None and proc.stdout is not None
            for cmd in ("power on", "agent on", "default-agent", "scan on"):
                proc.stdin.write(cmd + "\n")
                proc.stdin.flush()

            found_mac = ""
            found_name = ""
            deadline = time.monotonic() + scan_timeout_s
            pat = re.compile(r"Device\s+([0-9A-Fa-f:]{17})\s+(.*(?:Xbox|Wireless Controller).*)", re.IGNORECASE)
            while time.monotonic() < deadline:
                r, _, _ = select.select([proc.stdout], [], [], 0.35)
                if not r:
                    continue
                line = proc.stdout.readline()
                if not line:
                    break
                clean = re.sub(r"\x1b\[[0-9;]*[mK]", "", line).strip()
                m = pat.search(clean)
                if m:
                    cand_mac = m.group(1).upper()
                    cand_name = m.group(2).strip()
                    if "rssi:" in cand_name.lower() or "connected:" in cand_name.lower():
                        cand_name = "Xbox Wireless Controller"
                    found_mac = cand_mac
                    found_name = cand_name or "Xbox Wireless Controller"
                    break

            try:
                proc.stdin.write("scan off\nquit\n")
                proc.stdin.flush()
                proc.wait(timeout=3.0)
            except Exception:
                proc.kill()
            proc = None

            if not found_mac:
                self._set_status(
                    "error",
                    "No pairing Xbox controller found within 22s. Make sure Xbox logo is blinking fast and retry.",
                )
                return

            self._set_status("pairing", f"Found {found_name} ({found_mac}) — pairing & trusting...", target_mac=found_mac)
            self._run_bt(["pair", found_mac], timeout_s=15.0)
            self._run_bt(["trust", found_mac], timeout_s=6.0)
            self._run_bt(["connect", found_mac], timeout_s=12.0)

            for _ in range(6):
                if self._is_mac_connected(found_mac):
                    self._set_status(
                        "connected",
                        f"Paired & connected {found_name} ({found_mac})!",
                        target_mac=found_mac,
                        paired_mac=found_mac,
                    )
                    return
                time.sleep(0.5)

            self._set_status(
                "error",
                f"Paired {found_mac} but connection did not settle — click Reconnect or power-cycle controller.",
                target_mac=found_mac,
            )
        except Exception as exc:
            self._set_status("error", f"Bluetooth pairing failed: {exc}")
        finally:
            if proc is not None:
                try:
                    proc.kill()
                except Exception:
                    pass
            with self._lock:
                self._busy = False
