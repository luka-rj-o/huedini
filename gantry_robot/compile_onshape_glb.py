"""
Compiles and optimizes the Onshape bulk export (`V3 Gantry.glb`) into a compact,
60 FPS WebGL asset (`v3_gantry_compiled.bin` + `v3_gantry_compiled.json`).

Optimizations & Alignment:
1. Applies exact rigid Onshape GLB -> Plate/Three.js transform (det = +1):
   threeX = -X_glb_mm
   threeY =  Z_glb_mm
   threeZ =  Y_glb_mm - 16.2
   (eliminates the X-axis flip while keeping Fiducial/Course/Pedestal aligned to 0.05 mm)
2. Classifies all 114 occurrence nodes + EndEffectorv6Assem subtree into the 5
   kinematic stages (world, xStage, zStage, yStage, eeStage, plunger).
3. Performs normal-preserving spatial-grid vertex clustering on heavy COTS subparts
   (278k-vert spring, MGN12H ball-bearing assemblies, NEMA 11 motor internals) and
   merges all 5,271 primitives by (stage, material) into ~40–50 WebGL draw calls.
"""

import json
import struct
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Tuple
import numpy as np

SRC_GLB = Path(r"C:\Users\lukas\Downloads\V3 Gantry.glb")
OUT_DIR = Path(__file__).resolve().parent / "pc_dashboard" / "assets"
OUT_BIN = OUT_DIR / "v3_gantry_compiled.bin"
OUT_JSON = OUT_DIR / "v3_gantry_compiled.json"

# Exact CAD alignment constants derived from V3 Gantry.glb measurements:
# EE pocket & MG90 shaft center in GLB: X_glb=96.6, Y_glb=14.7 -> X_p=45.4, Y_p=61.5
# Compared to calibrated V3 Gantry homed EE_HOMED = (31.7, +6.4), Z_bottom = 70.36 mm:
CAD_MOTOR_X = 13.7
CAD_MOTOR_Y = 55.1
CAD_MOTOR_Z = 4.9

EE_PIVOT_THREE = (-96.6, 106.5, 14.7 - 16.2)  # (-96.6, 106.5, -1.5)
# Exact physical circular camera bore on bottom face of Load Cell Bracket inside yStageGroup:
CAM_LOCAL_THREE = (-95.63, 120.86, -40.73)

# Saturation-boosted palette overrides for washed-out pastel Onshape PLA colors
MATERIAL_COLOR_OVERRIDES = {
    1: [0.08, 0.64, 0.92],  # Accent cyan (originally [0.6157, 0.8118, 0.9294])
    3: [0.22, 0.72, 0.96],  # Sky-cyan PLA parts: Feet, Saddle, Bracket (originally [0.7686, 0.8863, 0.9529])
    8: [0.86, 0.36, 0.78],  # Pink/orchid PLA parts: X-Slider, Pulley Cap, EE_V5 (originally [0.8510, 0.7569, 0.8667])
}


def load_glb(path: Path) -> Tuple[Dict[str, Any], bytes]:
    with open(path, "rb") as f:
        magic, version, total_len = struct.unpack("<4sII", f.read(12))
        assert magic == b"glTF", f"Not a GLB file: {magic}"
        json_len, json_type = struct.unpack("<I4s", f.read(8))
        assert json_type == b"JSON"
        gltf = json.loads(f.read(json_len).decode("utf-8"))
        bin_len, bin_type = struct.unpack("<I4s", f.read(8))
        assert bin_type == b"BIN\x00"
        bin_bytes = f.read(bin_len)
    return gltf, bin_bytes


def node_matrix(node: Dict[str, Any]) -> np.ndarray:
    if "matrix" in node:
        return np.array(node["matrix"], dtype=np.float64).reshape((4, 4)).T
    M = np.eye(4, dtype=np.float64)
    if "translation" in node:
        M[:3, 3] = node["translation"]
    return M


def read_accessor(gltf: Dict[str, Any], bin_bytes: bytes, acc_idx: int) -> np.ndarray:
    acc = gltf["accessors"][acc_idx]
    bv = gltf["bufferViews"][acc["bufferView"]]
    offset = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    count = acc["count"]
    ctype = acc["componentType"]
    atype = acc["type"]

    dtype_map = {
        5120: np.int8,
        5121: np.uint8,
        5122: np.int16,
        5123: np.uint16,
        5125: np.uint32,
        5126: np.float32,
    }
    comp_map = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}
    dt = np.dtype(dtype_map[ctype])
    ncomp = comp_map[atype]
    stride = bv.get("byteStride")
    elem_bytes = dt.itemsize * ncomp

    if stride and stride != elem_bytes:
        raw = np.frombuffer(bin_bytes, dtype=np.uint8, count=count * stride, offset=offset)
        raw = raw.reshape(count, stride)[:, :elem_bytes]
        arr = np.frombuffer(raw.tobytes(), dtype=dt).reshape(count, ncomp)
    else:
        arr = np.frombuffer(bin_bytes, dtype=dt, count=count * ncomp, offset=offset)
        if ncomp > 1:
            arr = arr.reshape(count, ncomp)
    return arr.copy()


def classify_root_occurrence(idx: int, name: str, node_idx: int) -> str:
    """Deterministic kinematic stage assignment for V3 Gantry.glb root occurrences."""
    if idx in (18, 45) or (34 <= idx <= 44) or (58 <= idx <= 66) or (92 <= idx <= 95):
        return "world"
    if (5 <= idx <= 13) or (81 <= idx <= 91) or idx == 96:
        return "xStage"
    if (0 <= idx <= 3) or (14 <= idx <= 17) or (19 <= idx <= 33) or (46 <= idx <= 56) or idx == 57:
        return "zStage"
    if idx == 4 or (67 <= idx <= 80) or idx == 97 or (98 <= idx <= 112):
        return "yStage"
    if "endeffector" in name.lower():
        return "ee_subassembly"
    return "world"


def decimate_cell_mm_for_node(occ_idx: int, occ_name: str, sub_name: str, vert_count: int) -> Tuple[float, float]:
    """Choose (spatial_cell_mm, normal_quant_scale) based on component visual vs geometric complexity."""
    s = (occ_name + " " + sub_name).lower()
    if "fiducial" in s or "logo" in s or "course" in s or "baseplate" in s or "platform" in s or "endeffectorv5" in s:
        return 0.0, 0.0  # Exact crisp CAD geometry
    if 67 <= occ_idx <= 80 and occ_idx != 76:
        return 0.0, 0.0  # Exact "NO STEP - KEEP OFF" lettering
    if "spring_1692k49" in s or "circlip" in s:
        return 2.0, 0.0  # Tiny 8mm coil spring -> lightweight silhouette
    if "plunger" in s or "microservo horn" in s or "coil_défaut" in s:
        return 1.1, 0.5
    if (19 <= occ_idx <= 33) or (98 <= occ_idx <= 112):
        return 1.8, 0.45  # MGN12H bearing carriages + internal steel balls
    if (34 <= occ_idx <= 44) or (46 <= occ_idx <= 56) or (81 <= occ_idx <= 91):
        return 1.8, 0.45  # NEMA 11 motor housings & screw heads
    if "pulley cap" in s or "x-axis foot" in s or "x-axis cover" in s or "linear shaft" in s or "mg90servo" in s:
        return 1.5, 0.5
    if "load cell bracket" in s or "y-axis" in s or "z-motor" in s or "z-axis" in s or "x-axis slider" in s or "belt tensioner" in s or "linear rail" in s:
        return 1.0, 0.55
    if vert_count > 4000:
        return 0.85, 0.55
    return 0.0, 0.0


def cluster_decimate(pos: np.ndarray, nrm: np.ndarray, idx: np.ndarray, cell_mm: float, nrm_scale: float = 0.7) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Fast normal-aware vertex clustering that preserves sharp CAD edges & silhouettes."""
    if cell_mm <= 0.0 or len(pos) < 60:
        return pos, nrm, idx

    q_pos = np.round(pos / cell_mm).astype(np.int64)
    if nrm_scale > 0.0:
        q_nrm = np.round(nrm * nrm_scale).astype(np.int64)
        keys = np.hstack([q_pos, q_nrm])
    else:
        keys = q_pos

    # Structured view for fast vectorized np.unique over 6-int keys
    key_view = np.ascontiguousarray(keys).view(
        np.dtype((np.void, keys.dtype.itemsize * keys.shape[1]))
    ).ravel()
    _, first_occ, inverse = np.unique(key_view, return_index=True, return_inverse=True)

    new_idx = inverse[idx].reshape(-1, 3)
    valid = (new_idx[:, 0] != new_idx[:, 1]) & (new_idx[:, 1] != new_idx[:, 2]) & (new_idx[:, 0] != new_idx[:, 2])
    new_idx = new_idx[valid]
    if len(new_idx) == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.uint32)

    # Average positions & normals per cluster
    n_verts = len(first_occ)
    new_pos = np.zeros((n_verts, 3), dtype=np.float64)
    new_nrm = np.zeros((n_verts, 3), dtype=np.float64)
    counts = np.bincount(inverse, minlength=n_verts)[:, None]
    for c in range(3):
        new_pos[:, c] = np.bincount(inverse, weights=pos[:, c], minlength=n_verts)
        new_nrm[:, c] = np.bincount(inverse, weights=nrm[:, c], minlength=n_verts)
    new_pos /= np.maximum(counts, 1)
    norms = np.linalg.norm(new_nrm, axis=1, keepdims=True)
    new_nrm = np.divide(new_nrm, np.maximum(norms, 1e-8))

    # Compact to only vertices referenced by surviving triangles
    used, compact_inv = np.unique(new_idx.ravel(), return_inverse=True)
    return (
        new_pos[used].astype(np.float32),
        new_nrm[used].astype(np.float32),
        compact_inv.astype(np.uint32),
    )


def main() -> None:
    gltf, bin_bytes = load_glb(SRC_GLB)
    nodes = gltf["nodes"]
    meshes = gltf["meshes"]
    materials = gltf.get("materials", [])
    root_children = nodes[0]["children"]

    # Transform matrix from GLB meters (X, Y, Z) -> Three.js mm (threeX, threeY, threeZ)
    # threeX = -X_m * 1000, threeY = Z_m * 1000, threeZ = Y_m * 1000 - 16.2
    T_glb_to_three = np.array([
        [-1000.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1000.0, 0.0],
        [0.0, 1000.0, 0.0, -16.2],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)
    R_norm = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
    ], dtype=np.float64)

    # Accumulate (positions, normals, indices) per (stage, mat_idx)
    buckets: Dict[Tuple[str, int], List[Tuple[np.ndarray, np.ndarray, np.ndarray]]] = defaultdict(list)
    raw_prims = 0
    raw_tris = 0

    def process_mesh_node(n_idx: int, M_glb: np.ndarray, stage: str, occ_idx: int, occ_name: str) -> None:
        nonlocal raw_prims, raw_tris
        n = nodes[n_idx]
        sub_name = n.get("name", "")
        M_world = T_glb_to_three @ M_glb
        R_world = R_norm @ M_glb[:3, :3]

        if "mesh" in n:
            m = meshes[n["mesh"]]
            prims = m.get("primitives", [])
            # Group primitives of this single CAD part by material before optional decimation
            part_by_mat: Dict[int, List[Tuple[np.ndarray, np.ndarray, np.ndarray]]] = defaultdict(list)
            part_verts = 0
            for p in prims:
                raw_prims += 1
                mat_idx = int(p.get("material", 0))
                pos_acc = p["attributes"]["POSITION"]
                pos_local = read_accessor(gltf, bin_bytes, pos_acc).astype(np.float64)
                part_verts += len(pos_local)

                if "NORMAL" in p["attributes"]:
                    nrm_local = read_accessor(gltf, bin_bytes, p["attributes"]["NORMAL"]).astype(np.float64)
                    nrm_w = (R_world @ nrm_local.T).T
                else:
                    nrm_w = np.tile([0.0, 1.0, 0.0], (len(pos_local), 1))

                hom = np.hstack([pos_local, np.ones((len(pos_local), 1))])
                pos_w = (M_world @ hom.T).T[:, :3]

                # Shift eeStage & plunger into EE_PIVOT local coordinates for θ rotation
                if stage in ("eeStage", "plunger"):
                    pos_w[:, 0] -= EE_PIVOT_THREE[0]
                    pos_w[:, 1] -= EE_PIVOT_THREE[1]
                    pos_w[:, 2] -= EE_PIVOT_THREE[2]

                if "indices" in p:
                    idx = read_accessor(gltf, bin_bytes, p["indices"]).astype(np.uint32).ravel()
                else:
                    idx = np.arange(len(pos_w), dtype=np.uint32)

                raw_tris += len(idx) // 3
                part_by_mat[mat_idx].append((pos_w, nrm_w, idx))

            cell_mm, nrm_scale = decimate_cell_mm_for_node(occ_idx, occ_name, sub_name, part_verts)
            for mat_idx, chunks in part_by_mat.items():
                # Concatenate all face primitives of this part for the given material
                pos_cat, nrm_cat, idx_cat = merge_chunks(chunks)
                if cell_mm > 0.0:
                    pos_cat, nrm_cat, idx_cat = cluster_decimate(pos_cat, nrm_cat, idx_cat, cell_mm, nrm_scale)
                if len(idx_cat) > 0:
                    buckets[(stage, mat_idx)].append((pos_cat, nrm_cat, idx_cat))

        for c in n.get("children", []):
            M_child = M_glb @ node_matrix(nodes[c])
            process_mesh_node(c, M_child, stage, occ_idx, occ_name)

    def merge_chunks(chunks: List[Tuple[np.ndarray, np.ndarray, np.ndarray]]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pos_list, nrm_list, idx_list = [], [], []
        v_off = 0
        for p, n, ix in chunks:
            pos_list.append(p)
            nrm_list.append(n)
            idx_list.append(ix + v_off)
            v_off += len(p)
        return (
            np.vstack(pos_list).astype(np.float32),
            np.vstack(nrm_list).astype(np.float32),
            np.concatenate(idx_list).astype(np.uint32),
        )

    for occ_idx, c_idx in enumerate(root_children):
        cn = nodes[c_idx]
        occ_name = cn.get("name", f"Part_{occ_idx}")
        stage = classify_root_occurrence(occ_idx, occ_name, c_idx)
        M_occ = node_matrix(cn)

        if stage != "ee_subassembly":
            process_mesh_node(c_idx, M_occ, stage, occ_idx, occ_name)
        else:
            # EndEffectorv6Assem <1>: split MG90 body (yStage), Plunger (plunger), and gripper (eeStage)
            for ee_child in cn.get("children", []):
                ecn = nodes[ee_child]
                ename = ecn.get("name", "")
                M_ee_child = M_occ @ node_matrix(ecn)
                if "mg90servo_assembly" in ename.lower():
                    for s_child in ecn.get("children", []):
                        sn = nodes[s_child]
                        sname = sn.get("name", "")
                        M_sc = M_ee_child @ node_matrix(sn)
                        sub_stage = "yStage" if "body" in sname.lower() else "eeStage"
                        process_mesh_node(s_child, M_sc, sub_stage, occ_idx, sname)
                elif "plunger" in ename.lower():
                    process_mesh_node(ee_child, M_ee_child, "plunger", occ_idx, ename)
                else:
                    process_mesh_node(ee_child, M_ee_child, "eeStage", occ_idx, ename)

    # Write packed binary buffer & JSON manifest
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    mat_list = []
    for mat_idx, m in enumerate(materials):
        pbr = m.get("pbrMetallicRoughness", {})
        col = pbr.get("baseColorFactor", [0.85, 0.85, 0.85, 1.0])
        rgb = MATERIAL_COLOR_OVERRIDES.get(mat_idx, [round(float(c), 4) for c in col[:3]])
        mat_list.append({
            "color": list(rgb),
            "alpha": round(float(col[3]), 3),
            "metalness": round(float(pbr.get("metallicFactor", 0.0)), 3),
            "roughness": round(float(pbr.get("roughnessFactor", 0.45)), 3),
        })

    draw_calls = []
    bin_chunks = []
    byte_offset = 0
    total_out_verts = 0
    total_out_tris = 0

    for (stage, mat_idx), chunks in sorted(buckets.items()):
        pos, nrm, idx = merge_chunks(chunks)
        # Ensure 4-byte alignment
        pos_b = pos.tobytes()
        nrm_b = nrm.tobytes()
        idx_b = idx.tobytes()

        p_off = byte_offset
        bin_chunks.append(pos_b)
        byte_offset += len(pos_b)

        n_off = byte_offset
        bin_chunks.append(nrm_b)
        byte_offset += len(nrm_b)

        i_off = byte_offset
        bin_chunks.append(idx_b)
        byte_offset += len(idx_b)

        v_cnt = len(pos)
        t_cnt = len(idx) // 3
        total_out_verts += v_cnt
        total_out_tris += t_cnt

        draw_calls.append({
            "stage": stage,
            "material": mat_idx,
            "vertexCount": v_cnt,
            "triangleCount": t_cnt,
            "posOffset": p_off,
            "nrmOffset": n_off,
            "idxOffset": i_off,
            "idxCount": len(idx),
        })

    with open(OUT_BIN, "wb") as f:
        for ch in bin_chunks:
            f.write(ch)

    manifest = {
        "source": SRC_GLB.name,
        "cad_motor_x_mm": CAD_MOTOR_X,
        "cad_motor_y_mm": CAD_MOTOR_Y,
        "cad_motor_z_mm": CAD_MOTOR_Z,
        "ee_pivot_three": list(EE_PIVOT_THREE),
        "cam_local_three": list(CAM_LOCAL_THREE),
        "materials": mat_list,
        "draw_calls": draw_calls,
        "stats": {
            "raw_primitives": raw_prims,
            "raw_triangles": raw_tris,
            "compiled_draw_calls": len(draw_calls),
            "compiled_vertices": total_out_verts,
            "compiled_triangles": total_out_tris,
            "bin_bytes": byte_offset,
        },
    }
    OUT_JSON.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print(f"Compiled {SRC_GLB.name} successfully!")
    print(f"  Raw:      {raw_prims:,} draw-call primitives, {raw_tris:,} triangles ({SRC_GLB.stat().st_size:,} bytes)")
    print(f"  Compiled: {len(draw_calls):,} draw calls, {total_out_tris:,} triangles, {total_out_verts:,} verts ({byte_offset:,} bytes)")
    for st in ("world", "xStage", "zStage", "yStage", "eeStage", "plunger"):
        st_dc = [d for d in draw_calls if d["stage"] == st]
        print(f"    {st:<8}: {len(st_dc):2d} draw calls, {sum(d['triangleCount'] for d in st_dc):6,} tris")


if __name__ == "__main__":
    main()
