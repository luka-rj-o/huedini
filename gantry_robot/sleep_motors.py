import serial, time

try:
    s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
    time.sleep(0.5)
    s.reset_input_buffer()
    s.write(b"sleep\n")
    time.sleep(0.2)
    s.close()
    print("Motors put to sleep.")
except Exception as e:
    print(f"Error sleeping motors: {e}")
