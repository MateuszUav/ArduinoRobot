import sys, time, glob, serial

LOG = []
BAUD = 115200
PORT = "/dev/tty.usbmodem1201"
HELP_TEXT = """  HELP  •  Frame format:
PAYLOAD*CS\\n
where:
    PAYLOAD = ASCII (e.g., PING, M,120, R,-90, S)
    * = separator
    CS = 2-byte hex (uppercase) XOR of all bytes in PAYLOAD
    \\n = newline (LF, 0x0A) terminator
    Allowed chars in PAYLOAD: avoid * and \\n (keep it simple).
    Responses:
      Success: OK[,optional_data]*CS\\n (e.g., OK*xx\\n, OK,STATUS=READY*xx\\n)
      Error: ERR,<code>[,info]*CS\\n (codes: BAD_CHK, BAD_CMD, BAD_ARG)
    Examples:
      PC → PING*38\\n
      Arduino → OK*4F\\n
      PC → M,100*7C\\n (move 100 cm)
      Arduino → OK*4F\\n
"""
STX = 0x02
ETX = 0x03

def xor_checksum(data: bytes) -> int:
    cs = 0
    for b in data:
        cs ^= b
    return cs & 0xFF

def send_frame(ser: serial.Serial, payload: str) -> None:
    data = payload.encode("ascii", errors="ignore")
    pkt = bytes([STX]) + data + bytes([ETX, xor_checksum(data)])
    ser.write(pkt)

def read_frame(ser: serial.Serial, timeout=1.5) -> str | None:
    """Reads one framed message. Returns payload string or None on timeout/CHK error."""
    t0 = time.time()

    # 1) wait STX
    while True:
        if time.time() - t0 > timeout:
            return None
        b = ser.read(1)
        if not b:
            continue
        if b[0] == STX:
            break

    # 2) read until ETX (payload)
    payload = bytearray()
    while True:
        if time.time() - t0 > timeout:
            return None
        b = ser.read(1)
        if not b:
            continue
        if b[0] == ETX:
            break
        payload.append(b[0])

    # 3) read checksum byte
    csb = ser.read(1)
    if not csb:
        return None
    cs_recv = csb[0]
    cs_calc = xor_checksum(payload)
    if cs_recv != cs_calc:
        return None

    return payload.decode("ascii", errors="replace")

# === UTILS ===

def list_serial_ports():
    if sys.platform.startswith("win"):
        return [f"COM{i}" for i in range(1, 256)]
    if sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        import glob as _glob
        return _glob.glob("/dev/ttyUSB*") + _glob.glob("/dev/ttyACM*")
    if sys.platform.startswith("darwin"):
        import glob as _glob
        return _glob.glob("/dev/tty.usb*") + _glob.glob("/dev/tty.wchusb*") + _glob.glob("/dev/tty.acm*")
    return []

def choose_port_and_speed():
    global BAUD, PORT
    ports = list_serial_ports()
    print("Available ports:", ports or "(none found)")
    port = input(f"Port def:[{PORT}]: ").strip()
    if port:
        PORT = port
    bs = input(f"Baud def:[{BAUD}]: ").strip()
    if bs:
        BAUD = int(bs)
    return PORT

def main():
    port = choose_port_and_speed()

    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open {port}: {exc}")
        return

    # Let the MCU reset on DTR toggle (common on Arduinos)
    time.sleep(2.0)
    print(f"Connected on {ser.port} @ {BAUD}. Type 'help' for info, 'quit' to exit.")

    # Optional: read any initial greeting (if your Arduino sends one)
    ser.flushInput()
    t_end = time.time() + 0.5
    while time.time() < t_end:
        msg = read_frame(ser, timeout=0.1)
        if msg:
            print(f"< {msg}")

    try:
        while True:
            try:
                line = input(">> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[EXIT]")
                break

            if not line:
                continue

            lo = line.lower()
            if lo in ("quit", "exit", "q"):
                print("[Bye]")
                break

            if lo == "help":
                print(HELP_TEXT)
                continue

            # Send payload as-typed
            send_frame(ser, line)

            # Wait for a single response frame (adjust timeout if needed)
            resp = read_frame(ser, timeout=10)
            if resp is None:
                print("< [timeout / no valid frame]")
            else:
                print(f"< {resp}")

    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()