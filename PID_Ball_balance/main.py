
import sys, time, serial

STX = 0x02
ETX = 0x03
BAUD = 115200
PORT = "/dev/tty.usbserial-210"

HELP_TEXT = """Commands (type payload only; framing is added automatically):
  PING
  PID?
  PID <kp> <ki> <kd>
  DIST?
  SP <value_cm>
  STEP
  MAE
Other:
  help        Show this help
  ports       List available serial ports
  quit/exit/q Quit
Protocol:
  TX: STX + PAYLOAD(bytes) + ETX + XOR(PAYLOAD)
  RX: same framing, payload is ASCII
"""

def xor_checksum(data: bytes) -> int:
    cs = 0
    for b in data:
        cs ^= b
    return cs & 0xFF

def send_frame(ser: serial.Serial, payload: str) -> None:
    data = payload.encode("ascii", errors="ignore")
    pkt = bytes([STX]) + data + bytes([ETX, xor_checksum(data)])
    ser.write(pkt)

def read_frame(ser: serial.Serial, timeout: float = 5.0) -> str | None:
    ser.timeout = 0.1
    deadline = time.time() + timeout
    state = "WAIT_STX"
    payload = bytearray()

    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        byte = b[0]

        if state == "WAIT_STX":
            if byte == STX:
                payload.clear()
                state = "READ_PAYLOAD"
            # else keep waiting
        elif state == "READ_PAYLOAD":
            if byte == ETX:
                state = "WAIT_CSUM"
            else:
                payload.append(byte)
        elif state == "WAIT_CSUM":
            cs_recv = byte
            cs_calc = xor_checksum(payload)
            if cs_recv == cs_calc:
                try:
                    return payload.decode("ascii", errors="replace")
                finally:
                    payload.clear()
            else:
                return None  # checksum error
            # reset not strictly needed due to return
    return None  # timeout

def list_serial_ports():
    # Minimal cross-platform listing
    ports = []
    try:
        import glob
        if sys.platform.startswith("win"):
            ports = [f"COM{i}" for i in range(1, 256)]
        elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
            ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        elif sys.platform.startswith("darwin"):
            ports = glob.glob("/dev/tty.usb*") + glob.glob("/dev/tty.wchusb*") + glob.glob("/dev/tty.acm*")
    except Exception:
        pass
    return ports

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
    return PORT, BAUD

def main():
    port, baud = choose_port_and_speed()
    print(f"Opening {port} @ {baud} ...")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.1)
    except Exception as e:
        print(f"[Error] {e}")
        return

    print("Type 'help' for commands. Type 'quit' to exit.")

    try:
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[Bye]")
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

            if lo == "ports":
                print("Available ports:", list_serial_ports() or "(none found)")
                continue

            # Send payload as-typed (e.g., PING, PID?, PID 1.5 0.05 1.1, DIST?, SP 25, STEP)
            send_frame(ser, line)

            # Wait for a single response frame
            resp = read_frame(ser, timeout=5)
            if resp is None:
                print("< [timeout / checksum error / no valid frame]")
            else:
                print(f"< {resp}")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
