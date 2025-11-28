import serial
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO

# --- Serial config ---
SERIAL_PORT = "COM5"
BAUDRATE = 115200
TIMEOUT_S = 1

# --- Flask / SocketIO setup ---
app = Flask(__name__)
app.config["SECRET_KEY"] = "robot-network"
socketio = SocketIO(app, cors_allowed_origins="*")


def int8_from_byte(b: int) -> int:
    """Convert 0–255 byte to signed int8 (-128..127)."""
    return b - 256 if b >= 128 else b


def parse_packet(packet: bytes):
    """
    Layout after stripping \r\n:
      [0:8]   node ID
      [8:16]  neighbor ID
      [16]    RSSI (int8_t)
    Total 17 bytes.
    """
    if len(packet) != 17:
        print(f"[WARN] bad packet len={len(packet)} data={packet.hex()}")
        return None

    node_bytes = packet[0:8]
    neigh_bytes = packet[8:16]
    rssi_byte = packet[16]

    node_hex = node_bytes.hex()
    neigh_hex = neigh_bytes.hex()
    rssi = int8_from_byte(rssi_byte)

    return {
        "node_hex": node_hex,
        "neighbor_hex": neigh_hex,
        "rssi": rssi,
    }


def serial_reader():
    """Background thread: read serial and emit link updates."""
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT_S)
    print(f"[SERIAL] Opened {SERIAL_PORT} @ {BAUDRATE}")

    try:
        while True:
            line = ser.read_until(b"\r\n")
            if not line:
                continue

            data = line.rstrip(b"\r\n")
            pkt = parse_packet(data)
            if not pkt:
                continue

            # Send to all connected browsers
            socketio.emit("link_update", {
                "node": pkt["node_hex"],
                "neighbor": pkt["neighbor_hex"],
                "rssi": pkt["rssi"],
            })
    finally:
        ser.close()
        print("[SERIAL] Closed")


@app.route("/")
def index():
    # This expects templates/index.html (see next section)
    return render_template("index.html")


if __name__ == "__main__":
    # Start serial thread
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # Start web server
    socketio.run(app, host="0.0.0.0", port=5000)
