import serial

# === CONFIGURE YOUR SERIAL PORT HERE ===
SERIAL_PORT = "COM5"       # default COM5 as you said
BAUDRATE    = 115200       # change if needed
TIMEOUT_S   = 1            # seconds


def int8_from_byte(b: int) -> int:
    """Convert an unsigned byte (0–255) to signed int8 (-128–127)."""
    return b - 256 if b >= 128 else b


def parse_packet(packet: bytes):
    """
    Packet layout (after stripping \r\n):
        [0:8]   node ID (8 bytes)
        [8:16]  neighbor ID (8 bytes)
        [16]    RSSI (1 byte, int8_t)
    Total: 17 bytes
    """
    if len(packet) != 17:
        print(f"[WARN] Unexpected packet length {len(packet)} (expected 17). Raw: {packet.hex()}")
        return None

    node_bytes   = packet[0:8]
    neigh_bytes  = packet[8:16]
    rssi_byte    = packet[16]

    node_hex   = node_bytes.hex()
    neigh_hex  = neigh_bytes.hex()
    node_int   = int.from_bytes(node_bytes,  byteorder="big")
    neigh_int  = int.from_bytes(neigh_bytes, byteorder="big")
    rssi       = int8_from_byte(rssi_byte)

    return {
        "node_hex": node_hex,
        "neighbor_hex": neigh_hex,
        "node_int": node_int,
        "neighbor_int": neigh_int,
        "rssi": rssi,
    }


def main():
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT_S
    )

    print(f"Opened {SERIAL_PORT} @ {BAUDRATE} baud. Waiting for packets... (Ctrl+C to exit)")

    try:
        while True:
            # Read until CRLF (matches your '\r' '\n' termination)
            line = ser.read_until(b"\r\n")
            if not line:
                continue  # timeout, nothing received

            # Strip trailing \r\n
            data = line.rstrip(b"\r\n")

            parsed = parse_packet(data)
            if parsed is None:
                continue

            # Customize print format as you like
            print(
                f"node={parsed['node_hex']} "
                f"neighbor={parsed['neighbor_hex']} "
                f"RSSI={parsed['rssi']} dB"
            )
            # Or with integer IDs:
            # print(
            #     f"node={parsed['node_int']} "
            #     f"neighbor={parsed['neighbor_int']} "
            #     f"RSSI={parsed['rssi']} dB"
            # )

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
