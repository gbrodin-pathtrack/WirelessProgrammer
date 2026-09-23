import serial

# Serial configuration
SERIAL_PORT = "COM6"
BAUD_RATE = 115200
TIMEOUT_S = 2

# The bridge sends 16-bit header, type and payload length fields in little-endian order.
BRIDGE_HEADER = b"\x70\x74"  # 0x7470
PC_HEADER = b"\x70\x68"      # 0x6870
HEADER_SIZE = 6
CHECKSUM_SIZE = 2
MAX_PAYLOAD_LEN = 1024

# Bridge -> PC message types
PAIR_REQUEST = 0x0080
FW_INFO = 0x0081
KEEP_ALIVE = 0x0084
TAG_DATA = 0x008F

# PC -> bridge message types
PAIR_REQUEST_ACK = 0x0080
FW_INFO_ACK = 0x0003
KEEP_ALIVE_ACK = 0x0084
TAG_DATA_ACK = 0x008E

# PC protocol states
WAIT_FOR_PAIR_REQUEST = 0
WAIT_FOR_FW_INFO = 1
WAIT_FOR_DATA = 2


def payload_checksum(payload):
    """The bridge's two running 8-bit sums cover only the payload."""
    check_a = 0
    check_b = 0
    for byte in payload:
        check_a = (check_a + byte) & 0xFF
        check_b = (check_b + check_a) & 0xFF
    return bytes((check_a, check_b))


def read_exact(ser, size):
    data = bytearray()
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def receive_packet(ser):
    """Receive and validate a bridge packet, returning (type, payload, raw bytes)."""
    # Search for the two-byte bridge header, including overlapping candidates.
    matched = 0
    while matched < len(BRIDGE_HEADER):
        byte = ser.read(1)
        if not byte:
            return None
        if byte[0] == BRIDGE_HEADER[matched]:
            matched += 1
        else:
            matched = 1 if byte[0] == BRIDGE_HEADER[0] else 0

    fields = read_exact(ser, HEADER_SIZE - len(BRIDGE_HEADER))
    if fields is None:
        print("Incomplete serial header")
        return None

    message_type = int.from_bytes(fields[:2], "little")
    payload_length = int.from_bytes(fields[2:], "little")
    if payload_length > MAX_PAYLOAD_LEN:
        print(f"Invalid payload length: {payload_length}")
        return None

    tail = read_exact(ser, payload_length + CHECKSUM_SIZE)
    if tail is None:
        print("Incomplete serial payload or checksum")
        return None

    payload = tail[:-CHECKSUM_SIZE]
    if tail[-CHECKSUM_SIZE:] != payload_checksum(payload):
        print("Invalid serial payload checksum")
        return None

    return message_type, payload, BRIDGE_HEADER + fields + tail


def transmit_packet(ser, message_type, payload=b""):
    """Send a PC packet: header, type, length, payload, then checksum."""
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError("Payload cannot exceed 1024 bytes")

    packet = (
        PC_HEADER
        + message_type.to_bytes(2, "little")
        + len(payload).to_bytes(2, "little")
        + payload
        + payload_checksum(payload)
    )

    ser.write(packet)
    ser.flush()

    print("TX:", packet.hex(" ").upper())


def main():
    tag_packet_counter = 0
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud")

    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_S) as ser:
        print("Listening for wireless programmer...")
        pc_state = WAIT_FOR_PAIR_REQUEST

        while True:
            packet = receive_packet(ser)
            if packet is None:
                # An idle read is not a disconnect; the bridge retries pairing
                # with an explicit PAIR_REQUEST if its own connection is lost.
                continue

            message_type, payload, raw_packet = packet
            print("RX:", raw_packet.hex(" ").upper())
            print(
                f"type=0x{message_type:04X}, "
                f"length={len(payload)}, "
                f"payload={payload.hex(' ').upper() or '<none>'}"
            )

            # A new request can also arrive after the bridge reconnects.
            if message_type == PAIR_REQUEST:
                if len(payload) != 1:
                    print("PAIR_REQUEST must contain one device-type byte")
                    continue
                print(f"PAIR_REQUEST received, device type={payload[0]}")
                transmit_packet(ser, PAIR_REQUEST_ACK)
                pc_state = WAIT_FOR_FW_INFO
                print("Waiting for firmware information...")
                continue

            if pc_state == WAIT_FOR_FW_INFO:
                if message_type != FW_INFO or len(payload) != 6:
                    print(f"Expected six-byte FW_INFO, received 0x{message_type:04X}")
                    continue
                protocol_version = payload[0]
                git_revision = int.from_bytes(payload[2:6], "little")
                print(
                    f"Firmware info: protocol version={protocol_version}, "
                    f"git revision=0x{git_revision:08X}, "
                    f"padding=0x{payload[1]:02X}"
                )
                transmit_packet(ser, FW_INFO_ACK, b"\x00")  # Status: accepted
                pc_state = WAIT_FOR_DATA
                print("Waiting for tag data or keep-alive...")
                continue

            if pc_state == WAIT_FOR_DATA:
                if message_type == KEEP_ALIVE:
                    if payload:
                        print("KEEP_ALIVE must have no payload")
                        continue
                    transmit_packet(ser, KEEP_ALIVE_ACK)
                    continue

                if message_type == TAG_DATA:
                    print("Data received from tag:", payload.hex(" ").upper())
                    transmit_packet(ser, TAG_DATA_ACK)
                    tag_packet_counter += 1
                    print(f"{tag_packet_counter} tag packets received; waiting for next packet...")
                    continue

            print(f"Unexpected message 0x{message_type:04X} in state {pc_state}")


if __name__ == "__main__":
    main()
