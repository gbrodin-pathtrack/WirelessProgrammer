import serial

# Serial configuration
SERIAL_PORT = "COM6"
BAUD_RATE = 115200
TIMEOUT_S = 2

# Protocol definitions
PROTOCOL_HEADER = 0xAA

# Message types
SERIAL_PAIR_REQUEST = 0x01
SERIAL_PAIR_ACK = 0x02
ESB_PAIR_REQUEST = 0x03
ESB_DATA = 0x04
SERIAL_DATA = 0x05
SERIAL_RECEIVED = 0x06
ESB_RECEIVED = 0x07
KEEP_ALIVE = 0x08

# PC protocol states
WAIT_FOR_PAIR_REQUEST = 0
WAIT_FOR_DATA = 1

def receive_packet(ser):
    """
    Receive one packet with the format:

        Byte 0: Header
        Byte 1: Message type
        Byte 2: Payload length
        Byte 3+: Payload
    """

    # Wait until the protocol header is found.
    while True:
        byte = ser.read(1)

        if not byte:
            return None

        if byte[0] == PROTOCOL_HEADER:
            break

    # Read message type and payload length.
    message_type_raw = ser.read(1)
    length_raw = ser.read(1)

    if len(message_type_raw) != 1 or len(length_raw) != 1:
        return None

    message_type = message_type_raw[0]
    payload_length = length_raw[0]

    # Read payload, if one exists.
    payload = ser.read(payload_length)

    if len(payload) != payload_length:
        print("Incomplete payload received")
        return None

    return message_type, payload

def transmit_packet(ser, message_type, payload=b""):
    """
    Transmit one packet using:

        Header | Message type | Length | Payload
    """
    if len(payload) > 255:
        raise ValueError("Payload cannot exceed 255 bytes")

    packet = bytes([
        PROTOCOL_HEADER,
        message_type,
        len(payload),
    ]) + payload

    ser.write(packet)
    ser.flush()

    print("TX:", packet.hex(" ").upper())

def main():
    tagPacketCounter = 0
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud")

    with serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=TIMEOUT_S
    ) as ser:
        print("Listening for wireless programmer...")

        pc_state = WAIT_FOR_PAIR_REQUEST
        while True:
            packet = receive_packet(ser)

            if packet is None:
                pc_state = WAIT_FOR_PAIR_REQUEST
                continue

            message_type, payload = packet

            full_packet = bytes([
                PROTOCOL_HEADER,
                message_type,
                len(payload)
            ]) + payload

            print("RX:", full_packet.hex(" ").upper())
            print(
                f"type=0x{message_type:02X}, "
                f"length={len(payload)}, "
                f"payload={payload.hex(' ').upper() or '<none>'}"
            )

            if pc_state == WAIT_FOR_PAIR_REQUEST:
                if message_type != SERIAL_PAIR_REQUEST:
                    print(
                        f"Unexpected message while waiting for PAIR_REQUEST: "
                        f"0x{message_type:02X}"
                    )
                    continue

                if len(payload) != 0:
                    print("PAIR_REQUEST should not contain a payload")
                    continue

                print("PAIR_REQUEST received")

                transmit_packet(ser, SERIAL_PAIR_ACK)
                print("PAIR_ACK sent")

                pc_state = WAIT_FOR_DATA
                print("Waiting for tag data...")

            elif pc_state == WAIT_FOR_DATA:
                if message_type == KEEP_ALIVE:
                    print("Got keep alive")
                    continue

                if message_type != SERIAL_DATA:
                    print(
                        f"Expected KEEP_ALIVE or SERIAL_DATA, received "
                        f"0x{message_type:02X}"
                    )
                    pc_state = WAIT_FOR_PAIR_REQUEST
                    continue

                print("DATA reeived from tag")
                print("Tag payload: ", payload.hex(" ").upper())

                transmit_packet(ser, SERIAL_RECEIVED)
                print("SERIAL_RECEIVED sent")

                pc_state = WAIT_FOR_DATA
                tagPacketCounter += 1
                print(f"{tagPacketCounter} Waiting for next tag data packet...")

if __name__ == "__main__":
    main()
