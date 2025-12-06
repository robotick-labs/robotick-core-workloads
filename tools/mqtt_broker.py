# Copyright Robotick contributors
# SPDX-License-Identifier: Apache-2.0

import asyncio
import sys

TEST_TOPIC = "robotick/integration/topic"
BROKER_MESSAGE = "welcome from broker"


def encode_string(text: str) -> bytes:
    data = text.encode("utf-8")
    length = len(data)
    return bytes([length >> 8, length & 0xFF]) + data


def encode_remaining_length(value: int) -> bytes:
    encoded = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value > 0:
            byte |= 0x80
        encoded.append(byte)
        if value == 0:
            break
    return bytes(encoded)


async def read_packet(reader: asyncio.StreamReader):
    first = await reader.readexactly(1)
    multiplier = 1
    remaining = 0
    while True:
        byte = await reader.readexactly(1)
        byte_value = byte[0]
        remaining += (byte_value & 0x7F) * multiplier
        if (byte_value & 0x80) == 0:
            break
        multiplier *= 128
    payload = await reader.readexactly(remaining)
    return first[0], payload


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    try:
        header, payload = await read_packet(reader)
        if (header & 0xF0) != 0x10:
            raise RuntimeError("Expected CONNECT")

        writer.write(bytes([0x20, 0x02, 0x00, 0x00]))
        await writer.drain()

        header, payload = await read_packet(reader)
        if (header & 0xF0) != 0x80:
            raise RuntimeError("Expected SUBSCRIBE")

        packet_id = (payload[0] << 8) | payload[1]
        writer.write(bytes([0x90, 0x03, packet_id >> 8, packet_id & 0xFF, 0x00]))
        await writer.drain()

        topic_bytes = encode_string(TEST_TOPIC)
        payload_bytes = BROKER_MESSAGE.encode("utf-8")
        remaining = len(topic_bytes) + len(payload_bytes)
        writer.write(
            bytes([0x30])
            + encode_remaining_length(remaining)
            + topic_bytes
            + payload_bytes
        )
        await writer.drain()

        await read_packet(reader)  # client publish
    except (asyncio.IncompleteReadError, RuntimeError):
        pass
    finally:
        writer.close()
        await writer.wait_closed()


async def run_server(port: int):
    server = await asyncio.start_server(handle_client, "127.0.0.1", port)
    async with server:
        await server.serve_forever()


def main():
    if len(sys.argv) != 2:
        print("Usage: python mqtt_broker.py <port>", file=sys.stderr)
        sys.exit(1)

    port = int(sys.argv[1])
    asyncio.run(run_server(port))


if __name__ == "__main__":
    main()
