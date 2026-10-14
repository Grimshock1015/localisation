import argparse
import csv
import json
import math
from pathlib import Path
import re
import socket
import struct
import sys
import time
from datetime import datetime, timezone


DEFAULT_ADDRESS = "00:16:53:55:D9:FC"
SENSOR_TYPES = {4: "nxt_colour", 16: "touch", 29: "colour", 30: "ultrasonic", 32: "gyro", 33: "infrared"}
MOTOR_TYPES = {7: "large motor", 8: "medium motor"}
DEVICE_STATES = {125: "initializing", 126: "no device", 127: "device error (check cable/port)"}


def constant(value):
    if -32 <= value <= 31:
        return bytes([value & 0x3F])
    if -128 <= value <= 127:
        return b"\x81" + struct.pack("<b", value)
    if -32768 <= value <= 32767:
        return b"\x82" + struct.pack("<h", value)
    return b"\x83" + struct.pack("<i", value)


def timed_motor_command(port, power, duration):
    if port not in "ABCD" or len(port) != 1:
        raise ValueError("Motor port must be A, B, C, or D")
    if not 1 <= abs(power) <= 20:
        raise ValueError("Diagnostic power must be nonzero and within -20..20")
    if not 50 <= duration <= 1000:
        raise ValueError("Diagnostic duration must be within 50..1000 ms")
    mask = 1 << (ord(port) - ord("A"))
    return (b"\xad" + b"".join(constant(value) for value in
            (0, mask, power, 0, duration, 0, 1)) + b"\xaa" + constant(0) + constant(mask))


class SessionLog:
    def __init__(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("x", newline="", encoding="ascii")
        self.started = time.monotonic()
        self.writer = csv.writer(self.file)
        self.writer.writerow(["utc", "elapsed_s", "event", "details"])
        self.file.flush()

    def record(self, event, **details):
        self.writer.writerow([datetime.now(timezone.utc).isoformat(),
                              f"{time.monotonic() - self.started:.6f}", event,
                              json.dumps(details, sort_keys=True)])
        self.file.flush()

    def close(self):
        self.file.close()


class EV3:
    def __init__(self, connection, log, timeout=5):
        self.connection = connection
        self.log = log
        self.timeout = timeout
        self.counter = 0

    def receive_exact(self, size, deadline):
        result = bytearray()
        while len(result) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("EV3 reply deadline exceeded")
            self.connection.settimeout(remaining)
            chunk = self.connection.recv(size - len(result))
            if not chunk:
                raise ConnectionError("EV3 disconnected before completing its reply")
            result.extend(chunk)
        return bytes(result)

    def command(self, operations, global_size=0, reply=True):
        if not 0 <= global_size <= 1023:
            raise ValueError("Invalid EV3 global memory size")
        self.counter = (self.counter + 1) & 0xFFFF
        body = struct.pack("<HBH", self.counter, 0 if reply else 0x80, global_size) + operations
        packet = struct.pack("<H", len(body)) + body
        deadline = time.monotonic() + self.timeout
        self.connection.settimeout(self.timeout)
        self.log.record("command", message_id=self.counter, tx=packet.hex(" "))
        self.connection.sendall(packet)
        if not reply:
            return b""
        length = struct.unpack("<H", self.receive_exact(2, deadline))[0]
        if not 3 <= length <= 1027:
            raise ValueError(f"Invalid EV3 reply length: {length}")
        response = self.receive_exact(length, deadline)
        self.log.record("reply", message_id=self.counter, rx=response.hex(" "))
        message_id, status = struct.unpack("<HB", response[:3])
        if message_id != self.counter:
            raise ValueError(f"Unexpected EV3 message ID: {message_id}")
        if status != 2:
            raise RuntimeError(f"EV3 rejected command (reply type 0x{status:02x})")
        if len(response) != 3 + global_size:
            raise ValueError("EV3 reply has an unexpected payload size")
        return response[3:]

    def type_mode(self, port):
        operations = b"\x99\x05\x00" + constant(port) + b"\x60\x61"
        return tuple(self.command(operations, 2))

    def read_sensor(self, port, sensor_type, current_mode, colour_mode):
        if sensor_type == 29:
            mode = {"colour": 2, "rgb": 4, "reflected": 0}[colour_mode]
            count = 3 if mode == 4 else 1
            raw = mode == 4
            units = {2: "colour_index", 4: "raw_rgb", 0: "percent"}[mode]
        elif sensor_type == 16:
            mode, count, raw, units = 0, 1, False, "pressed"
        elif sensor_type == 30:
            mode, count, raw, units = 0, 1, False, "cm"
        elif sensor_type == 32:
            mode, count, raw, units = 3, 2, False, "degrees,degrees_per_second"
        elif sensor_type == 33:
            mode, count, raw, units = 0, 1, False, "proximity_percent"
        elif sensor_type == 4:
            mode, count, raw, units = 5, 4, True, "ambient_corrected_rgb"
        else:
            raise ValueError(f"Unsupported sensor type {sensor_type} in mode {current_mode}")
        operations = (b"\x99" + constant(28 if raw else 29) + constant(0)
                      + constant(port) + constant(sensor_type) + constant(mode)
                      + constant(count) + bytes(0x60 + 4 * index for index in range(count)))
        payload = self.command(operations, 4 * count)
        if sensor_type == 4:
            red, green, blue, ambient = struct.unpack("<hhhh", payload[:8])
            values = (red - ambient, green - ambient, blue - ambient)
        else:
            values = struct.unpack("<" + ("i" if raw else "f") * count, payload)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Sensor returned an invalid/nonfinite reading")
        return mode, values, units

    def encoder(self, port):
        index = ord(port) - ord("A")
        return struct.unpack("<i", self.command(b"\xb3\x00" + constant(index) + b"\x60", 4))[0]

    def brake(self, port):
        mask = 1 << (ord(port) - ord("A"))
        self.command(b"\xa3\x00" + constant(mask) + b"\x01", reply=False)
        self.log.record("brake_submitted", port=port)


def discover(ev3, log):
    sensors = {}
    for port in range(4):
        sensor_type, mode = ev3.type_mode(port)
        name = SENSOR_TYPES.get(sensor_type, DEVICE_STATES.get(sensor_type, f"unknown type {sensor_type}"))
        print(f"Input {port + 1}: {name}, mode {mode}", flush=True)
        log.record("sensor_discovered", port=port + 1, sensor_type=sensor_type, mode=mode, name=name)
        if sensor_type in SENSOR_TYPES:
            sensors[port] = (sensor_type, mode)
    motors = {}
    for index, port in enumerate("ABCD"):
        motor_type, mode = ev3.type_mode(16 + index)
        name = MOTOR_TYPES.get(motor_type, DEVICE_STATES.get(motor_type, f"unknown type {motor_type}"))
        print(f"Output {port}: {name}", flush=True)
        log.record("motor_discovered", port=port, motor_type=motor_type, mode=mode, name=name)
        if motor_type in MOTOR_TYPES:
            motors[port] = motor_type
    return sensors, motors


def sample_sensors(ev3, log, sensors, args):
    selected = sensors if args.sensor_port is None else {
        port: info for port, info in sensors.items() if port == args.sensor_port - 1}
    if not selected:
        raise RuntimeError("No supported sensor found on the selected input port(s)")
    for sample in range(args.samples):
        for port, (sensor_type, current_mode) in selected.items():
            mode, values, units = ev3.read_sensor(port, sensor_type, current_mode, args.colour_mode)
            log.record("sensor_reading", sample=sample, port=port + 1, sensor_type=sensor_type,
                       mode=mode, values=values, units=units)
            print(f"Sample {sample + 1}, input {port + 1}: {values} [{units}]", flush=True)
        if sample + 1 < args.samples:
            time.sleep(args.interval)


def test_motor(ev3, log, motors, args):
    port = args.motor_port
    if port not in motors:
        raise RuntimeError(f"No supported motor detected on output {port}")
    operations = timed_motor_command(port, args.power, args.duration_ms)
    before = ev3.encoder(port)
    try:
        log.record("motor_pulse", port=port, power=args.power, duration_ms=args.duration_ms,
                   encoder_before=before)
        ev3.command(operations)
    finally:
        ev3.brake(port)
    after = ev3.encoder(port)
    delta = after - before
    log.record("motor_result", port=port, encoder_before=before, encoder_after=after, delta_degrees=delta)
    print(f"Motor {port}: encoder {before} -> {after}, delta {delta} degrees", flush=True)
    if delta == 0:
        raise RuntimeError("No encoder motion detected; check wiring/load before increasing power")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="EV3 port discovery, CSV sensor logs, and bounded motor tests")
    parser.add_argument("action", choices=("discover", "sensors", "motor", "stop"))
    parser.add_argument("--address", default=DEFAULT_ADDRESS)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--timeout", type=float, default=5)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--sensor-port", type=int, choices=range(1, 5))
    parser.add_argument("--colour-mode", choices=("colour", "rgb", "reflected"), default="colour")
    parser.add_argument("--motor-port", choices=list("ABCD"))
    parser.add_argument("--power", type=int, default=10)
    parser.add_argument("--duration-ms", type=int, default=200)
    parser.add_argument("--wheels-lifted", action="store_true")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", args.address):
        parser.error("--address must be a Bluetooth MAC address")
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 30:
        parser.error("--timeout must be within 0..30 seconds (excluding zero)")
    if not 1 <= args.samples <= 10000 or not math.isfinite(args.interval) or not 0 <= args.interval <= 60:
        parser.error("--samples must be 1..10000 and --interval must be 0..60 seconds")
    if args.action in ("motor", "stop") and args.motor_port is None:
        parser.error("--motor-port is required for motor and stop actions")
    if args.action == "motor":
        if not args.wheels_lifted:
            parser.error("Lift the wheels/clear the mechanism, then explicitly pass --wheels-lifted")
        try:
            timed_motor_command(args.motor_port, args.power, args.duration_ms)
        except ValueError as error:
            parser.error(str(error))
    return args


def main(argv=None):
    args = parse_args(argv)
    path = args.log or Path("logs") / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + ".csv")
    try:
        log = SessionLog(path)
    except OSError as error:
        print(f"Cannot create log: {error}", file=sys.stderr)
        return 1
    connection = None
    result = 0
    print(f"Log: {path}", flush=True)
    try:
        log.record("session_start", **{key: str(value) if isinstance(value, Path) else value
                                     for key, value in vars(args).items()})
        connection = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        connection.settimeout(args.timeout)
        connection.connect((args.address, 1))
        log.record("connected", address=args.address, channel=1)
        ev3 = EV3(connection, log, args.timeout)
        if args.action == "stop":
            ev3.brake(args.motor_port)
            print(f"Brake command submitted for {args.motor_port}; verify physical standstill.")
        else:
            sensors, motors = discover(ev3, log)
            if args.action == "sensors":
                sample_sensors(ev3, log, sensors, args)
            elif args.action == "motor":
                test_motor(ev3, log, motors, args)
    except (OSError, ValueError, RuntimeError) as error:
        log.record("error", message=str(error), error_type=type(error).__name__)
        print(f"FAILED: {error}", file=sys.stderr)
        result = 1
    except KeyboardInterrupt:
        log.record("interrupted")
        print("Interrupted", file=sys.stderr)
        result = 130
    finally:
        if connection is not None:
            connection.close()
        log.record("session_end", exit_code=result)
        log.close()
    return result


if __name__ == "__main__":
    sys.exit(main())
