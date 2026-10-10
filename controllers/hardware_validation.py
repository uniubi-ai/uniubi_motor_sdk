"""SDK 0.5.0真实C接口只读检查，可选查询SN与标定状态；不会启动标定。"""
import argparse
import ctypes as c
import datetime
import json
from pathlib import Path
import subprocess
import tempfile
import time

import serial

SDK = Path(__file__).resolve().parents[1]
U8 = c.c_uint8


class Frame(c.Structure):
    _fields_ = [("can_id", c.c_uint32), ("len", U8), ("data", U8 * 64)]


class Status(c.Structure):
    _fields_ = [("position_rad", c.c_float), ("velocity_rad_s", c.c_float),
                ("torque_nm", c.c_float), ("predicted_temperature_c", c.c_int16),
                ("status_bits", U8), ("bus_voltage_v", U8)]


class Full(c.Structure):
    _fields_ = [(n, c.c_float) for n in (
        "iq_target_a", "iq_measured_a", "id_target_a", "id_measured_a",
        "position_target_rad", "phase_b_a", "phase_c_a", "output_velocity_rad_s",
        "mechanical_angle_rad", "output_position_rad", "ntc_temperature_c")]
    _fields_ += [("mode", U8), ("compensation_bits", U8)]


class Version(c.Structure):
    _fields_ = [(n, U8) for n in ("major", "minor", "patch", "encoder_type")]


class Identity(c.Structure):
    _fields_ = [(n, U8) for n in ("product_type", "major", "minor", "patch")]
    _fields_ += [("production_year", c.c_uint16), ("production_month", U8),
                ("production_day", U8), ("serial_number", U8 * 17),
                ("serial_length", U8), ("mcu_type", U8)]


class Calibration(c.Structure):
    _fields_ = [("flags", c.c_uint16)]


class Value(c.Union):
    _fields_ = [("status", Status), ("full_status", Full), ("version", Version),
                ("identity", Identity), ("calibration", Calibration)]


class Feedback(c.Structure):
    _fields_ = [("joint_id", U8), ("command", U8), ("kind", c.c_int), ("value", Value)]


class Event(c.Structure):
    _fields_ = [("kind", c.c_int), ("command_class", U8), ("sequence", U8),
                ("error_code", U8), ("channel", U8), ("frame", Frame),
                ("reply_length", U8), ("reply_data", U8 * 128)]


class Parser(c.Structure):
    _fields_ = [("used", c.c_size_t), ("expected", c.c_size_t), ("data", U8 * 128)]


WRITE = c.CFUNCTYPE(c.c_int, c.c_void_p, c.POINTER(U8), c.c_size_t)
SEND = c.CFUNCTYPE(c.c_int, c.c_void_p, c.POINTER(Frame))


class Port(c.Structure):
    _fields_ = [("write", WRITE), ("user", c.c_void_p), ("channel", U8), ("next_sequence", U8)]


class Motor(c.Structure):
    _fields_ = [("joint_id", U8), ("send", SEND), ("user", c.c_void_p)]


class PD(c.Structure):
    _fields_ = [(n, c.c_float) for n in ("position", "velocity", "kp", "kd", "torque")]


def plain(obj):
    result = {name: (bytes(getattr(obj, name)).hex() if isinstance(getattr(obj, name), c.Array)
                   else getattr(obj, name)) for name, _ in obj._fields_}
    if isinstance(obj, Identity):
        result["serial_text"] = bytes(obj.serial_number[:obj.serial_length]).decode("ascii", errors="replace")
    return result


class Session:
    def __init__(self, device, report):
        self.report = report
        self.report.setdefault("traffic", [])
        self.tmp = tempfile.TemporaryDirectory(prefix="u6913-live-")
        library = Path(self.tmp.name) / "sdk.so"
        # 编译实际交付C代码，不用Python重写协议来替代SDK测试。
        subprocess.run(["cc", "-std=c99", "-shared", "-fPIC", "-Wall", "-Wextra", "-Werror",
                        "-I", str(SDK / "include"), str(SDK / "controllers/uniubi6913.c"),
                        str(SDK / "controllers/uniubi6913_board.c"), "-o", str(library)], check=True)
        self.lib = c.CDLL(str(library))
        signatures = {
            "board_init": [c.POINTER(Port), U8, WRITE, c.c_void_p],
            "init": [c.POINTER(Motor), U8, SEND, c.c_void_p],
            "board_feed": [c.POINTER(Parser), U8, c.POINTER(Event)],
            "board_parser_reset": [c.POINTER(Parser)],
            "parse_feedback": [U8, c.c_uint32, c.POINTER(U8), c.c_size_t, c.POINTER(Feedback)],
            "set_mode": [c.POINTER(Motor), c.c_int],
            "speed_control": [c.POINTER(Motor), c.c_float, c.c_float],
            "position_control": [c.POINTER(Motor), c.c_float, c.c_float],
            "torque_control": [c.POINTER(Motor), c.c_float],
            "set_can_id": [c.POINTER(Motor), U8],
            "build_pd": [c.POINTER(Frame), c.POINTER(PD)],
            "send_frame": [c.POINTER(Motor), c.POINTER(Frame)],
        }
        for name in ("enable", "disable", "set_zero",
                     "request_serial_number", "request_calibration_status",
                     "request_status", "request_standard_status", "request_version"):
            signatures[name] = [c.POINTER(Motor)]
        for name, args in signatures.items():
            fn = getattr(self.lib, "u6913_" + name)
            fn.argtypes = args
            fn.restype = None if name == "board_parser_reset" else c.c_int
        self.ser = serial.Serial(device, 4000000, timeout=0.01, write_timeout=0.2, exclusive=True)
        self.parser = Parser()
        self.callback = WRITE(self.write)
        self.send = SEND(("u6913_board_send", self.lib))
        self.port = Port()
        assert self.lib.u6913_board_init(c.byref(self.port), 1, self.callback, None) == 0
        self.motor = Motor()
        self.select(1)

    def select(self, node):
        assert self.lib.u6913_init(c.byref(self.motor), node, self.send, c.byref(self.port)) == 0

    def write(self, user, data, size):
        raw = c.string_at(data, size)
        self.report["traffic"].append({"t": time.time(), "direction": "TX", "hex": raw.hex()})
        try:
            return 0 if self.ser.write(raw) == size else -1
        except Exception as exc:
            self.report["traffic"].append({"error": str(exc)})
            return -1

    def receive(self, duration):
        end = time.monotonic() + duration
        results = []
        while time.monotonic() < end:
            data = self.ser.read(max(1, min(4096, self.ser.in_waiting)))
            if not data:
                continue
            self.report["traffic"].append({"t": time.time(), "direction": "RX", "hex": data.hex()})
            for byte in data:
                event = Event()
                rc = self.lib.u6913_board_feed(c.byref(self.parser), byte, c.byref(event))
                if rc < 0:
                    raise RuntimeError(f"SDK board parser error {rc}")
                if rc != 1:
                    continue
                if event.kind == 1:
                    if event.error_code:
                        raise RuntimeError(f"Board error {event.error_code}, sequence {event.sequence}")
                    continue
                feedback = Feedback()
                f = event.frame
                rc = self.lib.u6913_parse_feedback(f.can_id, f.can_id, f.data, f.len, c.byref(feedback))
                if rc == -5:
                    # 复位后的C0启动通知不属于SDK当前反馈类型；保留证据并继续等待目标回复。
                    self.report.setdefault("unhandled_frames", []).append({
                        "id": f.can_id, "payload": bytes(f.data[:f.len]).hex(), "t": time.time()})
                    continue
                if rc:
                    raise RuntimeError(f"SDK feedback error {rc}: {bytes(f.data[:f.len]).hex()}")
                value = getattr(feedback.value, ("status", "full_status", "version", "identity", "calibration")[feedback.kind])
                result = {"id": feedback.joint_id, "command": feedback.command,
                                "kind": feedback.kind, "data": plain(value),
                                "payload": bytes(f.data[:f.len]).hex(), "received_at": time.time()}
                results.append(result)
        return results


    def command(self, name, expected, *args, timeout=None, node=None):
        # 切换事务前收集旧回复；事务匹配仍检查设备和命令。
        self.receive(0.02)
        rc = getattr(self.lib, "u6913_" + name)(c.byref(self.motor), *args)
        if rc:
            raise RuntimeError(f"{name}: SDK returned {rc}")
        replies = self.receive(timeout if timeout is not None else 0.12)
        found = [r for r in replies if r["id"] == (node or self.motor.joint_id) and r["command"] == expected]
        if not found:
            raise TimeoutError(f"{name}: no matching node {node or self.motor.joint_id} command {expected:02x}; {replies}")
        return found[-1]


    def pd(self, target):
        targets = (PD * 6)()
        targets[self.motor.joint_id - 1] = PD(*target)
        frame = Frame()
        assert self.lib.u6913_build_pd(c.byref(frame), targets) == 0
        self.receive(0.02)
        assert self.lib.u6913_send_frame(c.byref(self.motor), c.byref(frame)) == 0
        replies = self.receive(0.12)
        matches = [r for r in replies if r["id"] == self.motor.joint_id and r["command"] == 0xAE]
        if not matches:
            raise TimeoutError(f"PD: no AE feedback, received {replies}")
        return matches[-1]

    def close(self):
        self.ser.close()
        self.tmp.cleanup()


def discover(session):
    nodes = []
    for node in range(1, 7):
        session.select(node)
        try:
            result = session.command("request_version", 0xE9)
            nodes.append(result)
            print("DISCOVER", result, flush=True)
        except TimeoutError:
            pass
    return nodes


def read_only(session, report, extended=False):
    nodes = discover(session)
    report["discovery"] = nodes
    if not nodes:
        raise RuntimeError("No motor replied to version queries on IDs 1..6")
    for node in nodes:
        session.select(node["id"])
        data = {"standard": session.command("request_standard_status", 0xCB),
                "full": session.command("request_status", 0xFE)}
        if extended:
            data["identity"] = session.command("request_serial_number", 0xE8)
            data["calibration"] = session.command("request_calibration_status", 0xE1)
        report.setdefault("baseline", {})[str(node["id"])] = data
        print("BASELINE", json.dumps(data, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--phase", choices=("read-only",), default="read-only")
    parser.add_argument("--extended", action="store_true", help="额外查询SN、E1标定完成标志，均为只读")
    args = parser.parse_args()
    report = {"started": datetime.datetime.now().astimezone().isoformat(), "port": args.port,
              "baudrate": 4000000, "phase": args.phase, "traffic": []}
    session = None
    try:
        session = Session(args.port, report)
        read_only(session, report, extended=args.extended)
        report["result"] = "FAIL" if any(x["result"] == "FAIL" for x in report.get("cases", [])) else "PASS"
    except Exception as exc:
        report["result"] = "FAIL"
        report["error"] = repr(exc)
        raise
    finally:
        if session:
            session.close()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return 1 if report["result"] == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
