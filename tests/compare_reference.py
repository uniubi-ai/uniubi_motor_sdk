"""可选开发校验：将 C SDK 与现有 Python 协议的 CAN 载荷逐字节比较。

不连接硬件，不依赖第三方 Python 包；不将参考子仓打包进对外 SDK。
"""
import argparse
import ctypes as ct
import os
from pathlib import Path
import random
import shlex
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
SDK = Path(__file__).resolve().parents[1]


class Frame(ct.Structure):
    _fields_ = [("can_id", ct.c_uint32), ("len", ct.c_uint8), ("data", ct.c_uint8 * 64)]


class PD(ct.Structure):
    _fields_ = [(key, ct.c_float) for key in ("position", "velocity", "kp", "kd", "torque")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol-root", required=True, type=Path)
    args = parser.parse_args()
    sys.path.insert(0, str(args.protocol_root.resolve()))
    from joint_motor import joint_motor_cmd as ref
    from joint_motor import data_mapping as mapping
    from board.protocol import parse_transport_frame

    with tempfile.TemporaryDirectory(prefix="u6913-reference-") as temporary:
        library = Path(temporary) / "libu6913.so"
        subprocess.run(shlex.split(os.environ.get("CC", "cc")) + [
            "-std=c99", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
            "-I", str(SDK / "include"), str(SDK / "controllers/uniubi6913.c"),
            "-o", str(library),
        ], check=True)
        lib = ct.CDLL(str(library))
        signatures = {
            "simple": [ct.c_uint8, ct.c_int],
            "mode": [ct.c_uint8, ct.c_int],
            "speed": [ct.c_uint8, ct.c_float, ct.c_float],
            "position": [ct.c_uint8, ct.c_float, ct.c_float],
            "torque": [ct.c_uint8, ct.c_float],
            "pd": [ct.POINTER(PD)],
            "set_can_id": [ct.c_uint8, ct.c_uint8],
        }
        for name, signature in signatures.items():
            function = getattr(lib, "u6913_build_" + name)
            function.argtypes = [ct.POINTER(Frame)] + signature
            function.restype = ct.c_int

        checks = 0

        def compare(name, arguments, reference_frame=None, expected=None):
            nonlocal checks
            f = Frame()
            result = getattr(lib, "u6913_build_" + name)(ct.byref(f), *arguments)
            if reference_frame is not None:
                # A1 data domain = function + channel/mode/type + BE CAN ID + len + CAN data.
                domain = parse_transport_frame(reference_frame)["data_domain"]
                if domain[0] != 3 or domain[8] != 64 or domain[4:8] != bytes(4):
                    raise AssertionError("Reference transport changed; review protocol before updating test")
                expected = domain[9:]
            if result != 0 or f.can_id != 0 or f.len != 64 or bytes(f.data) != expected:
                raise AssertionError(f"{name} mismatch: rc={result}\nC={bytes(f.data).hex()}\nref={expected.hex()}")
            checks += 1

        simple_commands = [
            (0xFC, ref.build_enable_command), (0xFD, ref.build_disable_command),
            (0xEF, ref.build_set_zero_command),
            (0xAF, ref.build_start_auto_calib_command),
            (0xE1, ref.build_get_calib_status_command),
            (0xFE, ref.build_get_full_status_command), (0xE9, ref.build_get_version_command),
        ]
        for mask in range(1, 64):
            ids = [i + 1 for i in range(6) if mask & (1 << i)]
            for command, builder in simple_commands:
                compare("simple", (mask, command), builder(ids))
            for mode in range(4):
                compare("mode", (mask, mode), ref.build_set_mode_command(mode, ids))

        rng = random.Random(6913)
        for i in range(1000):
            joint = i % 6 + 1
            mask = 1 << (joint - 1)
            # 相同 binary32 输入避免把 Python double 与 C float 输入差异误判为编码错误。
            pos, vel, torque, limit = [ct.c_float(x).value for x in (
                rng.uniform(-12.5, 12.5), rng.uniform(-65, 65),
                rng.uniform(-100, 100), rng.uniform(0, 100))]
            compare("speed", (mask, vel, limit), ref.build_speed_control_command(vel, limit, joint))
            compare("position", (mask, pos, limit), ref.build_position_control_command(pos, limit, joint))
            compare("torque", (mask, torque), ref.build_current_control_command(torque, 0, joint))
            targets = (PD * 6)()
            targets[joint - 1] = PD(pos, vel, rng.uniform(0, 500), rng.uniform(0, 5), torque)
            t = targets[joint - 1]
            compare("pd", (targets,), ref.build_pd_control_command(t.position, t.velocity, t.kp, t.kd, t.torque, joint))

        for joint in range(1, 7):
            compare("set_can_id", (joint, 7 - joint), ref.build_set_can_id_command(7 - joint, joint))

        # 真正六轴不同目标的 PD 帧，按参考映射函数独立构造所有槽位。
        targets = (PD * 6)()
        expected = bytearray(64)
        expected[1] = 0xAE
        for i in range(6):
            targets[i] = PD(i - 3, (i - 3) * 10, i * 50, i, (i - 3) * 10)
            t = targets[i]
            fields = [mapping.map_position_to_raw(t.position), mapping.map_velocity_to_raw(t.velocity),
                      mapping.map_kp_to_raw(t.kp), mapping.map_kd_to_raw(t.kd), mapping.map_ff_torque_to_raw(t.torque)]
            expected[2 + 10*i:12 + 10*i] = b"".join(x.to_bytes(2, "little") for x in fields)
        compare("pd", (targets,), expected=bytes(expected))
        print(f"PASS: {checks} CAN payloads match current Python reference byte-for-byte")


if __name__ == "__main__":
    main()
