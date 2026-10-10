"""只读检查及更新后的ctypes反馈绑定回归，不打开真实串口。"""
import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

SDK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SDK / "controllers"))
from hardware_validation import Session, read_only, Feedback, U8, c, plain


class ReadOnlyValidationTests(unittest.TestCase):
    def test_recorded_feedback_with_current_c_library(self):
        history = json.loads((SDK / "tests/records/hardware-validation-2026-10-10/read-only.json").read_text())
        chunks = [bytes.fromhex(x["hex"]) for x in history["traffic"] if x.get("direction") == "RX"]
        port = Mock()
        port.in_waiting = 4096
        port.read.side_effect = lambda count: chunks.pop(0) if chunks else b""
        with patch("hardware_validation.serial.Serial", return_value=port):
            session = Session("offline", {})
        try:
            replies = session.receive(0.2)
            version = next(r for r in replies if r["command"] == 0xE9)
            self.assertEqual(version["kind"], 2)
            self.assertEqual(version["data"], dict(major=1, minor=2, patch=193, encoder_type=3))
            self.assertTrue(any(r["kind"] == 0 and r["command"] == 0xCB for r in replies))
            self.assertTrue(any(r["kind"] == 1 and r["command"] == 0xFE for r in replies))
            for name in ("read_param", "write_param", "build_read_param", "build_write_param",
                         "reset", "save_params", "restore_params", "request_calibration_status_v2"):
                self.assertFalse(hasattr(session.lib, "u6913_" + name), name)
            port.write.assert_not_called()
            identity = (U8 * 32)(1, 0xE8, 7, 1, 2, 193, 0xEA, 7, 10, 10)
            identity[10:16] = b"SN0123"
            identity[26] = 2
            identity[31] = 0xE8
            feedback = Feedback()
            self.assertEqual(session.lib.u6913_parse_feedback(1, 1, identity, 32, c.byref(feedback)), 0)
            self.assertEqual(feedback.kind, 3)
            self.assertEqual(plain(feedback.value.identity)["serial_text"], "SN0123")
            self.assertEqual(feedback.value.identity.production_year, 2026)
            calibration = (U8 * 48)(1, 0xE1)
            calibration[8:10] = bytes([0x51, 3])
            self.assertEqual(session.lib.u6913_parse_feedback(1, 1, calibration, 48, c.byref(feedback)), 0)
            self.assertEqual(feedback.kind, 4)
            self.assertEqual(plain(feedback.value.calibration), dict(flags=0x0351))
        finally:
            session.close()

    def test_read_only_issues_queries_only(self):
        session = Mock()
        def reply(name, command):
            if session.select.call_args.args[0] != 1:
                raise TimeoutError("No node")
            return {"id": 1, "command": command, "data": {}}
        session.command.side_effect = reply
        report = {}
        with contextlib.redirect_stdout(io.StringIO()):
            read_only(session, report)
        self.assertEqual([c.args[1] for c in session.command.call_args_list], [0xE9] * 6 + [0xCB, 0xFE])
        self.assertEqual(set(report["baseline"]["1"]), {"standard", "full"})
        session.command.reset_mock()
        with contextlib.redirect_stdout(io.StringIO()):
            read_only(session, {}, extended=True)
        self.assertEqual([c.args[1] for c in session.command.call_args_list],
                         [0xE9] * 6 + [0xCB, 0xFE, 0xE8, 0xE1])


if __name__ == "__main__":
    unittest.main()
