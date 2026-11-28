import struct
import unittest
from unittest.mock import patch

import unoq_io


def request(address=56, length=12):
    body = bytes((0xfe, 20, unoq_io.UNOQ_FEEDBACK_READ, address, length)) + unoq_io.UNOQ_FEEDBACK_IDS
    return b"\xff\xff" + body + bytes((~sum(body) & 0xff,))


def frames(length=12):
    result = bytearray()
    for identifier in unoq_io.UNOQ_FEEDBACK_IDS:
        body = bytes((identifier, length + 2, 0)) + bytes(length)
        result.extend(b"\xff\xff" + body + bytes((~sum(body) & 0xff,)))
    return bytes(result)


class FakeRouter:
    def __init__(self, version=2):
        self.version = version
        self.calls = []
        self.feedback = struct.pack("<II", 1, 8000) + frames()

    def call(self, method, *args, timed=False):
        self.calls.append((method, args))
        if method == "feedback_version":
            if self.version is None:
                raise OSError("method wonderduck/feedback_version not available")
            return self.version
        if method in ("feedback", "feedback_extended"):
            return (self.feedback, (1200, 2500)) if timed else self.feedback
        if method == "servo":
            return frames(args[0][6])
        if method == "feedback_stop":
            return 0
        raise AssertionError(method)


class FeedbackSessionTests(unittest.TestCase):
    def test_fresh_feedback_and_maintenance_stop(self):
        router = FakeRouter()
        session = unoq_io.FeedbackSession(router)
        result = session.read(request())
        self.assertEqual(result[2], unoq_io.UNOQ_FEEDBACK_METADATA_ID)
        sequence, age = struct.unpack_from("<II", result, 5)
        self.assertEqual(sequence, 1)
        self.assertEqual(age, 8000)
        self.assertEqual(result[16], unoq_io.UNOQ_FEEDBACK_TIMING_ID)
        self.assertEqual(struct.unpack_from("<QQ", result, 19), (1200, 2500))
        self.assertEqual(sum(result[16:36]) & 0xff, 0xff)
        self.assertEqual(result[36:], frames())
        self.assertEqual(session.read(request()), unoq_io.feedback_error_reply())
        router.feedback = struct.pack("<II", 2, 40001) + frames()
        self.assertEqual(session.read(request()), unoq_io.feedback_error_reply())
        router.feedback = struct.pack("<II", 2, 8000) + frames()[:-1] + b"\x00"
        self.assertEqual(session.read(request()), unoq_io.feedback_error_reply())
        self.assertEqual([args for method, args in router.calls if method == "feedback"],
                         [(0,), (1,), (1,), (1,)])
        session.stop()
        session.stop()
        self.assertEqual([method for method, _ in router.calls].count("feedback_stop"), 1)

    def test_old_firmware_uses_original_sync_read(self):
        for version in (None, 1):
            with self.subTest(version=version):
                router = FakeRouter(version)
                session = unoq_io.FeedbackSession(router)
                first = session.read(request())
                second = session.read(request())
                self.assertEqual(struct.unpack_from("<II", first, 5), (1, 0))
                self.assertEqual(struct.unpack_from("<II", second, 5), (2, 0))
                frame = next(args[0] for method, args in router.calls if method == "servo")
                self.assertEqual(frame[4], unoq_io.HX_SYNC_READ)
                self.assertEqual(sum(frame[2:]) & 0xff, 0xff)
                session.stop()
                self.assertEqual([method for method, _ in router.calls],
                                 ["feedback_version", "servo", "servo"])

    def test_invalid_request_is_rejected(self):
        session = unoq_io.FeedbackSession(FakeRouter())
        with self.assertRaises(OSError):
            session.read(request()[:-1])

    def test_extended_feedback_keeps_full_recording_frames(self):
        router = FakeRouter(3)
        router.feedback = struct.pack("<II", 19, 8000) + frames(28)
        session = unoq_io.FeedbackSession(router)
        result = session.read(request(40, 28))
        self.assertEqual(result[36:], frames(28))
        self.assertEqual(struct.unpack_from("<I", result, 5)[0], 19)
        self.assertIn(("feedback_extended", (0,)), router.calls)
        router.feedback = struct.pack("<II", 20, 8000) + frames()
        self.assertEqual(session.read(request(40, 28)), unoq_io.feedback_error_reply())
        router.feedback = struct.pack("<II", 21, 8000) + frames(28)
        self.assertEqual(struct.unpack_from("<I", session.read(request(40, 28)), 5)[0], 21)

    def test_version_two_uses_sync_for_extended_and_cache_for_compact(self):
        router = FakeRouter()
        session = unoq_io.FeedbackSession(router)
        self.assertEqual(session.read(request(40, 28))[14:], frames(28))
        compact = session.read(request())
        self.assertEqual(compact[36:], frames())
        self.assertEqual(struct.unpack_from("<I", compact, 5)[0], 2)
        self.assertEqual([method for method, _ in router.calls], ["feedback_version", "servo", "feedback"])

    def test_rpc_time_bounds_age_without_being_forwarded_twice(self):
        router = FakeRouter(3)
        session = unoq_io.FeedbackSession(router)
        router.feedback = struct.pack("<II", 1, 12_000) + frames(28)
        with patch.object(unoq_io.time, "monotonic_ns", side_effect=[0, 20_000_000]):
            result = session.read(request(40, 28))
        self.assertEqual(struct.unpack_from("<II", result, 5), (1, 12_000))
        router.feedback = struct.pack("<II", 2, 12_000) + frames(28)
        with patch.object(unoq_io.time, "monotonic_ns", side_effect=[0, 29_000_000]):
            self.assertEqual(session.read(request(40, 28)), unoq_io.feedback_error_reply())


if __name__ == "__main__":
    unittest.main()
