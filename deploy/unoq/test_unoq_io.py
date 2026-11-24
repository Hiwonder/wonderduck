import struct
import unittest

import unoq_io


def request():
    body = bytes((0xfe, 20, unoq_io.UNOQ_FEEDBACK_READ, 56, 12)) + unoq_io.UNOQ_FEEDBACK_IDS
    return b"\xff\xff" + body + bytes((~sum(body) & 0xff,))


def frames():
    result = bytearray()
    for identifier in unoq_io.UNOQ_FEEDBACK_IDS:
        body = bytes((identifier, 14, 0)) + bytes(12)
        result.extend(b"\xff\xff" + body + bytes((~sum(body) & 0xff,)))
    return bytes(result)


class FakeRouter:
    def __init__(self, version=2):
        self.version = version
        self.calls = []
        self.feedback = struct.pack("<II", 1, 8000) + frames()

    def call(self, method, *args):
        self.calls.append((method, args))
        if method == "feedback_version":
            if self.version is None:
                raise OSError("method wonderduck/feedback_version not available")
            return self.version
        if method == "feedback":
            return self.feedback
        if method == "servo":
            return b"sample"
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
        self.assertGreaterEqual(age, 8000)
        self.assertEqual(result[14:], frames())
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


if __name__ == "__main__":
    unittest.main()
