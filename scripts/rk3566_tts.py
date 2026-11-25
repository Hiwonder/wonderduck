"""Run the Piper text encoder on CPU and its waveform decoder on RK3566."""

import ctypes as c
import json
import os
import sys
from types import SimpleNamespace

import numpy as np


class RknnInput(c.Structure):
    _fields_ = [("index", c.c_uint32), ("buf", c.c_void_p), ("size", c.c_uint32),
                ("pass_through", c.c_uint8), ("type", c.c_int), ("fmt", c.c_int)]


class RknnOutput(c.Structure):
    _fields_ = [("want_float", c.c_uint8), ("is_prealloc", c.c_uint8),
                ("index", c.c_uint32), ("buf", c.c_void_p), ("size", c.c_uint32)]


class PiperWaveform:
    channels = 192
    frames = 28
    hop = 256
    context = 10

    def __init__(self, library, model):
        self.api = c.CDLL(str(library))
        ctx = c.c_uint64  # rknn_context on ARM64
        self.api.rknn_init.argtypes = [c.POINTER(ctx), c.c_char_p, c.c_uint32, c.c_uint32, c.c_void_p]
        self.api.rknn_inputs_set.argtypes = [ctx, c.c_uint32, c.POINTER(RknnInput)]
        self.api.rknn_run.argtypes = [ctx, c.c_void_p]
        self.api.rknn_outputs_get.argtypes = [ctx, c.c_uint32, c.POINTER(RknnOutput), c.c_void_p]
        self.api.rknn_outputs_release.argtypes = [ctx, c.c_uint32, c.POINTER(RknnOutput)]
        self.api.rknn_destroy.argtypes = [ctx]
        self.handle = ctx()
        self.check(self.api.rknn_init(c.byref(self.handle), os.fsencode(model), 0, 2, None), "load")

    def __del__(self):
        if getattr(self, "handle", None) and self.handle.value:
            self.api.rknn_destroy(self.handle)

    @staticmethod
    def check(result, operation):
        if result != 0:
            raise RuntimeError(f"RKNN waveform decoder could not {operation} ({result})")

    def decode(self, latent):
        # FP32 and NCHW are both enumerated as zero in the RKNN API.
        data = RknnInput(buf=latent.ctypes.data, size=latent.nbytes)
        output = RknnOutput(want_float=1)
        self.check(self.api.rknn_inputs_set(self.handle, 1, c.byref(data)), "set input")
        self.check(self.api.rknn_run(self.handle, None), "run")
        self.check(self.api.rknn_outputs_get(self.handle, 1, c.byref(output), None), "read audio")
        try:
            samples = self.frames * self.hop
            if not output.buf or output.size != samples * c.sizeof(c.c_float):
                raise RuntimeError(f"Unexpected RKNN waveform size ({output.size} bytes)")
            return np.ctypeslib.as_array(c.cast(output.buf, c.POINTER(c.c_float)), shape=(samples,)).copy()
        finally:
            self.api.rknn_outputs_release(self.handle, 1, c.byref(output))

    def generate(self, latent):
        frames = latent.shape[-1]
        if frames == 0 or latent.shape != (1, self.channels, frames):
            raise RuntimeError(f"Unexpected Piper encoder output shape {latent.shape}")
        step = self.frames - 2 * self.context
        audio = []
        previous_window = -1
        for start in range(0, frames, step):
            # Align the first and last windows with the utterance boundaries.
            window_start = min(max(0, start - self.context), max(0, frames - self.frames))
            if window_start != previous_window:
                window = np.ascontiguousarray(latent[:, :, window_start:window_start + self.frames])
                if window.shape[-1] < self.frames:
                    window = np.pad(window, ((0, 0), (0, 0), (0, self.frames - frames)))
                waveform = self.decode(window)
                previous_window = window_start
            left = (start - window_start) * self.hop
            audio.append(waveform[left:left + min(step, frames - start) * self.hop])
        return np.concatenate(audio)


class RknnPiper:
    def __init__(self, root, config, runtime):
        dependencies = str(runtime / "python-tts")
        if dependencies not in sys.path:
            sys.path.insert(0, dependencies)
        import onnxruntime as ort

        self.voice = json.loads((root / config["phoneme_config"]).read_text(encoding="utf-8"))
        session = ort.SessionOptions()
        session.intra_op_num_threads = config.get("threads", 2)
        session.inter_op_num_threads = 1
        session.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
        self.encoder = ort.InferenceSession(str(root / config["encoder"]), session,
                                            providers=["CPUExecutionProvider"])
        self.decoder = PiperWaveform(runtime / "lib/librknnrt.so", root / config["decoder"])

    def generate(self, text):
        from piper_phonemize import phonemize_espeak

        ids = self.voice["phoneme_id_map"]
        settings = self.voice["inference"]
        scales = np.array([settings["noise_scale"], settings["length_scale"], settings["noise_w"]],
                          dtype=np.float32)
        audio = []
        for sentence in phonemize_espeak(text, self.voice["espeak"]["voice"]):
            tokens = ids["^"] + ids["_"]
            for phoneme in sentence:
                if phoneme in ids:
                    tokens.extend(ids[phoneme])
                    tokens.extend(ids["_"])
            tokens.extend(ids["$"])
            latent, = self.encoder.run(None, {
                "input": np.array([tokens], dtype=np.int64),
                "input_lengths": np.array([len(tokens)], dtype=np.int64),
                "scales": scales,
            })
            audio.append(self.decoder.generate(latent))
        samples = np.concatenate(audio) if audio else np.empty(0, dtype=np.float32)
        return SimpleNamespace(samples=samples, sample_rate=self.voice["audio"]["sample_rate"])
