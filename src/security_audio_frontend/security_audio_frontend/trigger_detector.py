import numpy as np


def compute_dbfs(audio: np.ndarray) -> float:
    if audio.size == 0:
        return -100.0
    rms = np.sqrt(np.mean(np.square(audio)))
    if rms < 1e-12:
        return -100.0
    return float(20.0 * np.log10(rms + 1e-12))


class TriggerDetector:
    def __init__(self, threshold_dbfs: float = -35.0):
        self.threshold_dbfs = threshold_dbfs

    def is_triggered(self, audio_chunk: np.ndarray) -> tuple[bool, float]:
        level_dbfs = compute_dbfs(audio_chunk)
        return level_dbfs >= self.threshold_dbfs, level_dbfs