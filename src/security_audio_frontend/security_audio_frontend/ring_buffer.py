from collections import deque
import numpy as np


class AudioRingBuffer:
    def __init__(self, max_samples: int):
        self.max_samples = max_samples
        self.buffer = deque(maxlen=max_samples)

    def extend(self, samples: np.ndarray):
        for s in samples:
            self.buffer.append(float(s))

    def get_all(self) -> np.ndarray:
        if len(self.buffer) == 0:
            return np.array([], dtype=np.float32)
        return np.array(self.buffer, dtype=np.float32)

    def get_last_n(self, n: int) -> np.ndarray:
        if n <= 0:
            return np.array([], dtype=np.float32)
        data = list(self.buffer)
        return np.array(data[-n:], dtype=np.float32)