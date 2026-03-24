import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

import csv
import numpy as np
import soundfile as sf
import tensorflow as tf
import tensorflow_hub as hub


class YamnetInfer:
    def __init__(self):
        self.model = hub.load('https://tfhub.dev/google/yamnet/1')
        class_map_path = self.model.class_map_path().numpy().decode('utf-8')

        self.class_names = []
        with tf.io.gfile.GFile(class_map_path) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.class_names.append(row['display_name'])

    def load_audio(self, wav_path: str):
        audio_data, sample_rate = sf.read(wav_path)

        if len(audio_data.shape) > 1:
            audio_data = np.mean(audio_data, axis=1)

        max_amplitude = np.max(np.abs(audio_data))
        if max_amplitude > 0:
            audio_data = audio_data / max_amplitude

        audio_data = np.array(audio_data, dtype=np.float32)
        return audio_data, sample_rate

    def infer_file(self, wav_path: str, top_k: int = 5):
        audio_data, sample_rate = self.load_audio(wav_path)

        if sample_rate != 16000:
            raise ValueError(f'YAMNet expected 16000 Hz, got {sample_rate}')

        scores, embeddings, spectrogram = self.model(audio_data)
        scores_np = scores.numpy()

        max_scores = np.max(scores_np, axis=0)
        top_indices = np.argsort(max_scores)[::-1][:top_k]

        top_labels = [self.class_names[i] for i in top_indices]
        top_scores = [float(max_scores[i]) for i in top_indices]

        return {
            'sample_rate': sample_rate,
            'duration_sec': float(len(audio_data) / sample_rate),
            'top_labels': top_labels,
            'top_scores': top_scores,
            'top1_label': top_labels[0],
            'top1_confidence': top_scores[0],
        }