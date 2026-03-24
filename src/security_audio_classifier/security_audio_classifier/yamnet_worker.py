import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import csv
import json
import sys
import time

import numpy as np
import soundfile as sf
import tensorflow as tf
import tensorflow_hub as hub


MODEL_PATH = "/home/chan/models/yamnet"


def load_model():
    model = hub.load(MODEL_PATH)
    class_map_path = model.class_map_path().numpy().decode("utf-8")

    class_names = []
    with tf.io.gfile.GFile(class_map_path) as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            class_names.append(row["display_name"])

    return model, class_names


def load_audio(wav_path: str):
    audio_data, sample_rate = sf.read(wav_path)

    if len(audio_data.shape) > 1:
        audio_data = np.mean(audio_data, axis=1)

    max_amplitude = np.max(np.abs(audio_data))
    if max_amplitude > 0:
        audio_data = audio_data / max_amplitude

    return np.array(audio_data, dtype=np.float32), sample_rate


def collapse_to_task_classes(top_labels, top_scores):
    speech_keywords = [
        "speech", "conversation", "narration", "monologue",
        "male", "female", "child", "kids", "babbling",
        "whisper", "inside, small room"
    ]

    impact_keywords = [
        "bang", "slam", "thump", "thud", "bump", "hit",
        "smash", "crash", "breaking", "glass", "shatter",
        "explosion", "gunshot", "knock", "hammer", "clap",
        "wood", "chop", "percussion", "drum", "wood block"
    ]

    alarm_keywords = [
        "alarm", "siren", "buzzer", "beep", "smoke detector",
        "fire alarm", "burglar alarm", "civil defense siren"
    ]

    scream_keywords = [
        "scream", "screaming", "yell", "shout", "shriek",
        "crying", "wail", "sobbing"
    ]

    def label_to_task(label: str):
        l = label.lower()

        if any(k in l for k in scream_keywords):
            return "scream"
        if any(k in l for k in alarm_keywords):
            return "alarm"
        if any(k in l for k in impact_keywords):
            return "impact"
        if any(k in l for k in speech_keywords):
            return "speech"
        return None

    check_n = min(3, len(top_labels))
    for i in range(check_n):
        task = label_to_task(top_labels[i])
        if task is not None:
            return task, float(top_scores[i]), {
                "matched_rank": i + 1,
                "matched_label": top_labels[i]
            }

    return "ignore", 0.0, {
        "matched_rank": -1,
        "matched_label": ""
    }


def main():
    # GPU가 보이면 메모리 증가만 허용
    gpus = tf.config.list_physical_devices('GPU')
    for gpu in gpus:
        try:
            tf.config.experimental.set_memory_growth(gpu, True)
        except Exception:
            pass

    t0 = time.time()
    model, class_names = load_model()
    t1 = time.time()

    print(json.dumps({
        "ready": True,
        "load_model_sec": t1 - t0,
        "gpus": [d.name for d in gpus],
    }), flush=True)

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue

        try:
            req = json.loads(line)
            wav_path = req["wav_path"]

            t0 = time.time()
            audio_data, sample_rate = load_audio(wav_path)

            if sample_rate != 16000:
                print(json.dumps({
                    "ok": False,
                    "error": f"expected 16000 Hz, got {sample_rate}"
                }), flush=True)
                continue

            scores, embeddings, spectrogram = model(audio_data)
            scores_np = scores.numpy()

            max_scores = np.max(scores_np, axis=0)
            top_k = 5
            top_indices = np.argsort(max_scores)[::-1][:top_k]

            top_labels = [class_names[i] for i in top_indices]
            top_scores = [float(max_scores[i]) for i in top_indices]

            task_label, task_confidence, task_info = collapse_to_task_classes(
                top_labels,
                top_scores
            )

            t1 = time.time()

            print(json.dumps({
                "ok": True,
                "task_label": task_label,
                "task_confidence": task_confidence,
                "task_info": task_info,
                "raw_top1_label": top_labels[0],
                "raw_top1_confidence": top_scores[0],
                "top_labels": top_labels,
                "top_scores": top_scores,
                "timing_sec": t1 - t0
            }), flush=True)

        except Exception as e:
            print(json.dumps({
                "ok": False,
                "error": str(e)
            }), flush=True)


if __name__ == "__main__":
    main()
