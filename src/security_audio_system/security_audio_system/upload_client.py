import requests


def send_audio(
    server_url: str,
    audio_bytes: bytes,
    filename: str,
    event_label: str,
    doa: float,
    record_start_time: str,
    x: float,
    y: float,
    yaw: float,
):
    url = f"{server_url.rstrip('/')}/upload_audio"

    files = {
        "file": (filename, audio_bytes, "audio/wav")
    }

    data = {
        "model_label": event_label,
        "doa": str(doa),
        "timestamp": record_start_time,
        "x": str(x),
        "y": str(y),
        "yaw": str(yaw),
    }

    res = requests.post(url, files=files, data=data, timeout=5)
    res.raise_for_status()
    return res.json()