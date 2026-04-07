# make_engine.py 
# 인터넷 연결 필요

import argparse
import os
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--model", type=str, default="yolov8n.pt",
                        help="model name or path (ex: yolov8n.pt)")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="input image size")
    parser.add_argument("--half", action="store_true",
                        help="use FP16")
    parser.add_argument("--int8", action="store_true",
                        help="use INT8")
    parser.add_argument("--workspace", type=float, default=2.0,
                        help="GPU workspace size (GB)")
    parser.add_argument("--device", type=int, default=0,
                        help="GPU device id")

    args = parser.parse_args()

    print("=== YOLO → TensorRT Export ===")

    # 모델 체크 & 자동 다운로드
    if not os.path.exists(args.model):
        print(f"[INFO] Model not found locally → downloading: {args.model}")
    else:
        print(f"[INFO] Using local model: {args.model}")

    model = YOLO(args.model)

    print("Model loaded successfully")

    print(f"imgsz     : {args.imgsz}")
    print(f"half      : {args.half}")
    print(f"int8      : {args.int8}")
    print(f"workspace : {args.workspace} GB")
    print("==============================")

    # --------------------------------------------------
    # Export
    # --------------------------------------------------
    model.export(
        format="engine",
        imgsz=args.imgsz,
        half=args.half,
        int8=args.int8,
        workspace=args.workspace,
        device=args.device
    )

    print("\n✅ Export finished!")
    print("→ TensorRT .engine file 생성됨")


if __name__ == "__main__":
    main()