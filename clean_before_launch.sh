#!/bin/bash

echo "[CLEAN] stopping old processes..."

pkill -f realsense2_camera || true
pkill -f patrol_http_bridge || true
pkill -f person_detect_control_node || true
pkill -f capture_sender || true
pkill -f robot_gui_node || true

fuser -k 8090/tcp 2>/dev/null || true
fuser -k 8091/tcp 2>/dev/null || true

sleep 2

echo "[CLEAN] done"