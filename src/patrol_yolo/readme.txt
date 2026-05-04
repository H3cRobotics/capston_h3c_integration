**** 참고 https://rainbow-chaser.tistory.com/4


  1 package had stderr output: realsense2_ros_mqtt_bridge
chan@ubuntu:~/capston_h3c_integration$ source install/setup.bash 
chan@ubuntu:~/capston_h3c_integration$ ros2 launch capston_bringup bringup_all.launch.py 
[INFO] [launch]: All log files can be found below /home/chan/.ros/log/2026-04-16-21-44-28-670132-ubuntu-12800
[INFO] [launch]: Default logging verbosity is set to INFO
Warning: Parameter 'server_ip' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'image_topic' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'yolo_mode' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'rgb_camera.enable_auto_white_balance' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'rgb_camera.white_balance' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
[INFO] [launch.user]: 🚀 Launching as Normal ROS Node
[INFO] [realsense2_camera_node-1]: process started with pid [12807]
[INFO] [capture_sender-2]: process started with pid [12809]
[INFO] [patrol_http_bridge-3]: process started with pid [12811]
[INFO] [person_tracker_node-4]: process started with pid [12813]
[INFO] [person_detect_control_node-5]: process started with pid [12815]
[INFO] [person_event_sender_node-6]: process started with pid [12817]
[INFO] [robot_pose_sender-7]: process started with pid [12819]
[INFO] [robot_goal_sender-8]: process started with pid [12823]
[INFO] [can_teacher_node-9]: process started with pid [12825]
[INFO] [patrol_command_bridge-10]: process started with pid [12827]
[INFO] [audio_frontend_node-11]: process started with pid [12829]
[INFO] [respeaker_doa_node-12]: process started with pid [12831]
[INFO] [yamnet_classifier_node-13]: process started with pid [12847]
[INFO] [sound_event_manager_node-14]: process started with pid [12856]
[INFO] [clip_transfer_node-15]: process started with pid [12858]
[INFO] [sound_event_monitor_node-16]: process started with pid [12867]
[realsense2_camera_node-1] [INFO] [1776343469.284563055] [camera.camera]: RealSense ROS v4.57.7
[realsense2_camera_node-1] [INFO] [1776343469.284851450] [camera.camera]: Built with LibRealSense v2.57.7
[realsense2_camera_node-1] [INFO] [1776343469.284890203] [camera.camera]: Running with LibRealSense v2.57.7
[sound_event_manager_node-14] [INFO] [1776343470.198652170] [sound_event_manager_node]: sound_event_manager_node started
[sound_event_monitor_node-16] [INFO] [1776343470.276707849] [sound_event_monitor_node]: sound_event_monitor_node started
[patrol_command_bridge-10] [INFO] [1776343470.559080929] [patrol_command_bridge]: PatrolCommandBridge started | server=http://192.168.0.221:8000 | waypoints_topic=/patrol/waypoints_json | command_topic=/patrol/command | reload_waypoints_topic=/patrol/reload_waypoints
[respeaker_doa_node-12] Traceback (most recent call last):
[respeaker_doa_node-12]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/security_audio_frontend/respeaker_doa_node", line 33, in <module>
[respeaker_doa_node-12]     sys.exit(load_entry_point('security-audio-frontend==0.0.1', 'console_scripts', 'respeaker_doa_node')())
[respeaker_doa_node-12]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/respeaker_doa_node.py", line 91, in main
[respeaker_doa_node-12]     node = RespeakerDoaNode()
[respeaker_doa_node-12]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/respeaker_doa_node.py", line 39, in __init__
[respeaker_doa_node-12]     raise RuntimeError('ReSpeaker control device not found (0x2886:0x0018)')
[respeaker_doa_node-12] RuntimeError: ReSpeaker control device not found (0x2886:0x0018)
[robot_goal_sender-8] [INFO] [1776343470.618896753] [robot_goal_sender]: RobotGoalSender started | goal_topic=/goal_pose_2d | next_place_topic=/next_place_id | endpoint=http://192.168.0.221:8000/robot/goal
[realsense2_camera_node-1] [INFO] [1776343470.683654523] [camera.camera]: Device with serial number 241122307367 was found.
[realsense2_camera_node-1] 
[realsense2_camera_node-1] [INFO] [1776343470.685913648] [camera.camera]: Device with physical ID 2-1.2.4.3-5 was found.
[realsense2_camera_node-1] [INFO] [1776343470.686128632] [camera.camera]: Device with name Intel RealSense D455 was found.
[realsense2_camera_node-1] [INFO] [1776343470.686536135] [camera.camera]: Device with port number 2-1.2.4.3 was found.
[realsense2_camera_node-1] [INFO] [1776343470.686600394] [camera.camera]: Device USB type: 3.2
[realsense2_camera_node-1] [INFO] [1776343470.686790161] [camera.camera]: getParameters...
[realsense2_camera_node-1] [INFO] [1776343470.687226305] [camera.camera]: JSON file is not provided
[realsense2_camera_node-1] [INFO] [1776343470.687570926] [camera.camera]: Device Name: Intel RealSense D455
[realsense2_camera_node-1] [INFO] [1776343470.688291529] [camera.camera]: Device Serial No: 241122307367
[realsense2_camera_node-1] [INFO] [1776343470.688344683] [camera.camera]: Device physical port: 2-1.2.4.3-5
[realsense2_camera_node-1] [INFO] [1776343470.688380589] [camera.camera]: Device FW version: 5.17.0.10
[realsense2_camera_node-1] [INFO] [1776343470.688654487] [camera.camera]: Device Product ID: 0x0B5C
[realsense2_camera_node-1] [INFO] [1776343470.688694105] [camera.camera]: Sync Mode: Off
[clip_transfer_node-15] [INFO] [1776343470.743305476] [clip_transfer_node]: clip_transfer_node started | server_url=http://192.168.0.221:8000 | pose_topic=/robot_pose
[robot_pose_sender-7] [INFO] [1776343470.868529686] [robot_pose_sender]: RobotPoseSender started | topic=/robot_pose | status_topic=/robot_status | endpoint=http://192.168.0.221:8000/robot/pose
[ERROR] [respeaker_doa_node-12]: process has died [pid 12831, exit code 1, cmd '/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/security_audio_frontend/respeaker_doa_node --ros-args -r __node:=respeaker_doa_node --params-file /home/chan/capston_h3c_integration/install/security_audio_frontend/share/security_audio_frontend/config/frontend.yaml'].
[can_teacher_node-9] [INFO] [1776343471.033909392] [can_teacher_node]: CAN bus connected: can0
[can_teacher_node-9] [ERROR] [1776343471.038275764] [can_teacher_node]: CAN loop error: Error receiving: Network is down [Error Code 100]
[can_teacher_node-9] [INFO] [1776343471.051205211] [can_teacher_node]: CanTeacherNode started | pose_topic=/robot_pose | server=http://192.168.0.221:8000 | can_if=can0 | teach_can_id=0x102
[audio_frontend_node-11] [INFO] [1776343471.053785660] [audio_frontend_node]: Using input device index=0, name="NVIDIA Jetson Orin NX HDA: HDMI 0 (hw:0,3)", input_channels=6, channel_index=0, sample_rate=16000
[audio_frontend_node-11] Expression 'parameters->channelCount <= maxChans' failed in 'src/hostapi/alsa/pa_linux_alsa.c', line: 1514
[audio_frontend_node-11] Expression 'ValidateParameters( inputParameters, hostApi, StreamDirection_In )' failed in 'src/hostapi/alsa/pa_linux_alsa.c', line: 2818
[audio_frontend_node-11] Traceback (most recent call last):
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/security_audio_frontend/audio_frontend_node", line 33, in <module>
[audio_frontend_node-11]     sys.exit(load_entry_point('security-audio-frontend==0.0.1', 'console_scripts', 'audio_frontend_node')())
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/audio_frontend_node.py", line 258, in main
[audio_frontend_node-11]     node = AudioFrontendNode()
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/audio_frontend_node.py", line 96, in __init__
[audio_frontend_node-11]     self.start_audio_stream()
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/audio_frontend_node.py", line 129, in start_audio_stream
[audio_frontend_node-11]     self.stream = sd.InputStream(
[audio_frontend_node-11]   File "/home/chan/.local/lib/python3.10/site-packages/sounddevice.py", line 1457, in __init__
[audio_frontend_node-11]     _StreamBase.__init__(self, kind='input', wrap_callback='array',
[audio_frontend_node-11]   File "/home/chan/.local/lib/python3.10/site-packages/sounddevice.py", line 914, in __init__
[audio_frontend_node-11]     _check(_lib.Pa_OpenStream(self._ptr, iparameters, oparameters,
[audio_frontend_node-11]   File "/home/chan/.local/lib/python3.10/site-packages/sounddevice.py", line 2838, in _check
[audio_frontend_node-11]     raise PortAudioError(errormsg, err)
[audio_frontend_node-11] sounddevice.PortAudioError: Error opening InputStream: Invalid number of channels [PaErrorCode -9998]
[patrol_http_bridge-3] [INFO] [1776343471.414672938] [patrol_http_bridge]: HTTP bridge ready at :8090
[patrol_http_bridge-3] INFO:     Started server process [12811]
[patrol_http_bridge-3] INFO:     Waiting for application startup.
[patrol_http_bridge-3] INFO:     Application startup complete.
[patrol_http_bridge-3] INFO:     Uvicorn running on http://0.0.0.0:8090 (Press CTRL+C to quit)
[person_event_sender_node-6] [INFO] [1776343471.456303912] [person_event_sender_node]: [INIT] PersonEventSenderNode | tracks=/person_tracking/tracks_json | annotated=/person_tracking/annotated | robot_pose=/robot_pose
[ERROR] [audio_frontend_node-11]: process has died [pid 12829, exit code 1, cmd '/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/security_audio_frontend/audio_frontend_node --ros-args -r __node:=audio_frontend_node --params-file /home/chan/capston_h3c_integration/install/security_audio_frontend/share/security_audio_frontend/config/frontend.yaml'].
[capture_sender-2] [INFO] [1776343471.576323341] [capture_sender]: server_url     = http://192.168.0.221:8000
[capture_sender-2] [INFO] [1776343471.576989158] [capture_sender]: signaling_url  = http://192.168.0.221:8001
[capture_sender-2] [INFO] [1776343471.577475705] [capture_sender]: n_frames       = 5
[capture_sender-2] [INFO] [1776343471.577956875] [capture_sender]: sample_dt      = 0.2
[capture_sender-2] [INFO] [1776343471.578441341] [capture_sender]: capture_timeout_s = 5.0
[capture_sender-2] [INFO] [1776343471.578898062] [capture_sender]: post_timeout_s    = 10.0
[capture_sender-2] [INFO] [1776343471.599966055] [capture_sender]: capture sender ready
[person_detect_control_node-5] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/node.py:440: UserWarning: when declaring parameter named 'start_enabled', declaring a parameter only providing its name is deprecated. You have to either:
[person_detect_control_node-5]  - Pass a name and a default value different to "PARAMETER NOT SET" (and optionally a descriptor).
[person_detect_control_node-5]  - Pass a name and a parameter type.
[person_detect_control_node-5]  - Pass a name and a descriptor with `dynamic_typing=True
[person_detect_control_node-5]   warnings.warn(
[person_detect_control_node-5] INFO:     Started server process [12815]
[person_detect_control_node-5] INFO:     Waiting for application startup.
[person_detect_control_node-5] INFO:     Application startup complete.
[person_detect_control_node-5] INFO:     Uvicorn running on http://0.0.0.0:8091 (Press CTRL+C to quit)
[person_detect_control_node-5] [INFO] [1776343471.838586171] [person_detect_control_node]: [INIT] PersonDetectControlNode | robot_pose=/robot_pose | enable_topic=/person_tracking/enable | notify_http=0.0.0.0:8091 | startup_publish=disabled_until_first_config
[person_detect_control_node-5] [INFO] [1776343471.839529727] [person_detect_control_node]: [YOLO ENABLE] True reason=startup_mode_global
[person_detect_control_node-5] [INFO] [1776343471.840073075] [person_detect_control_node]: Initial state published: True
[patrol_command_bridge-10] [WARN] [1776343473.578807760] [patrol_command_bridge]: Timeout while fetching patrol waypoints
[person_tracker_node-4] [INFO] [1776343474.713784622] [person_tracker_node]: Loading YOLO model: /home/chan/capston_h3c_integration/src/patrol_yolo/model/yolov8n.engine
[person_tracker_node-4] [INFO] [1776343474.736576323] [person_tracker_node]: [MODE] realsense | color_topic=/camera/camera/color/image_raw | depth_topic=/camera/camera/aligned_depth_to_color/image_raw
[person_tracker_node-4] [INFO] [1776343474.739572307] [person_tracker_node]: [CONTROL] YOLO inference default=ON
[person_tracker_node-4] [INFO] [1776343474.740164457] [person_tracker_node]: PersonTrackerNode initialized with worker thread.
[patrol_command_bridge-10] [WARN] [1776343475.586467757] [patrol_command_bridge]: Timeout while fetching patrol command
[realsense2_camera_node-1] [WARN] [1776343475.928740633] [camera.camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-1] [INFO] [1776343475.934357387] [camera.camera]: Set ROS param depth_module.infra_profile to default: 848x480x30
[realsense2_camera_node-1] [WARN] [1776343476.649800693] [camera.camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-1] [INFO] [1776343476.651110630] [camera.camera]: Set ROS param gyro_fps to default: 200
[realsense2_camera_node-1] [INFO] [1776343476.651232459] [camera.camera]: Set ROS param accel_fps to default: 100
[realsense2_camera_node-1] [INFO] [1776343476.678478786] [camera.camera]: Stopping Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1776343476.678820527] [camera.camera]: Stopping Sensor: RGB Camera
[realsense2_camera_node-1] [INFO] [1776343476.809923397] [camera.camera]: Starting Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1776343476.903578951] [camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 640, Height: 480, FPS: 30
[realsense2_camera_node-1]  16/04 21:44:36,908 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,920 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,931 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,941 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,952 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,963 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,974 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,985 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:36,996 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,007 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1] [INFO] [1776343477.020933083] [camera.camera]: Starting Sensor: RGB Camera
[realsense2_camera_node-1] [INFO] [1776343477.043868816] [camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 640, Height: 480, FPS: 30
[realsense2_camera_node-1] [INFO] [1776343477.086564772] [camera.camera]: RealSense Node Is Up!
[realsense2_camera_node-1]  16/04 21:44:37,323 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,334 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,346 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,357 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,369 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,482 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,493 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,504 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,515 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,525 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,536 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,547 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,558 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,569 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,580 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,591 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[patrol_command_bridge-10] [WARN] [1776343477.592104311] [patrol_command_bridge]: Timeout while fetching patrol command
[realsense2_camera_node-1]  16/04 21:44:37,602 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,612 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,623 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:37,634 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[person_tracker_node-4] /usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.4
[person_tracker_node-4]   warnings.warn(f"A NumPy version >={np_minversion} and <{np_maxversion}"
[person_tracker_node-4] Loading /home/chan/capston_h3c_integration/src/patrol_yolo/model/yolov8n.engine for TensorRT inference...
[person_tracker_node-4] [04/16/2026-21:44:38] [TRT] [I] Loaded engine size: 13 MiB
[person_tracker_node-4] [04/16/2026-21:44:38] [TRT] [W] Using an engine plan file across different models of devices is not recommended and is likely to affect performance or even cause errors.
[person_tracker_node-4] [04/16/2026-21:44:38] [TRT] [I] [MemUsageChange] TensorRT-managed allocation in IExecutionContext creation: CPU +0, GPU +18, now: CPU 0, GPU 30 (MiB)
[patrol_command_bridge-10] [WARN] [1776343479.599325336] [patrol_command_bridge]: Timeout while fetching patrol command
[yamnet_classifier_node-13] [INFO] [1776343480.302471941] [yamnet_classifier_node]: worker ready: {"ready": true, "load_model_sec": 5.440779685974121, "gpus": []}
[yamnet_classifier_node-13] [INFO] [1776343480.303206624] [yamnet_classifier_node]: yamnet_classifier_node started
[patrol_command_bridge-10] [WARN] [1776343481.606978439] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343483.613624145] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343485.619829869] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343487.628372420] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343489.638036778] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343491.647477584] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343493.658820678] [patrol_command_bridge]: Timeout while fetching patrol command
[realsense2_camera_node-1]  16/04 21:44:54,947 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:54,958 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:44:54,969 WARNING [281472392751328] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[patrol_command_bridge-10] [WARN] [1776343495.666799759] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343497.675533410] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343499.684691222] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343501.694049957] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343503.704492687] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343505.712713698] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343507.721292155] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343509.728980529] [patrol_command_bridge]: Timeout while fetching patrol command
[patrol_command_bridge-10] [WARN] [1776343511.737018065] [patrol_command_bridge]: Timeout while fetching patrol command
[person_event_sender_node-6] [WARN] [1776343512.303808889] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343513.732855529] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[patrol_command_bridge-10] [WARN] [1776343513.748793299] [patrol_command_bridge]: Timeout while fetching patrol command
[person_event_sender_node-6] [WARN] [1776343513.825703588] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343513.863709805] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343513.943672136] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.007591445] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.078956737] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.157702221] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.229212350] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.292961410] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.361960924] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.423512435] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.474928875] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.543036294] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.628274643] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.703105495] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.763979959] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.835701359] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.906769297] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343514.975848205] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343515.047379548] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343515.116360369] [person_event_sender_node]: [SEND] skipped (person_present): no rob


 capston_bringup bringup_all.launch.py 
[INFO] [launch]: All log files can be found below /home/chan/.ros/log/2026-04-16-21-52-53-246790-ubuntu-19660
[INFO] [launch]: Default logging verbosity is set to INFO
Warning: Parameter 'server_ip' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'image_topic' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'yolo_mode' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
[INFO] [launch.user]: 🚀 Launching as Normal ROS Node
[INFO] [realsense2_camera_node-1]: process started with pid [19667]
[INFO] [capture_sender-2]: process started with pid [19669]
[INFO] [patrol_http_bridge-3]: process started with pid [19671]
[INFO] [person_tracker_node-4]: process started with pid [19673]
[INFO] [person_detect_control_node-5]: process started with pid [19675]
[INFO] [person_event_sender_node-6]: process started with pid [19677]
[INFO] [robot_pose_sender-7]: process started with pid [19679]
[INFO] [robot_goal_sender-8]: process started with pid [19681]
[INFO] [can_teacher_node-9]: process started with pid [19685]
[INFO] [patrol_command_bridge-10]: process started with pid [19687]
[INFO] [audio_frontend_node-11]: process started with pid [19689]
[INFO] [respeaker_doa_node-12]: process started with pid [19691]
[INFO] [yamnet_classifier_node-13]: process started with pid [19707]
[INFO] [sound_event_manager_node-14]: process started with pid [19716]
[INFO] [clip_transfer_node-15]: process started with pid [19718]
[INFO] [sound_event_monitor_node-16]: process started with pid [19727]
[realsense2_camera_node-1] [INFO] [1776343973.768744077] [camera.camera]: RealSense ROS v4.57.7
[realsense2_camera_node-1] [INFO] [1776343973.769003763] [camera.camera]: Built with LibRealSense v2.57.7
[realsense2_camera_node-1] [INFO] [1776343973.769034867] [camera.camera]: Running with LibRealSense v2.57.7
[sound_event_monitor_node-16] [INFO] [1776343975.146301224] [sound_event_monitor_node]: sound_event_monitor_node started
[realsense2_camera_node-1] [INFO] [1776343975.152400106] [camera.camera]: Device with serial number 241122307367 was found.
[realsense2_camera_node-1] 
[realsense2_camera_node-1] [INFO] [1776343975.153761095] [camera.camera]: Device with physical ID 2-1.2.4.3-5 was found.
[realsense2_camera_node-1] [INFO] [1776343975.153825992] [camera.camera]: Device with name Intel RealSense D455 was found.
[realsense2_camera_node-1] [INFO] [1776343975.154165711] [camera.camera]: Device with port number 2-1.2.4.3 was found.
[realsense2_camera_node-1] [INFO] [1776343975.155516780] [camera.camera]: Device USB type: 3.2
[realsense2_camera_node-1] [INFO] [1776343975.156962667] [camera.camera]: getParameters...
[realsense2_camera_node-1] [INFO] [1776343975.157559832] [camera.camera]: JSON file is not provided
[realsense2_camera_node-1] [INFO] [1776343975.157912991] [camera.camera]: Device Name: Intel RealSense D455
[realsense2_camera_node-1] [INFO] [1776343975.157951520] [camera.camera]: Device Serial No: 241122307367
[realsense2_camera_node-1] [INFO] [1776343975.157983553] [camera.camera]: Device physical port: 2-1.2.4.3-5
[realsense2_camera_node-1] [INFO] [1776343975.158007425] [camera.camera]: Device FW version: 5.17.0.10
[realsense2_camera_node-1] [INFO] [1776343975.158027906] [camera.camera]: Device Product ID: 0x0B5C
[realsense2_camera_node-1] [INFO] [1776343975.158046466] [camera.camera]: Sync Mode: Off
[sound_event_manager_node-14] [INFO] [1776343975.184948927] [sound_event_manager_node]: sound_event_manager_node started
[respeaker_doa_node-12] [INFO] [1776343975.242470951] [respeaker_doa_node]: respeaker_doa_node started
[patrol_command_bridge-10] [INFO] [1776343975.283884409] [patrol_command_bridge]: PatrolCommandBridge started | server=http://192.168.0.221:8000 | waypoints_topic=/patrol/waypoints_json | command_topic=/patrol/command | reload_waypoints_topic=/patrol/reload_waypoints
[robot_goal_sender-8] [INFO] [1776343975.320500132] [robot_goal_sender]: RobotGoalSender started | goal_topic=/goal_pose_2d | next_place_topic=/next_place_id | endpoint=http://192.168.0.221:8000/robot/goal
[audio_frontend_node-11] [INFO] [1776343975.469853327] [audio_frontend_node]: Using input device index=0, name="NVIDIA Jetson Orin NX HDA: HDMI 0 (hw:0,3)", input_channels=6, channel_index=0, sample_rate=16000
[audio_frontend_node-11] Expression 'parameters->channelCount <= maxChans' failed in 'src/hostapi/alsa/pa_linux_alsa.c', line: 1514
[audio_frontend_node-11] Expression 'ValidateParameters( inputParameters, hostApi, StreamDirection_In )' failed in 'src/hostapi/alsa/pa_linux_alsa.c', line: 2818
[robot_pose_sender-7] [INFO] [1776343975.541905197] [robot_pose_sender]: RobotPoseSender started | topic=/robot_pose | status_topic=/robot_status | endpoint=http://192.168.0.221:8000/robot/pose
[audio_frontend_node-11] Traceback (most recent call last):
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/security_audio_frontend/audio_frontend_node", line 33, in <module>
[audio_frontend_node-11]     sys.exit(load_entry_point('security-audio-frontend==0.0.1', 'console_scripts', 'audio_frontend_node')())
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/audio_frontend_node.py", line 258, in main
[audio_frontend_node-11]     node = AudioFrontendNode()
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/audio_frontend_node.py", line 96, in __init__
[audio_frontend_node-11]     self.start_audio_stream()
[audio_frontend_node-11]   File "/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/python3.10/site-packages/security_audio_frontend/audio_frontend_node.py", line 129, in start_audio_stream
[audio_frontend_node-11]     self.stream = sd.InputStream(
[audio_frontend_node-11]   File "/home/chan/.local/lib/python3.10/site-packages/sounddevice.py", line 1457, in __init__
[audio_frontend_node-11]     _StreamBase.__init__(self, kind='input', wrap_callback='array',
[audio_frontend_node-11]   File "/home/chan/.local/lib/python3.10/site-packages/sounddevice.py", line 914, in __init__
[audio_frontend_node-11]     _check(_lib.Pa_OpenStream(self._ptr, iparameters, oparameters,
[audio_frontend_node-11]   File "/home/chan/.local/lib/python3.10/site-packages/sounddevice.py", line 2838, in _check
[audio_frontend_node-11]     raise PortAudioError(errormsg, err)
[audio_frontend_node-11] sounddevice.PortAudioError: Error opening InputStream: Invalid number of channels [PaErrorCode -9998]
[clip_transfer_node-15] [INFO] [1776343975.562947565] [clip_transfer_node]: clip_transfer_node started | server_url=http://192.168.0.221:8000 | pose_topic=/robot_pose
[can_teacher_node-9] [INFO] [1776343975.817171313] [can_teacher_node]: CAN bus connected: can0
[person_detect_control_node-5] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/node.py:440: UserWarning: when declaring parameter named 'start_enabled', declaring a parameter only providing its name is deprecated. You have to either:
[person_detect_control_node-5]  - Pass a name and a default value different to "PARAMETER NOT SET" (and optionally a descriptor).
[person_detect_control_node-5]  - Pass a name and a parameter type.
[person_detect_control_node-5]  - Pass a name and a descriptor with `dynamic_typing=True
[person_detect_control_node-5]   warnings.warn(
[can_teacher_node-9] [ERROR] [1776343975.842294727] [can_teacher_node]: CAN loop error: Error receiving: Network is down [Error Code 100]
[can_teacher_node-9] [INFO] [1776343975.845002433] [can_teacher_node]: CanTeacherNode started | pose_topic=/robot_pose | server=http://192.168.0.221:8000 | can_if=can0 | teach_can_id=0x102
[ERROR] [audio_frontend_node-11]: process has died [pid 19689, exit code 1, cmd '/home/chan/capston_h3c_integration/install/security_audio_frontend/lib/security_audio_frontend/audio_frontend_node --ros-args -r __node:=audio_frontend_node --params-file /home/chan/capston_h3c_integration/install/security_audio_frontend/share/security_audio_frontend/config/frontend.yaml'].
[person_detect_control_node-5] INFO:     Started server process [19675]
[person_detect_control_node-5] INFO:     Waiting for application startup.
[person_detect_control_node-5] INFO:     Application startup complete.
[person_detect_control_node-5] INFO:     Uvicorn running on http://0.0.0.0:8091 (Press CTRL+C to quit)
[person_detect_control_node-5] [INFO] [1776343976.042069955] [person_detect_control_node]: [INIT] PersonDetectControlNode | robot_pose=/robot_pose | enable_topic=/person_tracking/enable | notify_http=0.0.0.0:8091 | startup_publish=disabled_until_first_config
[person_detect_control_node-5] [INFO] [1776343976.044582649] [person_detect_control_node]: [YOLO ENABLE] True reason=startup_mode_global
[person_detect_control_node-5] [INFO] [1776343976.047072558] [person_detect_control_node]: Initial state published: True
[person_event_sender_node-6] [INFO] [1776343976.083031595] [person_event_sender_node]: [INIT] PersonEventSenderNode | tracks=/person_tracking/tracks_json | annotated=/person_tracking/annotated | robot_pose=/robot_pose
[patrol_http_bridge-3] INFO:     Started server process [19671]
[patrol_http_bridge-3] INFO:     Waiting for application startup.
[patrol_http_bridge-3] INFO:     Application startup complete.
[patrol_http_bridge-3] INFO:     Uvicorn running on http://0.0.0.0:8090 (Press CTRL+C to quit)
[patrol_http_bridge-3] [INFO] [1776343976.171224031] [patrol_http_bridge]: HTTP bridge ready at :8090
[capture_sender-2] [INFO] [1776343976.339377529] [capture_sender]: server_url     = http://192.168.0.221:8000
[capture_sender-2] [INFO] [1776343976.340043591] [capture_sender]: signaling_url  = http://192.168.0.221:8001
[capture_sender-2] [INFO] [1776343976.340542194] [capture_sender]: n_frames       = 5
[capture_sender-2] [INFO] [1776343976.341043772] [capture_sender]: sample_dt      = 0.2
[capture_sender-2] [INFO] [1776343976.341533671] [capture_sender]: capture_timeout_s = 5.0
[capture_sender-2] [INFO] [1776343976.342014225] [capture_sender]: post_timeout_s    = 10.0
[capture_sender-2] [INFO] [1776343976.359872429] [capture_sender]: capture sender ready
[patrol_command_bridge-10] [WARN] [1776343978.297688554] [patrol_command_bridge]: Timeout while fetching patrol waypoints
[person_tracker_node-4] [INFO] [1776343979.533393215] [person_tracker_node]: Loading YOLO model: /home/chan/capston_h3c_integration/src/patrol_yolo/model/yolov8n.engine
[person_tracker_node-4] [INFO] [1776343979.552261776] [person_tracker_node]: [MODE] realsense | color_topic=/camera/camera/color/image_raw | depth_topic=/camera/camera/aligned_depth_to_color/image_raw
[person_tracker_node-4] [INFO] [1776343979.555737657] [person_tracker_node]: [CONTROL] YOLO inference default=ON
[person_tracker_node-4] [INFO] [1776343979.556374183] [person_tracker_node]: PersonTrackerNode initialized with worker thread.
[patrol_command_bridge-10] [WARN] [1776343980.303713706] [patrol_command_bridge]: Timeout while fetching patrol command
[realsense2_camera_node-1] [WARN] [1776343980.616236022] [camera.camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-1] [INFO] [1776343980.617599795] [camera.camera]: Set ROS param depth_module.infra_profile to default: 848x480x30
[realsense2_camera_node-1] [INFO] [1776343981.302134168] [camera.camera]: Set ROS param gyro_fps to default: 200
[realsense2_camera_node-1] [INFO] [1776343981.302422911] [camera.camera]: Set ROS param accel_fps to default: 100
[realsense2_camera_node-1] [INFO] [1776343981.332004819] [camera.camera]: Stopping Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1776343981.333679446] [camera.camera]: Stopping Sensor: RGB Camera
[realsense2_camera_node-1] [INFO] [1776343981.471322112] [camera.camera]: Starting Sensor: Depth Module
[realsense2_camera_node-1] [INFO] [1776343981.571560975] [camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 1280, Height: 720, FPS: 30
[realsense2_camera_node-1]  16/04 21:53:01,572 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:01,584 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1] [INFO] [1776343981.598244102] [camera.camera]: Starting Sensor: RGB Camera
[realsense2_camera_node-1] [INFO] [1776343981.628940145] [camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 1280, Height: 720, FPS: 30
[realsense2_camera_node-1] [INFO] [1776343981.676993037] [camera.camera]: RealSense Node Is Up!
[realsense2_camera_node-1]  16/04 21:53:01,902 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:01,913 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:01,924 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:01,935 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:01,947 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:01,958 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,071 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,083 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,095 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,107 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,118 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,130 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,141 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,152 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,163 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,174 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,185 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,196 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,207 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,218 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,229 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,241 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,252 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,265 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,276 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,287 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,298 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,309 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[patrol_command_bridge-10] [WARN] [1776343982.309941156] [patrol_command_bridge]: Timeout while fetching patrol command
[realsense2_camera_node-1]  16/04 21:53:02,320 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,331 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,342 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,353 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,364 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,375 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,386 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,397 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,408 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,419 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,430 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,441 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,452 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,463 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,474 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,485 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,496 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,507 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,518 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,529 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,541 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,552 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,563 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,565 ERROR [281471914797280] (uvc-streamer.cpp:105) uvc streamer watchdog triggered on endpoint: 130
[realsense2_camera_node-1]  16/04 21:53:02,574 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,585 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,596 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,608 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,620 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,631 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,641 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,653 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,664 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,675 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,686 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,697 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,708 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,719 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,730 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,742 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,753 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,764 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,776 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,787 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,797 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,809 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,820 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,831 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,843 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,853 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,864 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,875 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,886 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,897 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,909 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,921 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,932 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,943 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,955 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,967 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,978 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:02,989 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,001 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,012 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,027 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,039 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,050 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,061 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[person_tracker_node-4] /usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.4
[person_tracker_node-4]   warnings.warn(f"A NumPy version >={np_minversion} and <{np_maxversion}"
[realsense2_camera_node-1]  16/04 21:53:03,382 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,393 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[person_tracker_node-4] Loading /home/chan/capston_h3c_integration/src/patrol_yolo/model/yolov8n.engine for TensorRT inference...
[realsense2_camera_node-1] [WARN] [1776343983.474682630] [camera.camera]: XXX Hardware Notification:Depth stream start failure,1.77634e+12,Error,Hardware Error
[realsense2_camera_node-1] [WARN] [1776343983.474890570] [camera.camera]: Hardware Notification:Depth stream start failure,1.77634e+12,Error,Hardware Error
[realsense2_camera_node-1]  16/04 21:53:03,607 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,618 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,629 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-1]  16/04 21:53:03,641 WARNING [281472736749792] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[person_tracker_node-4] [04/16/2026-21:53:03] [TRT] [I] Loaded engine size: 13 MiB
[person_tracker_node-4] [04/16/2026-21:53:03] [TRT] [W] Using an engine plan file across different models of devices is not recommended and is likely to affect performance or even cause errors.
[person_tracker_node-4] [04/16/2026-21:53:03] [TRT] [I] [MemUsageChange] TensorRT-managed allocation in IExecutionContext creation: CPU +0, GPU +18, now: CPU 0, GPU 30 (MiB)
[patrol_command_bridge-10] [WARN] [1776343984.317748536] [patrol_command_bridge]: Timeout while fetching patrol command
[person_event_sender_node-6] [WARN] [1776343984.643780982] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343984.707064724] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343984.776377649] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343984.851789903] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343984.919672494] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.000331772] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.061159780] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[yamnet_classifier_node-13] [INFO] [1776343985.071625186] [yamnet_classifier_node]: worker ready: {"ready": true, "load_model_sec": 5.589294195175171, "gpus": []}
[yamnet_classifier_node-13] [INFO] [1776343985.072923774] [yamnet_classifier_node]: yamnet_classifier_node started
[person_event_sender_node-6] [WARN] [1776343985.134541047] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.207032663] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.294657463] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.340903275] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.419720241] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.492703643] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.549940855] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.625550297] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.699455671] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.762708819] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.830720820] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.899231871] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343985.978458158] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.033064402] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.088741773] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.168120062] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.236475941] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.293460156] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[patrol_command_bridge-10] [WARN] [1776343986.326804894] [patrol_command_bridge]: Timeout while fetching patrol command
[person_event_sender_node-6] [WARN] [1776343986.367960870] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.432607519] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.505143358] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.576208479] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.647738282] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.731989346] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.802825854] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.880199972] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343986.951602348] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343987.012264273] [person_event_sender_node]: [SEND] skipped (person_present): no robot pose cached
[person_event_sender_node-6] [WARN] [1776343987.055827627]


ear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'default_patrol_enabled' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'place_prefix' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'can_interface' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'teach_can_id' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'debounce_sec' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'http_timeout_sec' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'queue_size' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'waypoints_topic' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'command_topic' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'reload_waypoints_topic' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
Warning: Parameter 'command_poll_period_sec' is not supported. Supported parameters are:
['accel_fps', 'accelerate_gpu_with_glsl', 'align_depth.enable', 'angular_velocity_cov', 'base_frame_id', 'camera_name', 'camera_namespace', 'clip_distance', 'colorizer.enable', 'config_file', 'decimation_filter.enable', 'decimation_filter.filter_magnitude', 'depth_mapping_camera.labeled_point_cloud_profile', 'depth_mapping_camera.occupancy_profile', 'depth_module.color_format', 'depth_module.color_profile', 'depth_module.depth_format', 'depth_module.depth_profile', 'depth_module.enable_auto_exposure', 'depth_module.exposure', 'depth_module.exposure.1', 'depth_module.exposure.2', 'depth_module.gain', 'depth_module.gain.1', 'depth_module.gain.2', 'depth_module.hdr_enabled', 'depth_module.infra1_format', 'depth_module.infra2_format', 'depth_module.infra_format', 'depth_module.infra_profile', 'depth_module.inter_cam_sync_mode', 'device_type', 'diagnostics_period', 'disparity_filter.enable', 'enable_accel', 'enable_color', 'enable_depth', 'enable_gyro', 'enable_infra', 'enable_infra1', 'enable_infra2', 'enable_labeled_point_cloud', 'enable_motion', 'enable_occupancy', 'enable_rgbd', 'enable_safety', 'enable_sync', 'gyro_fps', 'hdr_merge.enable', 'hole_filling_filter.enable', 'initial_reset', 'json_file_path', 'linear_accel_cov', 'log_level', 'motion_fps', 'output', 'pointcloud.allow_no_texture_points', 'pointcloud.enable', 'pointcloud.ordered_pc', 'pointcloud.stream_filter', 'pointcloud.stream_index_filter', 'publish_tf', 'reconnect_timeout', 'rgb_camera.color_format', 'rgb_camera.color_profile', 'rgb_camera.enable_auto_exposure', 'rosbag_filename', 'rosbag_loop', 'rotation_filter.enable', 'rotation_filter.rotation', 'safety_camera.safety_mode', 'serial_no', 'spatial_filter.enable', 'temporal_filter.enable', 'tf_prefix', 'tf_publish_rate', 'unite_imu_method', 'usb_port_id', 'wait_for_device_timeout']
[INFO] [launch.user]: 🚀 Launching as Normal ROS Node
[INFO] [realsense2_camera_node-16]: process started with pid [28567]
[INFO] [capture_sender-1]: process started with pid [28516]
[INFO] [patrol_http_bridge-2]: process started with pid [28518]
[INFO] [person_tracker_node-3]: process started with pid [28520]
[INFO] [person_detect_control_node-4]: process started with pid [28522]
[INFO] [person_event_sender_node-5]: process started with pid [28524]
[INFO] [robot_pose_sender-6]: process started with pid [28526]
[INFO] [robot_goal_sender-7]: process started with pid [28528]
[INFO] [can_teacher_node-8]: process started with pid [28530]
[INFO] [patrol_command_bridge-9]: process started with pid [28532]
[INFO] [audio_frontend_node-10]: process started with pid [28534]
[INFO] [respeaker_doa_node-11]: process started with pid [28536]
[INFO] [yamnet_classifier_node-12]: process started with pid [28541]
[INFO] [sound_event_manager_node-13]: process started with pid [28547]
[INFO] [clip_transfer_node-14]: process started with pid [28556]
[INFO] [sound_event_monitor_node-15]: process started with pid [28565]
[realsense2_camera_node-16] [INFO] [1776410346.079584585] [camera.camera]: RealSense ROS v4.57.7
[realsense2_camera_node-16] [INFO] [1776410346.079820030] [camera.camera]: Built with LibRealSense v2.57.7
[realsense2_camera_node-16] [INFO] [1776410346.079857649] [camera.camera]: Running with LibRealSense v2.57.7
[sound_event_manager_node-13] [INFO] [1776410346.373709593] [sound_event_manager_node]: sound_event_manager_node started
[respeaker_doa_node-11] [INFO] [1776410346.384260943] [respeaker_doa_node]: respeaker_doa_node started
[sound_event_monitor_node-15] [INFO] [1776410346.477396354] [sound_event_monitor_node]: sound_event_monitor_node started
[robot_goal_sender-7] [INFO] [1776410346.479456647] [robot_goal_sender]: RobotGoalSender started | goal_topic=/goal_pose_2d | next_place_topic=/next_place_id | endpoint=http://192.168.0.221:8000/robot/goal
[patrol_command_bridge-9] [INFO] [1776410346.740266329] [patrol_command_bridge]: PatrolCommandBridge started | server=http://192.168.0.221:8000 | waypoints_topic=/patrol/waypoints_json | command_topic=/patrol/command | reload_waypoints_topic=/patrol/reload_waypoints
[patrol_command_bridge-9] [INFO] [1776410346.825675961] [patrol_command_bridge]: Published patrol waypoints | n=2
[robot_pose_sender-6] [INFO] [1776410346.837778069] [robot_pose_sender]: RobotPoseSender started | topic=/robot_pose | status_topic=/robot_status | endpoint=http://192.168.0.221:8000/robot/pose
[can_teacher_node-8] [INFO] [1776410346.876633123] [can_teacher_node]: CAN bus connected: can0
[can_teacher_node-8] [INFO] [1776410346.910381498] [can_teacher_node]: CanTeacherNode started | pose_topic=/robot_pose | server=http://192.168.0.221:8000 | can_if=can0 | teach_can_id=0x102
[realsense2_camera_node-16] [INFO] [1776410347.007665573] [camera.camera]: Device with serial number 241122307367 was found.
[realsense2_camera_node-16] 
[realsense2_camera_node-16] [INFO] [1776410347.007799560] [camera.camera]: Device with physical ID 2-1.3-6 was found.
[realsense2_camera_node-16] [INFO] [1776410347.007828183] [camera.camera]: Device with name Intel RealSense D455 was found.
[realsense2_camera_node-16] [INFO] [1776410347.008154169] [camera.camera]: Device with port number 2-1.3 was found.
[realsense2_camera_node-16] [INFO] [1776410347.008192172] [camera.camera]: Device USB type: 3.2
[realsense2_camera_node-16] [INFO] [1776410347.008344632] [camera.camera]: getParameters...
[realsense2_camera_node-16] [INFO] [1776410347.008730361] [camera.camera]: JSON file is not provided
[realsense2_camera_node-16] [INFO] [1776410347.008772686] [camera.camera]: Device Name: Intel RealSense D455
[realsense2_camera_node-16] [INFO] [1776410347.008795226] [camera.camera]: Device Serial No: 241122307367
[realsense2_camera_node-16] [INFO] [1776410347.008811137] [camera.camera]: Device physical port: 2-1.3-6
[realsense2_camera_node-16] [INFO] [1776410347.008824136] [camera.camera]: Device FW version: 5.17.0.10
[realsense2_camera_node-16] [INFO] [1776410347.008836654] [camera.camera]: Device Product ID: 0x0B5C
[realsense2_camera_node-16] [INFO] [1776410347.008869150] [camera.camera]: Sync Mode: Off
[capture_sender-1] [INFO] [1776410347.060821818] [capture_sender]: server_url     = http://192.168.0.221:8000
[capture_sender-1] [INFO] [1776410347.061593820] [capture_sender]: signaling_url  = http://192.168.0.221:8001
[capture_sender-1] [INFO] [1776410347.062201836] [capture_sender]: n_frames       = 5
[capture_sender-1] [INFO] [1776410347.062764389] [capture_sender]: sample_dt      = 0.2
[capture_sender-1] [INFO] [1776410347.063285193] [capture_sender]: capture_timeout_s = 5.0
[capture_sender-1] [INFO] [1776410347.063810543] [capture_sender]: post_timeout_s    = 10.0
[capture_sender-1] [INFO] [1776410347.084594588] [capture_sender]: capture sender ready
[clip_transfer_node-14] [INFO] [1776410347.119283626] [clip_transfer_node]: clip_transfer_node started | server_url=http://192.168.0.221:8000 | pose_topic=/robot_pose
[person_event_sender_node-5] [INFO] [1776410347.128590098] [person_event_sender_node]: [INIT] PersonEventSenderNode | tracks=/person_tracking/tracks_json | annotated=/person_tracking/annotated | robot_pose=/robot_pose
[audio_frontend_node-10] [INFO] [1776410347.181174089] [audio_frontend_node]: Using input device index=0, name="ReSpeaker 4 Mic Array (UAC1.0): USB Audio (hw:0,0)", input_channels=6, channel_index=0, sample_rate=16000
[audio_frontend_node-10] [INFO] [1776410347.193292398] [audio_frontend_node]: audio_frontend_node started
[patrol_http_bridge-2] INFO:     Started server process [28518]
[patrol_http_bridge-2] INFO:     Waiting for application startup.
[patrol_http_bridge-2] INFO:     Application startup complete.
[patrol_http_bridge-2] INFO:     Uvicorn running on http://0.0.0.0:8090 (Press CTRL+C to quit)
[patrol_http_bridge-2] [INFO] [1776410347.379294545] [patrol_http_bridge]: HTTP bridge ready at :8090
[person_detect_control_node-4] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/node.py:440: UserWarning: when declaring parameter named 'start_enabled', declaring a parameter only providing its name is deprecated. You have to either:
[person_detect_control_node-4] 	- Pass a name and a default value different to "PARAMETER NOT SET" (and optionally a descriptor).
[person_detect_control_node-4] 	- Pass a name and a parameter type.
[person_detect_control_node-4] 	- Pass a name and a descriptor with `dynamic_typing=True
[person_detect_control_node-4]   warnings.warn(
[person_detect_control_node-4] INFO:     Started server process [28522]
[person_detect_control_node-4] INFO:     Waiting for application startup.
[person_detect_control_node-4] INFO:     Application startup complete.
[person_detect_control_node-4] INFO:     Uvicorn running on http://0.0.0.0:8091 (Press CTRL+C to quit)
[person_detect_control_node-4] [INFO] [1776410347.535462848] [person_detect_control_node]: [INIT] PersonDetectControlNode | robot_pose=/robot_pose | enable_topic=/person_tracking/enable | notify_http=0.0.0.0:8091 | startup_publish=disabled_until_first_config
[person_detect_control_node-4] [INFO] [1776410347.536354654] [person_detect_control_node]: [YOLO ENABLE] True reason=startup_mode_global
[person_detect_control_node-4] [INFO] [1776410347.536870976] [person_detect_control_node]: Initial state published: True
[patrol_command_bridge-9] [INFO] [1776410347.700696351] [patrol_command_bridge]: Published patrol command | command=idle
[person_tracker_node-3] [INFO] [1776410350.514576687] [person_tracker_node]: Loading YOLO model: /home/chan/capston_h3c_integration/src/patrol_yolo/model/yolov8n.engine
[person_tracker_node-3] [INFO] [1776410350.529479107] [person_tracker_node]: [MODE] realsense | color_topic=/camera/camera/color/image_raw | depth_topic=/camera/camera/aligned_depth_to_color/image_raw
[person_tracker_node-3] [INFO] [1776410350.531481291] [person_tracker_node]: [CONTROL] YOLO inference default=ON
[person_tracker_node-3] [INFO] [1776410350.532079125] [person_tracker_node]: PersonTrackerNode initialized with worker thread.
[audio_frontend_node-10] [INFO] [1776410350.696939874] [audio_frontend_node]: Trigger detected: event_id=5d88ad3d-89d3-446e-acfa-9d4cae52b6af, level=-9.92 dBFS, doa=-142.0
[audio_frontend_node-10] [INFO] [1776410351.239862464] [audio_frontend_node]: Published clip info: /tmp/security_audio_clips/5d88ad3d-89d3-446e-acfa-9d4cae52b6af.wav
[realsense2_camera_node-16] [WARN] [1776410351.755464018] [camera.camera]: re-enable the stream for the change to take effect.
[realsense2_camera_node-16] [INFO] [1776410351.756458819] [camera.camera]: Set ROS param depth_module.infra_profile to default: 848x480x30
[realsense2_camera_node-16] [INFO] [1776410352.486899473] [camera.camera]: Set ROS param gyro_fps to default: 200
[realsense2_camera_node-16] [INFO] [1776410352.487256036] [camera.camera]: Set ROS param accel_fps to default: 100
[realsense2_camera_node-16] [INFO] [1776410352.512442729] [camera.camera]: Stopping Sensor: Depth Module
[realsense2_camera_node-16] [INFO] [1776410352.512623459] [camera.camera]: Stopping Sensor: RGB Camera
[realsense2_camera_node-16] [INFO] [1776410352.644045365] [camera.camera]: Starting Sensor: Depth Module
[realsense2_camera_node-16] [INFO] [1776410352.750885872] [camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 1280, Height: 720, FPS: 30
[realsense2_camera_node-16]  17/04 16:19:12,755 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-16]  17/04 16:19:12,766 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-16] [INFO] [1776410352.779850549] [camera.camera]: Starting Sensor: RGB Camera
[realsense2_camera_node-16] [INFO] [1776410352.806872848] [camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 1280, Height: 720, FPS: 30
[realsense2_camera_node-16] [INFO] [1776410352.861583154] [camera.camera]: RealSense Node Is Up!
[realsense2_camera_node-16]  17/04 16:19:12,980 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-16]  17/04 16:19:12,991 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-16]  17/04 16:19:13,002 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-16]  17/04 16:19:13,114 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-16]  17/04 16:19:13,125 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[realsense2_camera_node-16]  17/04 16:19:13,136 WARNING [281471922727136] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable, number: 11
[person_tracker_node-3] /usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.4
[person_tracker_node-3]   warnings.warn(f"A NumPy version >={np_minversion} and <{np_maxversion}"
[person_tracker_node-3] Loading /home/chan/capston_h3c_integration/src/patrol_yolo/model/yolov8n.engine for TensorRT inference...
[person_tracker_node-3] [04/17/2026-16:19:13] [TRT] [I] Loaded engine size: 13 MiB
[person_tracker_node-3] [04/17/2026-16:19:13] [TRT] [I] [MemUsageChange] TensorRT-managed allocation in IExecutionContext creation: CPU +0, GPU +18, now: CPU 0, GPU 30 (MiB)
[yamnet_classifier_node-12] [INFO] [1776410356.320084429] [yamnet_classifier_node]: worker ready: {"ready": true, "load_model_sec": 5.624163389205933, "gpus": []}
[yamnet_classifier_node-12] [INFO] [1776410356.321120755] [yamnet_classifier_node]: yamnet_classifier_node started
[yamnet_classifier_node-12] [INFO] [1776410356.731201017] [yamnet_classifier_node]: Classified event_id=5d88ad3d-89d3-446e-acfa-9d4cae52b6af, label=speech, conf=0.774, infer_sec=0.405
[sound_event_manager_node-13] [INFO] [1776410356.732367200] [sound_event_manager_node]: Published sound event: id=5d88ad3d-89d3-446e-acfa-9d4cae52b6af, label=speech, conf=0.774, doa=-142.0
[sound_event_monitor_node-15] [INFO] [1776410356.733126203] [sound_event_monitor_node]: [EVENT] label=speech conf=0.774 doa=-142.0 file=/tmp/security_audio_clips/5d88ad3d-89d3-446e-acfa-9d4cae52b6af.wav
[clip_transfer_node-14] [ERROR] [1776410356.733138657] [clip_transfer_node]: Upload failed: event_id=5d88ad3d-89d3-446e-acfa-9d4cae52b6af, error=robot pose is not available yet
[person_event_sender_node-5] [INFO] [1776410365.609081312] [person_event_sender_node]: [STATE] reset to NO_PERSON




0. 
제트팩 버전 확인  ==============================================
dpkg-query --show nvidia-l4t-core
apt show nvidia-jetpack

>>jP 6.2 계열로 확인


파이썬, 넘파이 , 제트팩 버전 확인  ==============================================

python3 --version
python3 -c "import sys; import numpy; print(sys.version); print(numpy.__version__)"
uname -m
>>
JetPack Runtime 6.2.1
Python 3.10.12
aarch64

설치 ==============================================
sudo apt-get update
sudo apt-get install -y python3-pip libopenblas-dev

pip3 install numpy==1.26.4
pip3 uninstall torch torchvision ultralytics -y

wget https://raw.githubusercontent.com/pytorch/pytorch/5c6af2b583709f6176898c017424dc9981023c28/.ci/docker/common/install_cusparselt.sh
sudo CUDA_VERSION=12.6 bash ./install_cusparselt.sh

* 다운받은 wget이 cuda version 적용 및 문법 오류가 있을 수  있는데 gpt 시켜서 ./install_cusparselt.sh를 수정해야한다.

========= 
1. torch
wget https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
python3 -m pip install --no-cache ./torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

python3 - <<EOF
import torch
print(torch.__version__)
print(torch.cuda.is_available())
EOF


2. torchvision

sudo apt-get install -y libjpeg-dev zlib1g-dev libpng-dev git

git clone --branch v0.20.0 https://github.com/pytorch/vision.git
cd vision

python3 -m pip install -v --no-build-isolation .
cd ..

확인 ========================================== 
python3 - <<EOF
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.version.cuda)
if torch.cuda.is_available():
    print(torch.cuda.get_device_name(0))
EOF

python3 -m pip install -U ultralytics

확인 ===
python3 - <<EOF
from ultralytics import YOLO
print("ultralytics ok")
EOF


0.1 pt파일 텐서화 

- python3 export_engine.py --model yolov8n.pt --half
# 추론 이미지 사이즈 640, FP16 최적화 >> engine 파일 생성

- yaml 설정에서 model path를 engine파일의 절대경로로 변경




==================================

1. 
cd ~/ros2_ws
colcon build --packages-select person_tracker_ros
source install/setup.bash

ros2 launch patrol_yolo person_tracker.launch.py


==============================


2. 확인 및 I/O정리

트레킹 결과 토픽 -- /person_tracking/tracks_json
ros2 topic echo /person_tracking/tracks_json


트레킹 시각화 토픽 -- /person_tracking/annotated
rqt_image_view





python3 -c "import sys, torch; print(sys.executable); print(torch.__version__); print(torch.cuda.is_available())"

colcon build --packages-select person_tracker_ros
source install/setup.bash
ros2 launch person_tracker_ros person_tracker.launch.py


odel  package.xml  patrol_yolo  readme.txt  resource  setup.cfg  setup.py
chan@ubuntu:~/capston_h3c_integration/src/patrol_yolo$ python make_engine.py 
=== YOLO → TensorRT Export ===
[INFO] Model not found locally → downloading: yolov26n.pt
Traceback (most recent call last):
  File "/home/chan/capston_h3c_integration/src/patrol_yolo/make_engine.py", line 62, in <module>
    main()
  File "/home/chan/capston_h3c_integration/src/patrol_yolo/make_engine.py", line 35, in main
    model = YOLO(args.model)
  File "/home/chan/.local/lib/python3.10/site-packages/ultralytics/models/yolo/model.py", line 76, in __init__
    super().__init__(model=model, task=task, verbose=verbose)
  File "/home/chan/.local/lib/python3.10/site-packages/ultralytics/engine/model.py", line 144, in __init__
    self._load(model, task=task)
  File "/home/chan/.local/lib/python3.10/site-packages/ultralytics/engine/model.py", line 283, in _load
    self.model, self.ckpt = load_checkpoint(weights)
  File "/home/chan/.local/lib/python3.10/site-packages/ultralytics/nn/tasks.py", line 1515, in load_checkpoint
    ckpt, weight = torch_safe_load(weight)  # load ckpt
  File "/home/chan/.local/lib/python3.10/site-packages/ultralytics/nn/tasks.py", line 1463, in torch_safe_load
    ckpt = torch_load(file, map_location="cpu")
  File "/home/chan/.local/lib/python3.10/site-packages/ultralytics/utils/patches.py", line 178, in torch_load
    return torch.load(*args, **kwargs)
  File "/home/chan/.local/lib/python3.10/site-packages/torch/serialization.py", line 1185, in load
    with _open_file_like(f, "rb") as opened_file:
  File "/home/chan/.local/lib/python3.10/site-packages/torch/serialization.py", line 562, in _open_file_like
    return _open_file(name_or_buffer, mode)
  File "/home/chan/.local/lib/python3.10/site-packages/torch/serialization.py", line 543, in __init__
    super().__init__(open(name, mode))
FileNotFoundError: [Errno 2] No such file or directory: 'yolov26n.pt'
chan@ubuntu:~/capston_h3c_integration/src/patrol_yolo$ python make_engine.py 
=== YOLO → TensorRT Export ===
[INFO] Model not found locally → downloading: yolo26n.pt
Downloading https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt to 'yolo26n.pt': 100% ━━━━━━━━━━━━ 5.3MB 6.1MB/s 0.9s
Model loaded successfully
imgsz     : 512
half      : False
int8      : False
workspace : 2.0 GB
==============================
Ultralytics 8.4.35 🚀 Python-3.10.12 torch-2.5.0a0+872d972e41.nv24.08 CUDA:0 (Orin, 15655MiB)
YOLO26n summary (fused): 122 layers, 2,408,932 parameters, 0 gradients, 5.4 GFLOPs

PyTorch: starting from 'yolo26n.pt' with input shape (1, 3, 512, 512) BCHW and output shape(s) (1, 300, 6) (5.3 MB)
requirements: Ultralytics requirements ['onnxslim>=0.1.71', 'onnxruntime-gpu'] not found, attempting AutoUpdate...
WARNING ⚠️ Retry 1/2 failed: Command 'pip install --no-cache-dir "onnxslim>=0.1.71" "onnxruntime-gpu" ' returned non-zero exit status 1.
WARNING ⚠️ Retry 2/2 failed: Command 'pip install --no-cache-dir "onnxslim>=0.1.71" "onnxruntime-gpu" ' returned non-zero exit status 1.
WARNING ⚠️ requirements: ❌ Command 'pip install --no-cache-dir "onnxslim>=0.1.71" "onnxruntime-gpu" ' returned non-zero exit status 1.
Defaulting to user installation because normal site-packages is not writeable
Collecting onnxslim>=0.1.71
  Downloading onnxslim-0.1.92-py3-none-any.whl.metadata (10 kB)
ERROR: Could not find a version that satisfies the requirement onnxruntime-gpu (from versions: none)

[notice] A new release of pip is available: 26.0.1 -> 26.1
[notice] To update, run: python -m pip install --upgrade pip
ERROR: No matching distribution found for onnxruntime-gpu


ONNX: starting export with onnx 1.14.0 opset 17...
/home/chan/.local/lib/python3.10/site-packages/torch/onnx/symbolic_opset9.py:5645: UserWarning: Exporting aten::index operator of advanced indexing in opset 17 is achieved by combination of multiple ONNX operators, including Reshape, Transpose, Concat, and Gather. If indices include negative values, the exported graph will produce incorrect results.
  warnings.warn(
WARNING ⚠️ ONNX: simplifier failure: No module named 'onnxslim'
ONNX: export success ✅ 6.3s, saved as 'yolo26n.onnx' (9.4 MB)

TensorRT: starting export with TensorRT 10.3.0...
[05/04/2026-22:06:59] [TRT] [I] [MemUsageChange] Init CUDA: CPU +2, GPU +0, now: CPU 742, GPU 4803 (MiB)
[05/04/2026-22:07:02] [TRT] [I] [MemUsageChange] Init builder kernel library: CPU +928, GPU +1096, now: CPU 1713, GPU 5945 (MiB)
[05/04/2026-22:07:02] [TRT] [I] ----------------------------------------------------------------
[05/04/2026-22:07:02] [TRT] [I] Input filename:   yolo26n.onnx
[05/04/2026-22:07:02] [TRT] [I] ONNX IR version:  0.0.8
[05/04/2026-22:07:02] [TRT] [I] Opset version:    17
[05/04/2026-22:07:02] [TRT] [I] Producer name:    pytorch
[05/04/2026-22:07:02] [TRT] [I] Producer version: 2.5.0
[05/04/2026-22:07:02] [TRT] [I] Domain:           
[05/04/2026-22:07:02] [TRT] [I] Model version:    0
[05/04/2026-22:07:02] [TRT] [I] Doc string:       
[05/04/2026-22:07:02] [TRT] [I] ----------------------------------------------------------------
TensorRT: input "images" with shape(1, 3, 512, 512) DataType.FLOAT
TensorRT: output "output0" with shape(1, 300, 6) DataType.FLOAT
TensorRT: building FP32 engine as yolo26n.engine
[05/04/2026-22:07:02] [TRT] [I] Local timing cache in use. Profiling results in this builder pass will not be stored.
[05/04/2026-22:09:32] [TRT] [I] [GraphReduction] The approximate region cut reduction algorithm is called.
[05/04/2026-22:09:32] [TRT] [I] Detected 1 inputs and 1 output network tensors.
[05/04/2026-22:09:34] [TRT] [I] Total Host Persistent Memory: 615088
[05/04/2026-22:09:34] [TRT] [I] Total Device Persistent Memory: 0
[05/04/2026-22:09:34] [TRT] [I] Total Scratch Memory: 1925120
[05/04/2026-22:09:34] [TRT] [I] [BlockAssignment] Started assigning block shifts. This will take 261 steps to complete.
[05/04/2026-22:09:34] [TRT] [I] [BlockAssignment] Algorithm ShiftNTopDown took 30.0142ms to assign 11 blocks to 261 nodes requiring 11420160 bytes.
[05/04/2026-22:09:34] [TRT] [I] Total Activation Memory: 11419648
[05/04/2026-22:09:34] [TRT] [I] Total Weights Memory: 9799936
[05/04/2026-22:09:34] [TRT] [I] Engine generation completed in 152.68 seconds.
[05/04/2026-22:09:34] [TRT] [I] [MemUsageStats] Peak memory usage of TRT CPU/GPU memory allocators: CPU 1 MiB, GPU 32 MiB
[05/04/2026-22:09:34] [TRT] [I] [MemUsageStats] Peak memory usage during Engine building and serialization: CPU: 2912 MiB
TensorRT: export success ✅ 163.0s, saved as 'yolo26n.engine' (11.4 MB)

Export complete (165.0s)
Results saved to /home/chan/capston_h3c_integration/src/patrol_yolo
Predict:         yolo predict task=detect model=yolo26n.engine imgsz=512 
Validate:        yolo val task=detect model=yolo26n.engine imgsz=512 data=/home/lq/codes/ultralytics/ultralytics/cfg/datasets/coco.yaml  
Visualize:       https://netron.app

✅ Export finished!
→ TensorRT .engine file 생성됨
chan@ubuntu:~/capston_h3c_integration/src/patrol_yolo$ 