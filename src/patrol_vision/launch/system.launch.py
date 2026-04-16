from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
	server_url = LaunchConfiguration('server_url')
	signaling_url = LaunchConfiguration('signaling_url')
	image_topic = LaunchConfiguration('image_topic')

	n_frames = LaunchConfiguration('n_frames')
	sample_dt = LaunchConfiguration('sample_dt')
	capture_timeout_s = LaunchConfiguration('capture_timeout_s')
	post_timeout_s = LaunchConfiguration('post_timeout_s')
	calib_file = LaunchConfiguration('calib_file')


	return LaunchDescription([
		
		DeclareLaunchArgument('image_topic',default_value='/camera/color/image_raw'),
		DeclareLaunchArgument('server_url', default_value='http://192.168.0.221:8000'),
		DeclareLaunchArgument('signaling_url', default_value='http://192.168.0.221:8001'),
		DeclareLaunchArgument('n_frames', default_value='5'),
		DeclareLaunchArgument('sample_dt', default_value='0.2'),
		DeclareLaunchArgument('capture_timeout_s', default_value='5.0'),
		DeclareLaunchArgument('post_timeout_s', default_value='10.0'),
		DeclareLaunchArgument(
					'calib_file',
					# 파일 위치에 맞춰주기
					default_value='/home/chan/capston_h3c_integration/data/camera_intrinsics.yaml'
				),
				
				Node( # 스트리밍 , 캡처 센더 노드
					package="patrol_vision",
					executable="capture_sender",
					name="capture_sender",
					output="screen",
					parameters=[
						{
							"server_url": server_url,
							"signaling_url": signaling_url,
							'image_topic': image_topic,

							"n_frames": n_frames,
							"sample_dt": sample_dt,
							"capture_timeout_s": capture_timeout_s,
							"post_timeout_s": post_timeout_s,
						}
					],
				),

				# 리얼센스로 변경시에 해당 노드 주석처리
				
				#Node(
				#	# 웹캠 카메라 publish node
				#	package="patrol_vision",
				#	executable="camera_publisher",
				#	name="camera_publisher",
				#	output="screen",
				#	parameters=[
				#		{
				#			"calib_file": calib_file
				#		}
				#	],
				#	remappings=[
				#		('/camera/color/image_raw', image_topic),
				#	],
				#),
				

		Node(
				package="patrol_vision",
				executable="patrol_http_bridge",
				name="patrol_http_bridge",
				output="screen",
			),
	])