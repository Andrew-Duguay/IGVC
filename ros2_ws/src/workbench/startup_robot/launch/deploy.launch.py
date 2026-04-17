# Real-robot deploy launch for LittleBlue.
#
# This launch starts, in order:
#   1. robot_state_publisher (real URDF, no Gazebo plugins)
#   2. Real sensor drivers (cameras, lidar, imu, gps, joy)
#   3. robot_localization EKF stack (local + global + navsat_transform)
#   4. /world_pose bridge (EKF global → PoseStamped expected by autonomy)
#   5. Motor bridge (cmd_vel → joy → serial to Arduino)
#   6. Full autonomy stack (lane_follower + detectors + accumulator +
#      lidar obstacle)
#
# Per-course tunables (NML waypoints, camera geometry) live in
# startup_robot/config/course.yaml — edit that, not this launch.
#
# Physical + never-touched parameters are set inline below.

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    deploy_pkg = get_package_share_directory('startup_robot')
    autonomy_pkg = get_package_share_directory('littleblue_autonomy')

    course_yaml = os.path.join(deploy_pkg, 'config', 'course.yaml')
    ekf_yaml = os.path.join(deploy_pkg, 'config', 'ekf.yaml')
    rviz_config = os.path.join(autonomy_pkg, 'config', 'autonomy_rviz.rviz')
    autonomy_params = os.path.join(autonomy_pkg, 'config', 'autonomy_params.yaml')
    urdf_xacro = os.path.join(deploy_pkg, 'urdf', 'littleblue_deploy.urdf.xacro')

    # ── Args ──
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz (off by default on the headless robot)',
    )
    approach_arg = DeclareLaunchArgument(
        'approach', default_value='A',
        description='Autonomy approach mode',
    )

    # ── 1. Robot description + state publisher ─────────────────────
    doc = xacro.process_file(urdf_xacro).toxml()
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': doc, 'use_sim_time': False}],
        output='screen',
    )

    # ── 2. Sensor drivers ───────────────────────────────────────────
    # Cameras: usb_cam, one instance per device. Device paths come from
    # the udev rules so USB enumeration order can't break things.
    left_cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='left_camera',
        parameters=[{
            'video_device': '/dev/camera_left',
            'image_width': 640, 'image_height': 480,
            'pixel_format': 'yuyv', 'framerate': 15.0,
            'camera_frame_id': 'left_camera_link',
            'camera_name': 'left_camera',
        }],
        remappings=[('image_raw', '/left_camera/image_raw'),
                    ('camera_info', '/left_camera/camera_info')],
        output='screen',
    )
    right_cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='right_camera',
        parameters=[{
            'video_device': '/dev/camera_right',
            'image_width': 640, 'image_height': 480,
            'pixel_format': 'yuyv', 'framerate': 15.0,
            'camera_frame_id': 'right_camera_link',
            'camera_name': 'right_camera',
        }],
        remappings=[('image_raw', '/right_camera/image_raw'),
                    ('camera_info', '/right_camera/camera_info')],
        output='screen',
    )

    # Lidar (RPLIDAR via sllidar_ros2). Professor installs the package
    # from the existing real-stack workspace.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'),
                         'launch', 'sllidar_a2m8_launch.py')),
        launch_arguments={
            'serial_port': '/dev/lidar',
            'frame_id': 'lidar_link',
        }.items(),
    )

    # ─── Depth camera driver (professor picks hardware) ────────────
    # Required output (per side):
    #   Left  depth image on  /left_camera/depth/image_raw   (32FC1, metres)
    #   Right depth image on  /right_camera/depth/image_raw  (32FC1, metres)
    #   frame_ids matching left_camera_link / right_camera_link
    #
    # Common picks:
    #   Intel RealSense D435(i)  → ros-iron-realsense2-camera
    #   Stereolabs ZED2          → zed_ros2_wrapper (from source)
    #   Orbbec Astra             → ros-iron-astra-camera
    #
    # If the chosen camera is a single stereo device (like ZED/RealSense)
    # that produces both RGB + depth, you can replace the two `usb_cam`
    # blocks above with two realsense2_camera launches (one per serial
    # number), each remapping its image_raw + depth/image_raw to the
    # left_camera/* or right_camera/* namespace.
    #
    # left_depth_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('realsense2_camera'),
    #                      'launch', 'rs_launch.py')),
    #     launch_arguments={
    #         'serial_no': '<LEFT_CAMERA_SERIAL>',
    #         'depth_module.profile': '640x480x15',
    #         'camera_name': 'left_camera',
    #         'frame_id': 'left_camera_link',
    #     }.items(),
    # )
    # right_depth_launch = ...   # mirror with RIGHT_CAMERA_SERIAL

    # ─── IMU driver (professor fills in) ───────────────────────────
    # Required output:
    #   Topic:        /imu/data
    #   Message:      sensor_msgs/msg/Imu
    #   frame_id:     imu_link
    #   Rate:         >= 50 Hz
    # See DEPLOY.md §3 for the full contract and driver picks.
    # Uncomment + replace <imu_pkg> / <imu>.launch.py with the chosen
    # driver's names, and append `imu_launch` to the LaunchDescription
    # list at the bottom of this file.
    #
    # imu_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('<imu_pkg>'),
    #                      'launch', '<imu>.launch.py')),
    #     launch_arguments={
    #         'port': '/dev/imu',           # udev symlink
    #         'frame_id': 'imu_link',
    #     }.items(),
    # )

    # ─── GPS driver (professor fills in) ───────────────────────────
    # Required output:
    #   Topic:        /gps/fix
    #   Message:      sensor_msgs/msg/NavSatFix
    #   frame_id:     gps_link
    #   Rate:         >= 1 Hz
    # See DEPLOY.md §3 for the full contract and driver picks.
    #
    # gps_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('<gps_pkg>'),
    #                      'launch', '<gps>.launch.py')),
    #     launch_arguments={
    #         'port': '/dev/gps',           # udev symlink
    #         'frame_id': 'gps_link',
    #     }.items(),
    # )

    # ── 3. Localization (robot_localization) ────────────────────────
    ekf_local = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_local_node', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )
    ekf_global = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_global_node', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odometry/filtered_map')],
    )
    navsat = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform_node', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': False}],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('gps/filtered', '/gps/filtered'),
            ('odometry/gps', '/odometry/gps'),
            ('odometry/filtered', '/odometry/filtered_map'),
        ],
    )

    # ── 4. /world_pose bridge ───────────────────────────────────────
    world_pose_bridge = Node(
        package='startup_robot', executable='world_pose_from_ekf.py',
        name='world_pose_from_ekf',
        parameters=[{
            'input_topic': '/odometry/filtered_map',
            'output_topic': '/world_pose',
            'frame_id': 'world',
        }],
        output='screen',
    )

    # ── 5. Motor bridge (cmd_vel → /joy → Arduino via game_controller) ─
    cmd_vel_to_joy = Node(
        package='littleblue_autonomy', executable='cmd_vel_to_joy_node',
        name='cmd_vel_to_joy_node',
        parameters=[{
            'use_sim_time': False,
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
            'publish_rate': 20.0,
        }],
        output='screen',
    )
    # joy_node — publishes /joy from a physical gamepad if plugged in.
    # Required by game_controller/listener regardless of autonomy state
    # (so the operator can override with the gamepad's buttons).
    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )
    # Serial bridge to Arduino — from the existing real-robot stack.
    game_controller = Node(
        package='game_controller', executable='listener',
        name='game_controller_listener',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ── 6. Autonomy stack ───────────────────────────────────────────
    lane_follower = Node(
        package='littleblue_autonomy', executable='lane_follower_node',
        name='lane_follower_node',
        parameters=[
            autonomy_params,
            course_yaml,
            {'use_sim_time': False,
             'approach_mode': LaunchConfiguration('approach')},
        ],
        output='screen',
    )

    # Camera lane detectors: physical/HSV defaults set inline; per-camera
    # mount geometry lives in course.yaml so re-mounting doesn't require
    # a code change.
    _detector_common = {
        'use_sim_time': False,
        'method': 'hsv_threshold',
        # HSV defaults locked for outdoor white-on-grass — see node.
        'hsv_low_v': 170, 'hsv_high_s': 60,
        'frame_skip': 2,
        'subsample_stride': 8,
    }
    left_detector = Node(
        package='littleblue_vision', executable='lane_candidate_node',
        name='left_lane_candidate',
        parameters=[
            course_yaml,
            {**_detector_common,
             'image_topic': '/left_camera/image_raw',
             'output_topic': '/candidate/left_lane_points',
             'debug_topic': '/candidate/left_debug'},
        ],
        output='screen',
    )
    right_detector = Node(
        package='littleblue_vision', executable='lane_candidate_node',
        name='right_lane_candidate',
        parameters=[
            course_yaml,
            {**_detector_common,
             'image_topic': '/right_camera/image_raw',
             'output_topic': '/candidate/right_lane_points',
             'debug_topic': '/candidate/right_debug'},
        ],
        output='screen',
    )
    lane_accumulator = Node(
        package='littleblue_vision', executable='lane_accumulator_node',
        name='lane_accumulator_node',
        parameters=[{
            'use_sim_time': False,
            'mode': 'accumulate',
            'max_range': 8.0,
            'publish_rate': 15.0,
        }],
        output='screen',
    )
    lidar_obstacle = Node(
        package='littleblue_vision', executable='lidar_obstacle_node',
        name='lidar_obstacle_node',
        parameters=[{
            'use_sim_time': False,
            'min_range': 0.5, 'max_range': 8.0,
            'cluster_gap': 0.2, 'min_cluster_points': 3,
        }],
        output='screen',
    )

    # ─── YOLO TensorRT detection (left + right cameras) ────────────
    # Runs on the Jetson GPU. Weights file is a compiled TensorRT
    # engine — see DEPLOY.md §YOLO for how to produce one from a .pt
    # or .onnx on the Jetson.
    yolo_weights = os.environ.get(
        'YOLO_ENGINE',
        os.path.expanduser('~/yolo_weights/yolo11n.engine'))
    yolo_common = {
        'use_sim_time': False,
        'weights': yolo_weights,
        'img_size': 640,
        'conf': 0.5,
        'device': 0,          # GPU index; set to 'cpu' for CPU fallback
    }
    left_yolo = Node(
        package='yolo_trt_ros2', executable='yolo_trt_node',
        name='left_yolo',
        parameters=[{**yolo_common,
                     'image_topic': '/left_camera/image_raw',
                     'detections_topic': '/left_yolo/detections',
                     'annotated_topic': '/left_yolo/image_annotated'}],
        output='screen',
    )
    right_yolo = Node(
        package='yolo_trt_ros2', executable='yolo_trt_node',
        name='right_yolo',
        parameters=[{**yolo_common,
                     'image_topic': '/right_camera/image_raw',
                     'detections_topic': '/right_yolo/detections',
                     'annotated_topic': '/right_yolo/image_annotated'}],
        output='screen',
    )

    # ─── Obstacle projector: YOLO bbox + depth → /obstacle_points ─
    # Publishes alongside lidar_obstacle on the same topic; lane_follower
    # consumes the union.
    projector_common = {
        'use_sim_time': False,
        # RGB intrinsics must match the usb_cam config above. These are
        # rough defaults; refine once the camera is calibrated.
        'rgb_fx': 349.2, 'rgb_fy': 349.2,
        'rgb_cx': 320.0, 'rgb_cy': 240.0,
        'depth_fx': 337.22, 'depth_fy': 337.22,
        'depth_cx': 320.0, 'depth_cy': 240.0,
        'camera_height': 0.32,
        'camera_forward_offset': 0.38,
        'depth_patch_size': 5,
        'min_range': 0.3, 'max_range': 8.0,
        'sync_slop': 0.1,
    }
    left_projector = Node(
        package='littleblue_vision', executable='obstacle_projector_node',
        name='left_obstacle_projector',
        parameters=[{**projector_common,
                     'camera_lateral_offset': 0.28,
                     'detection_topic': '/left_yolo/detections',
                     'depth_topic': '/left_camera/depth/image_raw'}],
        output='screen',
    )
    right_projector = Node(
        package='littleblue_vision', executable='obstacle_projector_node',
        name='right_obstacle_projector',
        parameters=[{**projector_common,
                     'camera_lateral_offset': -0.28,
                     'detection_topic': '/right_yolo/detections',
                     'depth_topic': '/right_camera/depth/image_raw'}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        approach_arg, rviz_arg,
        # Robot description
        rsp,
        # Sensors
        left_cam, right_cam, lidar_launch,
        # imu_launch, gps_launch,  # uncomment once driver pkgs chosen
        # Localization
        ekf_local, ekf_global, navsat,
        world_pose_bridge,
        # Motor path
        cmd_vel_to_joy, joy_node, game_controller,
        # Autonomy
        lane_follower,
        left_detector, right_detector, lane_accumulator, lidar_obstacle,
        # YOLO + depth-projected obstacles (Jetson TensorRT)
        left_yolo, right_yolo,
        left_projector, right_projector,
        rviz_node,
    ])
