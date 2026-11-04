from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import os


def generate_launch_description():
    roscopter_dir = get_package_share_directory('roscopter')

    # Forwarded to roscopter.launch.py. Without this, selecting a benchmark planner
    # configuration meant launching roscopter.launch.py directly -- which silently drops the two
    # nodes below, so the waypoint markers (red spheres, green line strip) and the truth
    # transcriber disappear. Forward it instead so the sim launch stays the one entry point.
    planner_param_file_arg = DeclareLaunchArgument(
        'planner_param_file',
        default_value=os.path.join(roscopter_dir, 'params', 'planner_default.yaml'),
        description="Planner limits YAML (e.g. params/forest_benchmark.yaml)."
    )
    planner_param_file = LaunchConfiguration('planner_param_file')

    # LiDAR-inertial odometry. Sim-only for now: roscopter.launch.py (the hardware launch) is
    # deliberately untouched, so these reach the estimator through a scoped SetParameter below.
    use_lio_arg = DeclareLaunchArgument(
        'use_lio', default_value='false',
        description="Run the FAST-LIO stand-in and fuse its position in the estimator."
    )
    use_lio = LaunchConfiguration('use_lio')
    use_gnss_updates_arg = DeclareLaunchArgument(
        'use_gnss_updates', default_value='true',
        description="false = GPS initialises the estimator but never corrects it (GPS-denied)."
    )
    lio_lever_arm_arg = DeclareLaunchArgument(
        'lio_lever_arm_frd', default_value='[0.0, 0.0, 0.0]',
        description="LiDAR IMU offset from the CG, body FRD metres. Shared by stand-in and bridge."
    )
    lio_lever_arm = LaunchConfiguration('lio_lever_arm_frd')
    # Stand-in error model. Zero both to test the bridge's frame and lever-arm math exactly.
    lio_noise_arg = DeclareLaunchArgument(
        'lio_position_noise', default_value='0.02', description="Stand-in white noise, m.")
    lio_drift_arg = DeclareLaunchArgument(
        'lio_drift', default_value='0.05', description="Stand-in drift, m per sqrt(m) travelled.")

    # Must match the sim side: multirotor_standalone.launch.py lidar_output_frame:=sensor.
    use_lidar_preprocessor_arg = DeclareLaunchArgument(
        'use_lidar_preprocessor', default_value='false',
        description="Read the raw /livox/lidar scan through lidar_preprocessor instead of the "
                    "mock's truth-pose world cloud."
    )
    use_lidar_preprocessor = LaunchConfiguration('use_lidar_preprocessor')
    # Pose the preprocessor places points with. estimated_state is what the vehicle has; pointing
    # it at sim/roscopter/state (truth) isolates the preprocessor's own math for testing.
    lidar_pose_topic_arg = DeclareLaunchArgument(
        'lidar_pose_topic', default_value='estimated_state',
        description="State topic lidar_preprocessor uses (estimated_state, or sim/roscopter/state)."
    )
    lidar_lever_arm_arg = DeclareLaunchArgument(
        'lidar_lever_arm_frd', default_value='[0.0, 0.0, 0.0]',
        description="LiDAR sensor origin relative to the CG, body FRD metres."
    )

    base_launch_include = GroupAction([
        SetParameter(name='lidar_topic', value=PythonExpression([
            "'lidar/points_ned' if '", use_lidar_preprocessor, "' == 'true' else '/mock_env/lidar'"])),
        SetParameter(name='use_lio', value=use_lio),
        SetParameter(name='use_gnss_updates', value=LaunchConfiguration('use_gnss_updates')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    roscopter_dir,
                    'launch',
                    'roscopter.launch.py'
                )
            ),
            launch_arguments={'planner_param_file': planner_param_file}.items()
        ),
    ])

    return LaunchDescription([
        planner_param_file_arg,
        use_lio_arg,
        use_gnss_updates_arg,
        lio_lever_arm_arg,
        lio_noise_arg,
        lio_drift_arg,
        use_lidar_preprocessor_arg,
        lidar_pose_topic_arg,
        lidar_lever_arm_arg,
        base_launch_include,
        Node(
            package='roscopter',
            executable='lidar_preprocessor',
            name='lidar_preprocessor',
            output='screen',
            parameters=[{'lever_arm_frd': LaunchConfiguration('lidar_lever_arm_frd')}],
            remappings=[('estimated_state', LaunchConfiguration('lidar_pose_topic'))],
            condition=IfCondition(use_lidar_preprocessor),
        ),
        Node(
            package='roscopter_gcs',
            executable='rviz_waypoint_publisher',
            output='screen',
        ),
        Node(
            package='roscopter_sim',
            executable='sim_state_transcriber',
            output='screen',
            name='roscopter_truth'
        ),
        Node(
            package='roscopter_sim',
            executable='mock_lio_odometry',
            name='mock_lio_odometry',
            output='screen',
            parameters=[{'lever_arm_frd': lio_lever_arm,
                         'position_noise_stdev': LaunchConfiguration('lio_position_noise'),
                         'drift_stdev_per_sqrt_m': LaunchConfiguration('lio_drift')}],
            condition=IfCondition(use_lio),
        ),
        Node(
            package='roscopter',
            executable='lio_ned_bridge',
            name='lio_ned_bridge',
            output='screen',
            parameters=[{'lever_arm_frd': lio_lever_arm}],
            condition=IfCondition(use_lio),
        ),
    ])
