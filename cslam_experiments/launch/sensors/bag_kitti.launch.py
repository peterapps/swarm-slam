import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def launch_setup(context, *args, **kwargs):
    launch_args = [
        DeclareLaunchArgument('bag_file', default_value='', description=''),
        DeclareLaunchArgument('namespace', default_value='/r0',
                              description=''),
        DeclareLaunchArgument('rate', default_value='1.0', description=''),
        DeclareLaunchArgument('read_ahead_queue_size', default_value='None'),
        DeclareLaunchArgument('bag_start_delay',
                              default_value='5.0',
                              description=''),
        DeclareLaunchArgument('remap_time_offset', default_value='0.0')
    ]
    ns = LaunchConfiguration('namespace').perform(context)
    remappings = [
        ('/kitti/camera_color/left/camera_info',
            ns + '/stereo_camera/left/camera_info',
            None,
            None),
        ('/kitti/camera_color/left/image_rect_color',
            ns + '/stereo_camera/left/image_rect_color',
            None,
            None),
        ('/kitti/camera_color/right/camera_info',
            ns + '/stereo_camera/right/camera_info',
            None,
            None),
        ('/kitti/camera_color/right/image_rect_color',
            ns + '/stereo_camera/right/image_rect_color',
            None,
            None),
        ('/kitti/velo/pointcloud',
            ns + '/pointcloud',
            '/lidar_in',
            '/lidar_out'),
        ('/kitti/oxts/gps/fix',
            ns + '/gps/fix',
            '/gps_in',
            '/gps_out'),
        ('/kitti/oxts/imu',
            ns + '/imu/data',
            '/imu_in',
            '/imu_out'),
        ('/tf_static',
            ns + '/tf_static',
            '/tf_static_in',
            '/tf_static_out'),
        ('/tf',
            ns + '/tf',
            '/tf_in',
            '/tf_out')
    ]
    bag_remap = []
    util_remap = []
    for old, new, util_in, util_out in remappings:
        if util_in is not None:
            bag_remap.append((old, new + '_raw'))
            util_remap.append((util_in, new + '_raw'))
            util_remap.append((util_out, new))
        else:
            bag_remap.append((old, new))

    # bag_remap = {
    #     '/kitti/camera_color/left/camera_info': '/stereo_camera/left/camera_info',
    #     '/kitti/camera_color/left/image_rect_color': '/stereo_camera/left/image_rect_color',
    #     '/kitti/camera_color/right/camera_info': '/stereo_camera/right/camera_info',
    #     '/kitti/camera_color/right/image_rect_color': '/stereo_camera/right/image_rect_color',
    #     '/kitti/velo/pointcloud': '/pointcloud',
    #     '/kitti/oxts/gps/fix': '/gps/fix',
    #     '/kitti/oxts/imu': '/imu/data',
    #     '/tf_static': '/tf_static',
    #     '/tf': '/tf'
    # }.items()
    bag_remap_cmd = ['{}:={}'.format(a, b) for a, b in bag_remap]

    queue_size = LaunchConfiguration('read_ahead_queue_size').perform(context)
    queue_size_cmd = []
    if queue_size != 'None':
        queue_size_cmd = ['--read-ahead-queue-size', queue_size]

    bag_cmd = [
        'ros2', 'bag', 'play',
        LaunchConfiguration('bag_file').perform(context),
        '-r', LaunchConfiguration('rate').perform(context),
        *queue_size_cmd,
        '--remap',
        *bag_remap_cmd
    ]

    return [
        *launch_args,
        Node(
            package='kitti_utils',
            executable='kitti_remap_time',
            name='kitti_remap_time_' + ns[1:],
            remappings=util_remap,
            parameters=[
                {'offset_sec': LaunchConfiguration('remap_time_offset')}
            ]
        ),
        TimerAction(
            period=LaunchConfiguration('bag_start_delay'),
            actions=[
                ExecuteProcess(
                    cmd=bag_cmd,
                    name='bag',
                    output='screen',
                )
            ]),
    ]


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
