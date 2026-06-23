"""Launch as2_camera_overlay for a single drone namespace.

Usage:
    ros2 launch ./launch/camera_overlay.launch.py namespace:=drone0
    ros2 launch ./launch/camera_overlay.launch.py namespace:=drone1 fov:=120.0
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_node(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    fov_h = float(LaunchConfiguration('fov').perform(context))
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    render_scale = float(LaunchConfiguration('render_scale').perform(context))
    max_fps = float(LaunchConfiguration('max_fps').perform(context))
    cell_count = int(LaunchConfiguration('cell_count').perform(context))

    fov_rad = math.radians(fov_h)
    fx = (width / 2.0) / math.tan(fov_rad / 2.0)
    fy = fx * height / width
    cx = width / 2.0
    cy = height / 2.0

    params = {
        'fixed_frame': 'earth',
        'near_plane': 0.05,
        'far_plane': 2000.0,
        'zoom_factor': 1.0,
        'render_scale': render_scale,
        'max_render_fps': max_fps,
        'input.image_topic': f'/{namespace}/sensor_measurements/image_raw',
        'input.camera_info_topic': f'/{namespace}/sensor_measurements/camera_info',
        'output.topic': f'/{namespace}/sensor_measurements/image_with_markers',
        'enabled_displays': [
            'as2_camera_overlay/GridDisplay',
            'as2_camera_overlay/MarkerArrayDisplay',
        ],
        'displays.GridDisplay.reference_frame': 'earth',
        'displays.GridDisplay.plane': 'XY',
        'displays.GridDisplay.cell_count': cell_count,
        'displays.GridDisplay.cell_size': 1.0,
        'displays.GridDisplay.line_width': 0.02,
        'displays.GridDisplay.color_rgba': [0.4, 0.4, 0.4, 0.8],
        'displays.MarkerArrayDisplay.topics': ['/solar_panels'],
        'displays.MarkerArrayDisplay.queue_size': 10,
    }

    node = Node(
        package='as2_camera_overlay',
        executable='as2_camera_overlay_node',
        name=f'camera_overlay_{namespace}',
        output='screen',
        parameters=[params],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0',
                              description='Drone ROS namespace'),
        DeclareLaunchArgument('fov', default_value='90.0',
                              description='Horizontal field of view in degrees'),
        DeclareLaunchArgument('width', default_value='640',
                              description='Camera image width in pixels'),
        DeclareLaunchArgument('height', default_value='480',
                              description='Camera image height in pixels'),
        DeclareLaunchArgument('render_scale', default_value='0.5',
                              description='Render resolution scale (0.1–1.0)'),
        DeclareLaunchArgument('max_fps', default_value='10.0',
                              description='Max overlay render FPS (0 = unlimited)'),
        DeclareLaunchArgument('cell_count', default_value='30',
                              description='Grid cell count per axis'),
        OpaqueFunction(function=_build_node),
    ])
