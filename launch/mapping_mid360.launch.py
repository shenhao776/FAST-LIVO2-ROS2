import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 获取包路径
    pkg_share = get_package_share_directory("map_updater")

    # 2. 定义配置文件路径
    # 对应原文件: <rosparam command="load" file="$(find map_updater)/config/mid360.yaml" />
    mid360_config = os.path.join(pkg_share, "config", "mid360.yaml")

    # 对应原文件: <rosparam file="$(find map_updater)/config/camera_pinhole_mid360.yaml" />
    camera_config = os.path.join(pkg_share, "config", "camera_pinhole_mid360.yaml")

    # RViz 配置
    rviz_config = os.path.join(pkg_share, "rviz_cfg", "map_updater.rviz")

    # 3. 声明启动参数
    # 对应原文件: <arg name="rviz" default="true" />
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz"
    )

    # 4. 定义主节点
    # 对应原文件: <node pkg="map_updater" type="map_updater" name="map_updater" ...>
    # 注意: 在 ROS 2 中参数是特定于节点的，所以我们将两个 yaml 文件都传给这个节点
    map_updater_node = Node(
        package="map_updater",
        executable="map_updater_node",  # CMakeLists.txt 中定义的可执行文件名
        name="map_updater",
        output="screen",
        parameters=[mid360_config, camera_config],
    )

    # 5. 定义 RViz 节点
    # 对应原文件: <node ... pkg="rviz" type="rviz" ... />
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # 6. 定义图像解压节点 (Image Transport Republish)
    # 对应原文件: <node pkg="image_transport" type="republish" ... args="compressed in:=... raw out:=..." />
    # ROS 2 语法: republish compressed raw --ros-args -r in/compressed:=... -r out:=...
    republish_node = Node(
        package="image_transport",
        executable="republish",
        name="republish",
        output="screen",
        respawn=True,
        arguments=["compressed", "raw"],
        remappings=[
            # 输入: 订阅 /camera/camera/color/image_raw/compressed
            ("in/compressed", "/camera/camera/color/image_raw/compressed"),
            # 输出: 发布 /camera/camera/color/image_raw (raw 格式)
            ("out", "/camera/camera/color/image_raw"),
        ],
    )

    return LaunchDescription([rviz_arg, map_updater_node, rviz_node, republish_node])
