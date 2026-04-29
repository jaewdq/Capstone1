#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node


class GzCameraBridge(Node):
    def __init__(self):
        super().__init__('gz_camera_bridge')

        model = 'px4vision_0'

        # 전방 카메라 (기존)
        front_gz = f'/world/default/model/{model}/link/camera_link/sensor/camera/image'
        # 하단 카메라 (신규)
        down_gz  = f'/world/default/model/{model}/link/down_camera_link/sensor/down_camera/image'
        down_info_gz = f'/world/default/model/{model}/link/down_camera_link/sensor/down_camera/camera_info'

        bridges = [
            # gz토픽@ROS메시지타입[gz메시지타입  형식
            (front_gz    + '@sensor_msgs/msg/Image[gz.msgs.Image',      'front_camera',    front_gz),
            (down_gz     + '@sensor_msgs/msg/Image[gz.msgs.Image',      'down_camera',     down_gz),
            (down_info_gz+ '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', 'down_info',  down_info_gz),
        ]

        ros_remaps = {
            front_gz:     '/camera/image_raw',
            down_gz:      '/camera/down_image_raw',
            down_info_gz: '/camera/down_camera_info',
        }

        self.procs = []
        for bridge_str, label, gz_topic in bridges:
            ros_topic = ros_remaps[gz_topic]
            cmd = [
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                bridge_str,
                '--ros-args', '-r', f'{gz_topic}:={ros_topic}'
            ]
            proc = subprocess.Popen(cmd)
            self.procs.append(proc)
            self.get_logger().info(f'[{label}] {gz_topic} → {ros_topic} (pid={proc.pid})')

    def destroy_node(self):
        for proc in self.procs:
            proc.terminate()
        super().destroy_node()


def main():
    rclpy.init()
    node = GzCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
