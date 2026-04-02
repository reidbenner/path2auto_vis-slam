#!/usr/bin/env python3
"""
Reads IMU data from O3R port6 via PCIC and publishes
sensor_msgs/Imu on /imu/data at ~1kHz.

The IIM42652 IMU on port6 batches up to 128 samples per frame.
Each sample is published individually using the VPU hardware timestamp.
"""
import threading
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
from agv_imu.deserialize_imu import IMUOutput


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('ip', '10.167.53.27')
        self.declare_parameter('pcic_port', 50016)
        self.declare_parameter('frame_id', 'imu_link')

        ip = self.get_parameter('ip').value
        pcic_port = self.get_parameter('pcic_port').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Imu, '/imu/data', 200)

        self.o3r = O3R(ip)
        self.o3r.set({'ports': {'port6': {'state': 'RUN'}}})

        self.fg = FrameGrabber(self.o3r, pcic_port)
        # Must call start() with no args — O3R_RESULT_IMU is not a standard chunk
        self.fg.start()

        # Track first hardware timestamp to anchor to ROS time
        self._hw_base_us = None
        self._ros_base_ns = None
        self._running = True

        # Poll frames in a background thread (on_new_frame callback doesn't fire for IMU)
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'IMU publisher started: {ip}:{pcic_port} -> /imu/data')

    def _poll_loop(self):
        import time
        while self._running:
            try:
                ok, frame = self.fg.wait_for_frame().wait_for(500)
                if ok:
                    self._on_frame(frame)
            except Exception as e:
                self.get_logger().warn(
                    f'IMU poll error, reconnecting in 2s: {e}',
                    throttle_duration_sec=5.0)
                time.sleep(2.0)
                try:
                    self.fg.stop()
                    self.fg = FrameGrabber(self.o3r, self.get_parameter('pcic_port').value)
                    self.fg.start()
                except Exception as e2:
                    self.get_logger().warn(f'Reconnect failed: {e2}', throttle_duration_sec=5.0)

    def _on_frame(self, frame):
        try:
            raw = frame.get_buffer(buffer_id.O3R_RESULT_IMU)
            imu_output = IMUOutput.parse(raw)
        except (struct.error, Exception) as e:
            self.get_logger().warn(
                f'IMU deserialize error: {e}', throttle_duration_sec=5.0)
            return

        n = imu_output.num_samples
        if n == 0:
            return

        now_ns = self.get_clock().now().nanoseconds

        # Anchor VPU hardware timestamps to ROS time on first frame
        first_hw_us = imu_output.imu_samples[0].timestamp
        if self._hw_base_us is None or first_hw_us < self._hw_base_us:
            self._hw_base_us = first_hw_us
            self._ros_base_ns = now_ns

        for i in range(n):
            sample = imu_output.imu_samples[i]

            # Offset each sample from the ROS anchor using VPU HW timestamps
            offset_us = sample.timestamp - self._hw_base_us
            stamp_ns = self._ros_base_ns + offset_us * 1000

            msg = Imu()
            msg.header.frame_id = self.frame_id
            msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)

            # No orientation output from IIM42652 — signal unknown to EKF
            msg.orientation_covariance[0] = -1.0

            msg.angular_velocity.x = float(sample.gyro_x)
            msg.angular_velocity.y = float(sample.gyro_y)
            msg.angular_velocity.z = float(sample.gyro_z)
            msg.angular_velocity_covariance[0] = 0.002
            msg.angular_velocity_covariance[4] = 0.002
            msg.angular_velocity_covariance[8] = 0.002

            msg.linear_acceleration.x = float(sample.accel_x)
            msg.linear_acceleration.y = float(sample.accel_y)
            msg.linear_acceleration.z = float(sample.accel_z)
            msg.linear_acceleration_covariance[0] = 0.01
            msg.linear_acceleration_covariance[4] = 0.01
            msg.linear_acceleration_covariance[8] = 0.01

            self.pub.publish(msg)

        # Re-anchor each frame so timestamps stay aligned to ROS clock
        last_sample = imu_output.imu_samples[n - 1]
        self._hw_base_us = last_sample.timestamp
        self._ros_base_ns = now_ns + (last_sample.timestamp - first_hw_us) * 1000

    def destroy_node(self):
        self._running = False
        self.fg.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
