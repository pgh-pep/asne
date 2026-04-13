#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import smbus2
import time
import math

BUS  = 7
ADDR = 0x28

# Registers
REG_CHIP_ID     = 0x00
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_UNIT_SEL    = 0x3B
REG_CALIB_STAT  = 0x35

REG_EUL_H_LSB  = 0x1A  # Euler heading (for yaw)
REG_GYR_X_LSB  = 0x14  # Gyroscope (for yaw rate)
REG_LIA_X_LSB  = 0x28  # Linear acceleration (gravity removed)
REG_QUA_W_LSB  = 0x20  # Quaternion (for orientation)

MODE_CONFIG = 0x00
MODE_NDOF   = 0x0C


class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_imu_node')

        # Parameters
        self.declare_parameter('i2c_bus', BUS)
        self.declare_parameter('i2c_addr', ADDR)
        self.declare_parameter('frequency', 30.0)
        self.declare_parameter('frame_id', 'imu_link')

        bus_num  = self.get_parameter('i2c_bus').value
        addr     = self.get_parameter('i2c_addr').value
        freq     = self.get_parameter('frequency').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher — topic matches your EKF config
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # Init I2C
        self.bus  = smbus2.SMBus(bus_num)
        self.addr = addr
        self._init_sensor()

        # Timer at 30 Hz to match EKF frequency
        self.timer = self.create_timer(1.0 / freq, self.publish_imu)
        self.get_logger().info(f'BNO055 node started on i2c-{bus_num} @ 0x{addr:02X}, publishing at {freq} Hz')

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read(self, reg, n):
        return self.bus.read_i2c_block_data(self.addr, reg, n)

    def _init_sensor(self):
        chip_id = self._read(REG_CHIP_ID, 1)[0]
        if chip_id != 0xA0:
            raise RuntimeError(f'BNO055 not found, chip ID: 0x{chip_id:02X}')

        self._write(REG_OPR_MODE, MODE_CONFIG)
        time.sleep(0.025)
        # self._write(REG_SYS_TRIGGER, 0x20)  # reset
        # time.sleep(0.65)
        self._write(REG_PWR_MODE, 0x00)     # normal power
        time.sleep(0.01)
        self._write(REG_UNIT_SEL, 0x00)     # m/s², degrees, celsius
        self._write(REG_OPR_MODE, MODE_NDOF)
        time.sleep(0.02)
        self.get_logger().info('BNO055 initialized in NDOF mode')

    def _read_vector_3(self, reg, scale):
        data = self._read(reg, 6)
        vals = []
        for i in range(3):
            raw = (data[i*2+1] << 8) | data[i*2]
            if raw > 32767:
                raw -= 65536
            vals.append(raw / scale)
        return vals

    def _read_quaternion(self):
        data = self._read(REG_QUA_W_LSB, 8)
        vals = []
        for i in range(4):
            raw = (data[i*2+1] << 8) | data[i*2]
            if raw > 32767:
                raw -= 65536
            vals.append(raw / 16384.0)
        return vals  # w, x, y, z

    def _get_calib(self):
        s = self._read(REG_CALIB_STAT, 1)[0]
        return (s >> 6) & 0x03  # system calibration 0-3

    def publish_imu(self):
        try:
            quat   = self._read_quaternion()                    # w, x, y, z
            gyro   = self._read_vector_3(REG_GYR_X_LSB, 16.0)  # deg/s → convert below
            linacc = self._read_vector_3(REG_LIA_X_LSB, 100.0) # m/s²

            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Orientation as quaternion (from NDOF fusion)
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]

            # Orientation covariance
            # BNO055 heading accuracy ~2.5°, roll/pitch ~0.5° when calibrated
            msg.orientation_covariance = [
                0.002, 0.0,   0.0,
                0.0,   0.002, 0.0,
                0.0,   0.0,   0.04,   # yaw less accurate
            ]

            # Angular velocity (gyro) — convert deg/s to rad/s
            msg.angular_velocity.x = math.radians(gyro[0])
            msg.angular_velocity.y = math.radians(gyro[1])
            msg.angular_velocity.z = math.radians(gyro[2])
            msg.angular_velocity_covariance = [
                0.0001, 0.0,    0.0,
                0.0,    0.0001, 0.0,
                0.0,    0.0,    0.0001,
            ]

            # Linear acceleration (gravity already removed by BNO055)
            msg.linear_acceleration.x = linacc[0]
            msg.linear_acceleration.y = linacc[1]
            msg.linear_acceleration.z = linacc[2]
            msg.linear_acceleration_covariance = [
                0.01, 0.0,  0.0,
                0.0,  0.01, 0.0,
                0.0,  0.0,  0.01,
            ]

            self.pub.publish(msg)

            # Log calibration status periodically
            calib = self._get_calib()
            if calib < 3:
                self.get_logger().warn(
                    f'IMU not fully calibrated: SYS={calib}/3 — move sensor in figure-8',
                    throttle_duration_sec=5.0
                )

        except Exception as e:
            self.get_logger().error(f'IMU read failed: {e}')

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()