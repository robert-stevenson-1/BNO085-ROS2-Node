import time
from math import atan2, sqrt
from math import pi as PI

from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

orientation_quat = [0,0,0,0] # x, y, z, w (real)
orientation_rad = [0,0,0] # roll, pitch, yaw(heading)  (in rad)
orientation_deg = [0,0,0] # roll, pitch, yaw(heading)  (in deg)

linear_accel = [0,0,0] # x, y, z  (in m/s^2)
gyro = [0,0,0] # x, y, z  (in deg/s)

class BNO085_Publisher(Node):
    def __init__(self):
        super().__init__('BNO085_Publisher')
        # create the publisher for the IMU data
        self.imu_data_publisher = self.create_publisher(
            Imu, # ROS Message
            'IMU_Data',  # Topic
            10)
        # create the publisher for the Robots Orientation data
        self.robot_orientation_publisher  = self.create_publisher(
            Vector3, # ROS Message
            'Robot_Euler_Orientation', # Topic
            10)
        
        # IMU sensor (BNO085)
        self.imu = None
        self.init_sensor()

        # create timer for reading and publishing data (@ rate of ~4hz)
        self.read_send_timer = self.create_timer(0.25, self.read_and_send_imu_data)

    def init_sensor(self):
        i2c = I2C(3)
        try:
            self.imu = BNO08X_I2C(i2c)
        except:
            self.get_logger().error('Failed to connect to BNO085 via I2C...')
            raise Exception('Failed to connect to BNO085 via I2')
            

        # enable the reports from the IMU
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR) # For orientation_quat and Euler (heading) data
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION) # Linear acceleration data
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE) # For Angular Velocity data

        time.sleep(0.5) # Make sure we the IMU is initialized

    def read_and_send_imu_data(self):
        # get the Angular Velocity (gryo data) of the robot
        gyro[0], gyro[1], gyro[2] = self.imu.gyro 
        # get the Linear Acceleration of the robot
        linear_accel[0], linear_accel[1], linear_accel[2] = self.imu.linear_acceleration  
        # get the quaternion representation of the robot's orientation
        orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3] = self.imu.quaternion
        # calculate the euler representation of the robot's orientation
        self.__calc_euler_angles_deg()

        #create messages to publish
        imu_data_msg = Imu()
        robot_ori_euler_msg = Vector3()

        # TODO: Double check that this is true
        # IMU X right, Y forward, Z up
        # ROS Y left, X forward, Z up
        imu_data_msg.angular_velocity.x = gyro[1]
        imu_data_msg.angular_velocity.y = -gyro[0]
        imu_data_msg.angular_velocity.z = gyro[2]
        imu_data_msg.linear_acceleration.x = linear_accel[1]
        imu_data_msg.linear_acceleration.y = -linear_accel[0]
        imu_data_msg.linear_acceleration.z = linear_accel[2]
        imu_data_msg.orientation.x = orientation_quat[1]
        imu_data_msg.orientation.y = -orientation_quat[0]
        imu_data_msg.orientation.z = orientation_quat[2]
        imu_data_msg.orientation.w = orientation_quat[3]

        robot_ori_euler_msg.x = orientation_deg[0] # roll
        robot_ori_euler_msg.y = orientation_deg[1] # pitch
        robot_ori_euler_msg.z = orientation_deg[2] # yaw
        
        #TODO: Look into getting the IMU's Covariance values
        # Following the recommendation here: https://robotics.stackexchange.com/questions/22756/what-would-be-a-way-to-estimate-imu-noise-covariance-matrix
        # we set the covariance values to a small values (just in case they are needed) on the diagonal
        # imu_data_msg.orientation_covariance[0] = 0.01
        # imu_data_msg.orientation_covariance[4] = 0.01
        # imu_data_msg.orientation_covariance[8] = 0.01
        # imu_data_msg.angular_velocity_covariance[0] = 0.01
        # imu_data_msg.angular_velocity_covariance[4] = 0.01
        # imu_data_msg.angular_velocity_covariance[8] = 0.01
        # imu_data_msg.linear_acceleration_covariance[0] = 0.01
        # imu_data_msg.linear_acceleration_covariance[4] = 0.01
        # imu_data_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_data_publisher.publish(imu_data_msg)
        self.robot_orientation_publisher.publish(robot_ori_euler_msg)

    def __calc_euler_orientation_angles_rad(self):
        # calc the roll (x-axis rotation)
        sinr_cosp = 2 * (orientation_quat[3] * orientation_quat[0] + orientation_quat[1] * orientation_quat[2])
        cosr_cosp = 1 - 2 * (orientation_quat[0] * orientation_quat[0] + orientation_quat[1] * orientation_quat[1])
        orientation_rad[0] = atan2(sinr_cosp, cosr_cosp)
        
        # calc the pitch (y-axis rotation)
        sinp = sqrt(1 + 2 * (orientation_quat[3] * orientation_quat[1] - orientation_quat[0] * orientation_quat[2]))
        cosp = sqrt(1 - 2 * (orientation_quat[3] * orientation_quat[1] - orientation_quat[0] * orientation_quat[2]))
        orientation_rad[1] = 2 * atan2(sinp, cosp) - PI/2
        
        # calc the yaw (z-axis rotation)
        siny_cosp = 2 * (orientation_quat[3] * orientation_quat[2] + orientation_quat[0] * orientation_quat[1])
        cosy_cosp = 1 - 2 * (orientation_quat[1] * orientation_quat[1] + orientation_quat[2] * orientation_quat[2])
        orientation_rad[2] = atan2(siny_cosp, cosy_cosp)
            
    def __calc_euler_angles_deg(self):
        self.__calc_euler_orientation_angles_rad()
        orientation_deg[0] = orientation_rad[0] * (180/PI)
        orientation_deg[1] = orientation_rad[1] * (180/PI)
        orientation_deg[2] = 360 - (orientation_rad[2] * (180/PI))


def main(args=None):
    rclpy.init(args=args)
    try:
        bno_publisher = BNO085_Publisher()
        rclpy.spin(bno_publisher)
        bno_publisher.destroy_node()
    except Exception as e: 
        print(orientation_quat)
        print("------------------")
        print(traceback.format_exec())
        #print(e)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
