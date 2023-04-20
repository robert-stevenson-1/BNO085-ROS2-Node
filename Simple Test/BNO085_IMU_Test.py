import time
from math import atan2, sqrt
from math import pi as PI
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

def main(args=None):
    i2c = I2C(3)
    try:
        imu = BNO08X_I2C(i2c)
    except:
        raise Exception('Failed to connect to BNO085 via I2')

    # enable the reports from the IMU
    imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR) # For orientation_quat and Euler (heading) data
    imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION) # Linear acceleration data
    imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE) # For Angular Velocity data

    orientation_quat = [0,0,0,0] # x, y, z, w (real)
    orientation_rad = [0,0,0] # roll, pitch, yaw(heading)  (in rad)
    orientation_deg = [0,0,0] # roll, pitch, yaw(heading)  (in deg)

    linear_accel = [0,0,0] # x, y, z  (in m/s^2)
    gyro = [0,0,0] # x, y, z  (in deg/s)

    def calcEulerOrientationAngles_rad():
        #calc the roll (x-axis rotation)
        sinr_cosp = 2 * (orientation_quat[3] * orientation_quat[0] + orientation_quat[1] * orientation_quat[2])
        cosr_cosp = 1 - 2 * (orientation_quat[0] * orientation_quat[0] + orientation_quat[1] * orientation_quat[1])
        orientation_rad[0] = atan2(sinr_cosp, cosr_cosp)
        
        #calc the pitch (y-axis rotation)
        sinp = sqrt(1 + 2 * (orientation_quat[3] * orientation_quat[1] - orientation_quat[0] * orientation_quat[2]))
        cosp = sqrt(1 - 2 * (orientation_quat[3] * orientation_quat[1] - orientation_quat[0] * orientation_quat[2]))
        orientation_rad[1] = 2 * atan2(sinp, cosp) - PI/2
        
        #calc the yaw (z-axis rotation)
        siny_cosp = 2 * (orientation_quat[3] * orientation_quat[2] + orientation_quat[0] * orientation_quat[1])
        cosy_cosp = 1 - 2 * (orientation_quat[1] * orientation_quat[1] + orientation_quat[2] * orientation_quat[2])
        orientation_rad[2] = atan2(siny_cosp, cosy_cosp)
        
    def calcEulerAnglesDeg():
        calcEulerOrientationAngles_rad();
        orientation_deg[0] = orientation_rad[0] * (180/PI)
        orientation_deg[1] = orientation_rad[1] * (180/PI)
        orientation_deg[2] = 360 - (180 + orientation_rad[2] * (180/PI))


    while True:
        time.sleep(0.1)
        
        print("Gyro:")
        gyro[0], gyro[1], gyro[2] = imu.gyro  # pylint:disable=no-member
        print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro[0], gyro[1], gyro[2]))
        print("")

        print("Linear Acceleration:")
        linear_accel[0], linear_accel[1], linear_accel[2] = imu.linear_acceleration  # pylint:disable=no-member
        print(
            "X: %0.6f  Y: %0.6f Z: %0.6f m/s^2"
            % (linear_accel[0], linear_accel[1], linear_accel[2])
        )
        print("")
        
        
        print("Rotation Vector orientation_quaternion:")
        orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3] = imu.quaternion  # pylint:disable=no-member
        print(
            "X: %0.6f  Y: %0.6f Z: %0.6f  W(Real): %0.6f" % (orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3])
        )
        print("")
        
        calcEulerAnglesDeg()
        print("Orientation (Degree): ")
        print(
            "roll: %0.2f  pitch: %0.2f heading(yaw): %0.2f " % (orientation_deg[0], orientation_deg[1], orientation_deg[2])
        )
        print("")
        
        time.sleep(0.2)

if __name__ == "__main__":
    main()