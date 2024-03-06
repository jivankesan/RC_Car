import math
import smbus
from time import sleep

class GYRO:
    def __init__(self, device_address=0x68):
        self.device_address = device_address
        self.bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
        
        # MPU6050 Registers and their Addresses
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        
        self.init_mpu()
    
    def init_mpu(self):
        """Initialize the MPU6050 with default settings"""
        # Write to sample rate register
        self.bus.write_byte_data(self.device_address, self.SMPLRT_DIV, 7)
        
        # Write to power management register to wake the MPU6050
        self.bus.write_byte_data(self.device_address, self.PWR_MGMT_1, 1)
        
        # Write to Configuration register
        self.bus.write_byte_data(self.device_address, self.CONFIG, 0)
        
        # Write to Gyro configuration register
        self.bus.write_byte_data(self.device_address, self.GYRO_CONFIG, 24)
        
        # Write to interrupt enable register
        self.bus.write_byte_data(self.device_address, self.INT_ENABLE, 1)
    
    def read_raw_data(self, addr):
        """Read raw data from the specified address"""
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)
        
        value = ((high << 8) | low)
        
        if value > 32768:
            value -= 65536
        return value
    
    def get_accel_data(self):
        """Get acceleration data"""
        acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self.read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self.read_raw_data(self.ACCEL_ZOUT_H)
        
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0
        
        return Ax, Ay, Az
    
    def get_gyro_data(self):
        """Get gyroscope data"""
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
        
        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0
        
        return Gx, Gy, Gz
    
    def get_orientation(self):
        """Get the orientation of the robot in degrees using a complementary filter"""
        Ax, Ay, Az = self.get_accel_data()
        Gx, Gy, Gz = self.get_gyro_data()
        
        # Calculate the angles from the accelerometer
        roll_acc = math.atan2(Ay, Az) * 180 / math.pi
        pitch_acc = math.atan2(-Ax, math.sqrt(Ay**2 + Az**2)) * 180 / math.pi
        
        # Assuming Gx, Gy, Gz are in degrees per second and dt is in seconds
        # For the first iteration, assume dt=1, but for real applications, measure the time between calls
        dt = 0.01  # Adjust based on your loop rate
        roll_gyro = Gx * dt
        pitch_gyro = Gy * dt
        
        # Complementary filter
        # The ratio here is 0.96 to gyroscope and 0.04 to accelerometer data
        # These ratios are tunable based on your application's requirements
        roll = 0.96 * (self.prev_roll + roll_gyro) + 0.04 * roll_acc
        pitch = 0.96 * (self.prev_pitch + pitch_gyro) + 0.04 * pitch_acc
        
        # Store the current orientation to be used in the next iteration
        self.prev_roll = roll
        self.prev_pitch = pitch
        
        return roll, pitch

    prev_roll = 0  # Static variables to store the previous orientation
    prev_pitch = 0

if __name__ == "__main__":
    mpu = GYRO()
    print("Reading Orientation of the Robot")
    
    while True:
        roll, pitch = mpu.get_orientation()
        print("Roll=%.2f° Pitch=%.2f°" % (roll, pitch))
        sleep(0.01)  # Match the dt in get_orientation
