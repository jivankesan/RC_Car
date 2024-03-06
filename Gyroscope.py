import math
import smbus
from time import sleep
import time

class GYRO:
    def __init__(self, device_address=0x68):
        self.device_address = device_address
        self.bus = smbus.SMBus(1)  # Use SMBus(0) for older version boards
        
        # MPU6050 Registers
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47
        
        self.init_mpu()
    
    def init_mpu(self):
        """Initialize the MPU6050"""
        self.bus.write_byte_data(self.device_address, self.SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.device_address, self.PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.device_address, self.CONFIG, 0)
        self.bus.write_byte_data(self.device_address, self.GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.device_address, self.INT_ENABLE, 1)
    
    def read_raw_data(self, addr):
        """Read raw data from MPU6050"""
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)
        value = ((high << 8) | low)
        if value > 32768:
            value -= 65536
        return value
    
    def get_gyro_data(self):
        """Get gyroscope data"""
        gyro_x = self.read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self.read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self.read_raw_data(self.GYRO_ZOUT_H)
        
        Gx = gyro_x / 131.0
        Gy = gyro_y / 131.0
        Gz = gyro_z / 131.0
        
        return Gx, Gy, Gz
    
    def get_orientation(self, current_time):
        """Estimate the orientation based on gyroscope data"""
        Gx, Gy, Gz = self.get_gyro_data()
        
        # Calculate time difference
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # Update orientation based on gyroscope data
        self.roll += Gx * dt
        self.pitch += Gy * dt
        self.yaw += Gz * dt
        
        return self.roll, self.pitch, self.yaw

    # Initialize orientation and time
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    prev_time = time.time()

    prev_roll = 0
    prev_pitch = 0
    prev_yaw = 0

if __name__ == "__main__":
    gyro = GYRO()
    print("Reading Gyroscope Orientation of the Device")
    
    try:
        while True:
            current_time = time.time()
            roll, pitch, yaw = gyro.get_orientation(current_time)
            print(f"Roll={roll:.2f}° Pitch={pitch:.2f}° Yaw={yaw:.2f}°")
            time.sleep(0.01)  # Small delay to prevent spamming
    except KeyboardInterrupt:
        print("Program terminated.")
