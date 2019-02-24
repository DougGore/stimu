# LSM6DS3 6-axis sensor

import smbus
import time
import sys

from enum import IntEnum
from math import sin, tan, cos

from time import sleep

# Default I2C address of the chip
LSM6DS3_ADDR = 0x6a

FUNC_CFG_ADDRESS = 0x01
SENSOR_SYNC_TIME_FRAME = 0x04
FIFO_CTRL1 = 0x06
FIFO_CTRL2 = 0x07
FIFO_CTRL3 = 0x08
FIFO_CTRL4 = 0x09
FIFO_CTRL5 = 0x0a
ORIENT_CFG_G = 0x0b
INT1_CTRL = 0x0d
INT2_CTRL = 0x0e
REG_WHOAMI = 0x0f
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
CTRL4_C = 0x13
CTRL5_C = 0x14
CTRL6_C = 0x15
CTRL7_G = 0x16
CTRL8_XL = 0x17
CTRL9_XL = 0x18
CTRL10_C = 0x19
MASTER_CONFIG = 0x1a
WAKE_UP_SRC = 0x1b
TAP_SRC = 0x1c
D6D_SRC = 0x1d
STATUS_REG = 0x1e
OUT_TEMP_L = 0x20
OUT_TEMP_H = 0x21
OUTX_L_G = 0x22
OUTX_H_G = 0x23
OUTY_L_G = 0x24
OUTY_H_G = 0x25
OUTZ_L_G = 0x26
OUTZ_H_G = 0x27
OUTX_L_XL = 0x28
OUTX_H_XL = 0x29
OUTY_L_XL = 0x2a
OUTY_H_XL = 0x2b
OUTZ_L_XL = 0x2c
OUTZ_H_XL = 0x2d
SENSORHUB1_REG = 0x2e
SENSORHUB2_REG = 0x2f
SENSORHUB3_REG = 0x30
SENSORHUB4_REG = 0x31
SENSORHUB5_REG = 0x32
SENSORHUB6_REG = 0x33
SENSORHUB7_REG = 0x34
SENSORHUB8_REG = 0x35
SENSORHUB9_REG = 0x36
SENSORHUB10_REG = 0x37
SENSORHUB11_REG = 0x38
SENSORHUB12_REG = 0x39
FIFO_STATUS1 = 0x3a
FIFO_STATUS2 = 0x3b
FIFO_STATUS3 = 0x3c
FIFO_STATUS4 = 0x3d
FIFO_DATA_OUT_L = 0x3e
FIFO_DATA_OUT_H = 0x3f
TIMESTAMP0_REG = 0x40
TIMESTAMP1_REG = 0x41
TIMESTAMP2_REG = 0x42
STEP_TIMESTAMP_L = 0x49
STEP_TIMESTAMP_H = 0x4a
STEP_COUNTER_L = 0x4b
STEP_COUNTER_H = 0x4c
SENSORHUB13_REG = 0x4d
SENSORHUB14_REG = 0x4e
SENSORHUB15_REG = 0x4f
SENSORHUB16_REG = 0x50
SENSORHUB17_REG = 0x51
SENSORHUB18_REG = 0x52
FUNC_SRC = 0x53
TAP_CFG = 0x58
TAP_THS_6D = 0x59
INT_DUR2 = 0x5a
WAKE_UP_THS = 0x5b
WAKE_UP_DUR = 0x5c
FREE_FALL = 0x5d
MD1_CFG = 0x5e
MD2_CFG = 0x5f
OUT_MAG_RAW_X_L = 0x66
OUT_MAG_RAW_X_H = 0x67
OUT_MAG_RAW_Y_L = 0x68
OUT_MAG_RAW_Y_H = 0x69
OUT_MAG_RAW_Z_L = 0x6a
OUT_MAG_RAW_Z_H = 0x6b

class DataStatus(IntEnum):
    ACCEL_DATA_AVAILABLE = 0
    GYROSCOPE_DATA_AVAILABLE = 1
    TEMP_DATA_AVAILABLE = 2

class AccelDataRate(IntEnum):
    POWER_DOWN = 0
    RATE_12_5_HZ = 1
    RATE_26_HZ = 2
    RATE_52_HZ = 3
    RATE_104_HZ = 4
    RATE_208_HZ = 5
    RATE_416_HZ = 6
    RATE_833_HZ = 7
    RATE_1_66_KHZ = 8
    RATE_3_33_KHZ = 9
    RATE_6_66_KHZ = 10

class GyroDataRate(IntEnum):
    POWER_DOWN = 0
    RATE_12_5_HZ = 1
    RATE_26_HZ = 2
    RATE_52_HZ = 3
    RATE_104_HZ = 4
    RATE_208_HZ = 5
    RATE_416_HZ = 6
    RATE_833_HZ = 7
    RATE_1_66_KHZ = 8

class GyroScale(IntEnum):
    SCALE_125_DPS = 5
    SCALE_245_DPS = 0
    SCALE_500_DPS = 1
    SCALE_1000_DPS = 2
    SCALE_2000_DPS = 3

class Int1Enable(IntEnum):
    ACCEL_DATA_READY = 0
    GYRO_DATA_READY = 1
    BOOT_STATUS_AVAILABLE = 2
    FIFO_THRESHOLD = 3
    FIFO_OVERRUN = 4
    FIFO_FULL = 5
    SIGNIFICANT_MOTION = 6
    STEP_DETECTOR = 7

class IMU:
    def __init__(self):
        # +-----+------+----------+
        # | Reg | dps  | gain     |
        # +-----+------+----------+
        # |     | 125  | 0.004375 |
        # | 00  | 245  | 0.00875  |
        # | 01  | 500  | 0.0175   |
        # | 10  | 1000 | 0.035    |
        # | 11  | 2000 | 0.07     |
        # +-----+------+----------+
        self.gyro_sensitivty = [ 0.00875, 0.0175, 0.035, 0.07, 0.004375 ]
        self.gyro_sleep = [ 0, 1 / 12.5, 1 / 26, 1 / 52, 1 / 104, 1 / 208,
            1 / 416, 1 / 833, 1 / 1660 ]

        self.i2c_bus = smbus.SMBus(1)
        self.gyro_scale = GyroScale.SCALE_500_DPS
        self.gyro_data_rate = GyroDataRate.RATE_104_HZ
        self.gyro_high_performance = True
        self.gyro_bias_z = 0

        self.config_gyro()

    def read_chip(self, register, length=1):
        try:
            if length == 1:
                return self.i2c_bus.read_byte_data(LSM6DS3_ADDR, register)
            else:
                return self.i2c_bus.read_i2c_block_data(LSM6DS3_ADDR, register, length)
        except OSError:
            print("Error reading data from the control board")
            print(" - Ensure the control board is connected to the Raspberry Pi")
            print(" - Check the connections to ensure they have not come loose")
            sys.exit()

    def write_chip(self, register, value):
        try:
            if type(value) is list:
                self.i2c_bus.write_i2c_block_data(LSM6DS3_ADDR, register, value)
            else:
                self.i2c_bus.write_byte_data(LSM6DS3_ADDR, register, value)
        except OSError:
            print("Error reading data from the control board")
            print(" - Ensure the control board is connected to the Raspberry Pi")
            print(" - Check the connections to ensure they have not come loose")
            sys.exit()

    def clear_bit(self, register, bit):
        """Clear a bit on a register
        
        Args:
            register (int): Register to modify
            bit (int): Index of the bit to clear
        """
        register_value = self.read_chip(register)
        register_value &= ~(1 << bit) & 0xFF
        self.write_chip(register, register_value)

    def set_bit(self, register, bit):
        """Set a bit on a register
        
        Args:
            register (int): Register to modify
            bit (int): Index of the bit to set
        """
        register_value = self.read_chip(register)
        register_value &= ~(1 << bit) & 0xFF
        register_value |= 1 << bit
        self.write_chip(register, register_value)

    @staticmethod
    def twos_comp(val, bits):
        """compute the 2's complement of int value val"""
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val                         # return positive value as is

    def read_chip_u16(self, start_addr):
        return (self.read_chip(start_addr + 1) << 8) | self.read_chip(start_addr)

    def read_chip_s16(self, start_addr):
        result = self.read_chip_u16(start_addr)

        if result > 32767:
            result -= 65536
        return result

    def config_gyro(self):
        ctrl2 = 0
        ctrl7 = 0

        ctrl2 |= self.gyro_data_rate << 4

        if self.gyro_scale < GyroScale.SCALE_125_DPS:
            ctrl2 |= self.gyro_scale << 2
        else:
            ctrl2 |= 0x02

        if self.gyro_high_performance:
            ctrl7 |= 0b10000000

        #initialise the gyroscope
        # writeGRY(LSM9DS0_CTRL_REG1_G, 0b00001111)   #Normal power mode, all axes enabled
        # writeGRY(LSM9DS0_CTRL_REG4_G, 0b00110000)   #Continuos update, 2000 dps full scale
        self.write_chip(CTRL10_C, 0x38)  # Enable x, y, z axis for gyro
        self.write_chip(CTRL2_G, ctrl2)
        self.write_chip(CTRL7_G, ctrl7)

        # writeGRY(FIFO_CTRL5, 0b00000011) # Continous mode
        self.write_chip(FIFO_CTRL5, 0b00000000) # Bypass mode

    def detected(self):
        return self.read_chip(REG_WHOAMI) == 0x69

    def temperature(self):
        temp_reading = self.read_chip_s16(OUT_TEMP_L)
        return 25.0 + temp_reading / 16.0

    def gyroX(self):
        xvalue = self.read_chip_s16(OUTX_L_G)
        return xvalue * self.gyro_sensitivty[self.gyro_scale]

    def gyroY(self):
        yvalue = self.read_chip_s16(OUTY_L_G)
        return yvalue * self.gyro_sensitivty[self.gyro_scale]

    def gyroZ(self):
        zvalue = self.read_chip_s16(OUTZ_L_G)
        return (zvalue * self.gyro_sensitivty[self.gyro_scale]) - self.gyro_bias_z

    def calibrate_gyro(self, seconds=5):
        sleep_period = self.gyro_sleep_period()
        start_time = time.time()
        end_time = start_time
        accumulator = 0
        
        # Run the calibration process given period
        while end_time - start_time < seconds:
            delta = time.time() - end_time
            end_time = time.time()

            accumulator += self.gyroZ() * delta

            sleep(sleep_period)

        timeframe = end_time - start_time
        self.gyro_bias_z = accumulator / timeframe
        
    def gyroZ_bias(self):
        return self.gyro_bias_z

    def gyro_sleep_period(self):
        return self.gyro_sleep[self.gyro_data_rate]

    def enable_sensor():
        set_bit(REG_CTRL_REG1, 7)

    def disable_sensor():
        clear_bit(REG_CTRL_REG1, 7)

    def trigger_oneshot():
        set_bit(REG_CTRL_REG2, 0)

    def status():
        result = []
        status = read_chip(STATUS_REG)

        for bit in DataStatus:
            if status & (1 << bit):
                result.append(DataStatus(bit))

        return result

    def accel_xyz():
        '''
        Returns the acceleration in 10^(-3) g.
        1 g = 9.81 m/s^2
        '''
        config_data = AccelDataRate.RATE_416_HZ << 4
        write_chip(CTRL1_XL, config_data)

        x = self.read_chip_s16(OUTX_L_XL)
        y = self.read_chip_s16(OUTY_L_XL)
        z = self.read_chip_s16(OUTZ_L_XL)

        # Get sensitivity
        # sens = (self.read_u8(LSM6DS3_XG_CTRL1_XL) & LSM6DS3_XL_FS['MASK'])
        # sens = (1 << (sens >> LSM6DS3_XL_FS['SHIFT'])) * 0.061
        # return (x*sens, y*sens, z*sens)

        return x, y, z

    def gyro_performance_mode_disabled(enable):
        if enable:
            set_bit(CTRL7_G, 7)
        else:
            clear_bit(CTRL7_G, 7)

    def data_ready():
        status = read_chip(STATUS_REG)

        if status & 0x01:
            return True

        return False

    def step_count():
        return (read_chip(STEP_COUNTER_H) << 8) | read_chip(STEP_COUNTER_L)

    def step_timestamp():
        return (read_chip(STEP_TIMESTAMP_H) << 8) | read_chip(STEP_TIMESTAMP_L)

    def enable_stepcounter():
        set_bit(CTRL10_C, 2)
        set_bit(TAP_CFG, 6)

    def enable_interrupt(interrupt):
        set_bit(INT1_CTRL, interrupt.value)

    def monitor_values():
        while True:
            gx, gy, gz = gyro_xyz()
            ax, ay, az = accel_xyz()
            print("Accel XYZ: ({x}, {y}, {z})".format(x=gx, y=gy, z=gz))
            print("Gyro XYZ: ({x}, {y}, {z})".format(x=ax, y=ay, z=az))
            sleep(0.2)

    def position_monitor():
        gyro_phi, gyro_theta, gyro_psi = (0, 0, 0)
        new_gyro_phi, new_gyro_theta, new_gyro_psi = (0, 0, 0)

        igx, igy, igz = gyro_xyz()
        iax, iay, iaz = accel_xyz()

        while True:
            # Wx, Wy, Wz = gyro_xyz()

            # Roll
            # Phi’ = Wx + Wy * Sin(Phi) * Tan(Theta) + Wz * Cos(Phi) * Tan(Theta)
            # roll_derivative = Wx + Wy * sin(gyro_phi) * tan(gyro_theta) + Wz * cos(gyro_phi) * tan(gyro_theta)

            # Pitch derivative: Theta’ = Wy * Cos(Phi) – Wz * Sin(Phi)
            # pitch_derivative = Wy * cos(gyro_phi) - Wz * sin(gyro_phi)

            # Yaw derivative: Psi’ = Wy * Sin(Phi) / Cos(Theta) + Wz * Cos(Phi) / Cos(Theta)
            # yaw_derivative = Wy * sin(gyro_phi) / cos(gyro_theta) + Wz * cos(gyro_phi) / cos(gyro_thera)

            # Roll: Phi(t+Ts) = Phi(t) + Phi’ * Ts
            # roll = roll_derivative * 
            
            # Pitch: Theta(t+Ts) = Theta(t) + Theta’ * Ts
            
            # Yaw: Psi(t+Ts) = Psi(t) + Psi’ * Ts        

            gx, gy, gz = gyro_xyz()
            ax, ay, az = accel_xyz()

            rgx = gx - igx
            rgy = gy - igy
            rgz = gz - igz

            rax = ax - iax
            ray = ay - iay
            raz = az - iaz

            print("Accel XYZ: ({x}, {y}, {z})".format(x=rgx, y=rgy, z=rgz))
            print("Gyro XYZ: ({x}, {y}, {z})".format(x=rax, y=ray, z=raz))
            sleep(0.2)

if __name__ == '__main__':
    imu = IMU()

    print("Temprature = {}C".format(imu.temperature()))

    sample_count = 100
    sleep_period = imu.gyro_sleep_period()

    imu.calibrate_gyro()

    bias = imu.gyroZ_bias()
    a = time.time()
    start_time = a
    gyro_z_angle = 0

    while time.time() - start_time < 60:
        GYRz = imu.gyroZ()

        ##Calculate loop Period(LP). How long between Gyro Reads
        b = time.time() - a
        a = time.time()
        LP = b
        # gyro_z_corrected = GYRz - bias
        gyro_z_angle += GYRz * LP

        if sample_count == 0:
            print("Gyro Z = {:.2f}, angle = {:.2f}".format(GYRz, gyro_z_angle))
            sample_count = 100

        sample_count -= 1
        sleep(sleep_period)

    print("Gyro drift after 60 seconds = {} degrees".format(gyro_z_angle))
