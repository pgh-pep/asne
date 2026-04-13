#!/usr/bin/env python3
import time
import smbus2

BUS = 7
ADDR = 0x28

# BNO055 Register addresses
REG_CHIP_ID     = 0x00
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_UNIT_SEL    = 0x3B
REG_CALIB_STAT  = 0x35

# Data registers
REG_EUL_H_LSB   = 0x1A  # Euler angles
REG_LIA_X_LSB   = 0x28  # Linear acceleration
REG_GYR_X_LSB   = 0x14  # Gyroscope
REG_ACC_X_LSB   = 0x08  # Raw accelerometer
REG_MAG_X_LSB   = 0x0E  # Magnetometer
REG_QUA_W_LSB   = 0x20  # Quaternion

# Operation modes
MODE_CONFIG = 0x00
MODE_NDOF   = 0x0C  # Full fusion mode (best for orientation)

def write(bus, reg, val):
    bus.write_byte_data(ADDR, reg, val)

def read_bytes(bus, reg, n):
    return bus.read_i2c_block_data(ADDR, reg, n)

def read_vector(bus, reg, scale=1.0):
    data = read_bytes(bus, reg, 6)
    vals = []
    for i in range(3):
        raw = (data[i*2+1] << 8) | data[i*2]
        if raw > 32767:
            raw -= 65536
        vals.append(raw / scale)
    return vals

def read_euler(bus):
    data = read_bytes(bus, REG_EUL_H_LSB, 6)
    vals = []
    for i in range(3):
        raw = (data[i*2+1] << 8) | data[i*2]
        if raw > 32767:
            raw -= 65536
        vals.append(raw / 16.0)  # degrees
    return vals  # heading, roll, pitch

def read_quaternion(bus):
    data = read_bytes(bus, REG_QUA_W_LSB, 8)
    vals = []
    for i in range(4):
        raw = (data[i*2+1] << 8) | data[i*2]
        if raw > 32767:
            raw -= 65536
        vals.append(raw / 16384.0)
    return vals  # w, x, y, z

def get_calib_status(bus):
    status = read_bytes(bus, REG_CALIB_STAT, 1)[0]
    sys  = (status >> 6) & 0x03
    gyro = (status >> 4) & 0x03
    acc  = (status >> 2) & 0x03
    mag  = (status >> 0) & 0x03
    return sys, gyro, acc, mag

def init_bno055(bus):
    # Verify chip ID
    chip_id = read_bytes(bus, REG_CHIP_ID, 1)[0]
    if chip_id != 0xA0:
        raise RuntimeError(f"Wrong chip ID: 0x{chip_id:02X} (expected 0xA0)")
    print(f"BNO055 found, chip ID: 0x{chip_id:02X}")

    # Reset
    write(bus, REG_SYS_TRIGGER, 0x20)
    time.sleep(0.65)

    # Set power mode normal
    write(bus, REG_PWR_MODE, 0x00)
    time.sleep(0.01)

    # Set units: m/s^2, degrees, celsius
    write(bus, REG_UNIT_SEL, 0x00)

    # Set NDOF fusion mode
    write(bus, REG_OPR_MODE, MODE_NDOF)
    time.sleep(0.02)
    print("BNO055 initialized in NDOF fusion mode\n")

def main():
    bus = smbus2.SMBus(BUS)
    init_bno055(bus)

    print(f"{'Reading':<20} {'X/Heading':>12} {'Y/Roll':>12} {'Z/Pitch':>12}")
    print("-" * 60)

    try:
        while True:
            sys_cal, gyro_cal, acc_cal, mag_cal = get_calib_status(bus)
            euler   = read_euler(bus)
            gyro    = read_vector(bus, REG_GYR_X_LSB, 16.0)   # deg/s
            accel   = read_vector(bus, REG_ACC_X_LSB, 100.0)  # m/s^2
            linacc  = read_vector(bus, REG_LIA_X_LSB, 100.0)  # m/s^2
            quat    = read_quaternion(bus)

            print(f"\033[H\033[J", end="")  # clear screen
            print(f"=== BNO055 IMU Readings  (Ctrl+C to quit) ===\n")
            print(f"Calibration:  SYS={sys_cal}/3  GYR={gyro_cal}/3  ACC={acc_cal}/3  MAG={mag_cal}/3")
            print(f"  (Move sensor to calibrate. SYS=3 means fully calibrated)\n")

            print(f"{'':20} {'X/Heading':>12} {'Y/Roll':>12} {'Z/Pitch':>12}")
            print(f"{'-'*60}")
            print(f"{'Euler (deg)':<20} {euler[0]:>12.2f} {euler[1]:>12.2f} {euler[2]:>12.2f}")
            print(f"{'Gyro (deg/s)':<20} {gyro[0]:>12.2f} {gyro[1]:>12.2f} {gyro[2]:>12.2f}")
            print(f"{'Accel (m/s²)':<20} {accel[0]:>12.2f} {accel[1]:>12.2f} {accel[2]:>12.2f}")
            print(f"{'Linear Acc (m/s²)':<20} {linacc[0]:>12.2f} {linacc[1]:>12.2f} {linacc[2]:>12.2f}")
            print(f"\nQuaternion:  W={quat[0]:>8.4f}  X={quat[1]:>8.4f}  Y={quat[2]:>8.4f}  Z={quat[3]:>8.4f}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()