import serial
import struct

# Fix the import syntax error (must be separate lines)
import serial
import struct

ser = serial.Serial('COM3', 115200, timeout=1)

def Num_To_Accel(x):
    return x / 16384.0 * 9.81

def Num_To_Rate(x):
    return x / 131.0

def Num_To_Temp(x):
    # 1. Convert raw MPU6050 register value to Celsius
    celsius = (x / 340.0) + 36.53
    # 2. Convert that Celsius value to Fahrenheit
    return (celsius * 9.0 / 5.0) + 32.0

def convert(ax, ay, az, temp, gx, gy, gz):
    ax = Num_To_Accel(ax)
    ay = Num_To_Accel(ay)
    az = Num_To_Accel(az)
    temp = Num_To_Temp(temp)
    gx = Num_To_Rate(gx)
    gy = Num_To_Rate(gy)
    gz = Num_To_Rate(gz)
    return ax, ay, az, temp, gx, gy, gz





try:
    while True:
        data = ser.read(14)
        if len(data) == 14:
            ax, ay, az, temp, gx, gy, gz = struct.unpack('>7h', data)
            ax, ay, az, temp, gx, gy, gz = convert(ax, ay, az, temp, gx, gy, gz)
            
            # Using :.2f inside the f-string rounds each printed value to the nearest hundredth
            print(f"A: {ax:.2f} N {ay:.2f} N {az:.2f} N | "
                  f"G: {gx:.2f} °/s {gy:.2f} °/s {gz:.2f} °/s | "
                  f"T: {temp:.2f} F")
                  
except KeyboardInterrupt:
    ser.close()
    print("Connection closed.")
