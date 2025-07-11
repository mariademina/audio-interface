import serial
import serial.tools.list_ports

devices = serial.tools.list_ports.comports()

for device in devices:
        if(device[1][0:3]=="STM"):
            port = device[0]
            print(f"Connected to STM32 on port {port}.")

ser = serial.Serial(port, 115200, timeout=3)

ser.write(b'm')