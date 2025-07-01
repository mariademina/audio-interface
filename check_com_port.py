import serial
import serial.tools.list_ports

# Poll serial devices and detect the port used by STM32
devices = serial.tools.list_ports.comports()
for device in devices:
    if(device[1][0:3]=="STM"):
        port = device[0]
        print(port)
        break