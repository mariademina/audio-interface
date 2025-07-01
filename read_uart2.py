# Continuously read and print data received over UART2
import serial
import serial.tools.list_ports
import time

def serial_setup():
    # Poll serial devices and detect the port used by STM32
    port = None
    devices = serial.tools.list_ports.comports()
    for device in devices:
        if(device[1][0:3]=="STM"):
            port = device[0]
            print(f"Connected to STM32 on port {port}.")
    if port is None:
        print("No STM32 device detected. Exiting.")
        exit()

    # Specify correct serial device
    ser = serial.Serial(port, 115200)
    return ser

def send_char(char):
    # Convert the string to a bytes object
    char_bytes = char.encode('utf-8')
    # Send the bytes object
    ser.write(char_bytes)  


ser = serial_setup()
send_char("m")
while True:
    byte = ser.read(1)
    print(f"Byte received: {byte}")
    time.sleep(0.5)