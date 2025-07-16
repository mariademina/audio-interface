# Command-line interface
import serial
import serial.tools.list_ports
import time
import math
import keyboard
import export # LOL
import numpy as np

counter_size_us = 120
baud_rate = 128000
expected_sample_rate = 1 / (counter_size_us * math.pow(10, -6))
actual_sample_rate = None

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
    ser = serial.Serial(port, baud_rate, timeout=3)
    return ser


def menu(ser):
    print("------ Menu ------")
    print("Choose an option: \n1. Record audio\n2. Exit")
    mode_choice = int(input())

    match mode_choice:
        case 1:
            record(ser)
        case 2:
            exit()


def record(ser):  
    global actual_sample_rate
    print("Press Enter to start/stop recording.")

    # Wait for user to press enter
    while True:
        if keyboard.is_pressed('enter'):
            break
    
    print("Recording... Press Enter to stop")
    data = []
    start_time = time.time()
    bytes_received = 0

    # Clear any buffered data before starting
    ser.reset_input_buffer()

    while True:
        if keyboard.is_pressed('enter'):
            break
        try:
            # Non-blocking read (when enough bytes are available)
            available = ser.in_waiting # number of bytes waiting in serial buffer to be read
            if available >= 2:
                sample = ser.read(2)
                bytes_received += 2
                value = int.from_bytes(sample, byteorder='little')
                data.append(value)
            else:
                time.sleep(0.001) # prevent cpu hogging
        except Exception as e:
            print(f"Exception: {e}")
            return

    duration = time.time() - start_time
    actual_sample_rate = len(data) / duration

    print(f"{duration} seconds of data ({bytes_received} bytes) collected.\n")
    print(f"Expected sample rate = {expected_sample_rate:.1f} b/s")
    print(f"Actual sample rate: {actual_sample_rate:.1f} b/s")
    export_menu(data)


def convert_and_normalise(data):
    data = np.array(data)                                   # convert to numpy array
    data = (data - data.min()) / (data.max() - data.min())  # min-max normalisation
    data = data * 4095                                      # scale to 12-bit range
    data = data.astype(np.uint16)                           # convert to uint16
    return data

        
def export_menu(data):
    # Convert and normalise data
    data = convert_and_normalise(data)

    select = input(f"\n------ Export options ------\n1. WAV\n2. PNG\n3. CSV\n4. Save all\n5. Return to main menu\n> ")
    match select:
        case "1":
            export.create_wav(data, actual_sample_rate)
        case "2":
            export.png_create(data)
        case "3":
            export.csv_write(data)
        case "4":
            export.csv_write(data)
            export.png_create(data)
            export.create_wav(data, actual_sample_rate)
        case "5":
            menu(ser)


if __name__ == "__main__":
    # Specify serial device
    ser = serial_setup()

    while True:
        menu(ser)