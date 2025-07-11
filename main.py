# Command-line interface
import serial
import serial.tools.list_ports
import time
import math
import adc_to_wav
import adc_to_csv
import adc_to_graph
import numpy as np

counter_size_us = 100
sample_rate = 1 / (counter_size_us * math.pow(10, -6))

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
    ser = serial.Serial(port, 115200, timeout=3)
    return ser


def menu(ser):
    print("------ Menu ------")
    print("Choose an option: \n1. Manual recording mode\n2. Exit")
    mode_choice = int(input())

    match mode_choice:
        case 1:
            # Send command "m" for manual recording to STM32
            ser.write(b'm')
            print("Sent command 'm' for manual mode to processing STM32.")
            manual_mode(ser)
        case 2:
            exit()


def manual_mode(ser):
    try:
        duration = int(input("Enter the time (seconds) to record for: "))
    except ValueError:
        print("Invalid input.")
        return
    
    print(f"Start recording for {duration} seconds? Y/N")
    if (input() == "Y"):
        data = []

        # Record data until duration has passed
        start_time = time.time()
        while (time.time() - start_time) < duration:
            try:
                # Reading a single sample as an integer from binary data
                sample = ser.read(2)
                value = int.from_bytes(sample, byteorder='little')
                print(f"Value received: {value}")
                data.append(int(value))

            except Exception:
                print("Exception; 0 appended to data")
                data.append(0)
    else:
        menu(ser)

    print(f"{duration} seconds of data ({len(data)} samples) collected.\n")
    print(f"Actual sample rate: {len(data) / duration} b/s")
    export(data)


def convert_and_normalise(data):
    # Convert list of data to numpy array and normalise
    data = np.array(data)
    data = (data - data.min()) / (data.max() - data.min())
    data = data * 255
    data = data.astype(np.uint8)

    print(f"Expected sample rate = {sample_rate} b/s")
    return data

        
def export(data):
    # Convert and normalise data
    data = convert_and_normalise(data)

    select = input(f"\n------ Export options ------\n1. WAV\n2. PNG\n3. CSV\n4. Save all\n5. Return to main menu\n> ")
    match select:
        case "1":
            adc_to_wav.create_wav(data)
        case "2":
            adc_to_graph.png_create(data)
        case "3":
            adc_to_csv.csv_write(data)
        case "4":
            adc_to_csv.csv_write(data)
            adc_to_graph.png_create(data)
            adc_to_wav.create_wav(data)
        case "5":
            menu(ser)


if __name__ == "__main__":
    # Specify serial device
    ser = serial_setup()

    while True:
        menu(ser)