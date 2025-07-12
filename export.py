# Exports numpy array of ADC data
import csv
import matplotlib.pyplot as plt
import wave

# CSV
def csv_write(data):
    with open("data.csv", 'w', newline='') as csvfile:
        # headers
        header = ["ADC voltage"]
        writer = csv.DictWriter(csvfile, header)

        # write the headers
        writer.writeheader()

        # write the values
        for value in data:
            writer.writerow({'ADC voltage':value})
    print("CSV file ""data.csv"" created.")

# PNG graph
def png_create(data):
    plt.plot(data)
    plt.ylabel('Amplitude')
    plt.xlabel('Time')
    plt.title('Signal amplitude vs time')
    plt.savefig("data.png")

    print("Png ""data.png"" created.")

# WAV
file_name = "Song.wav"
def create_wav(data):
    with wave.open(file_name, 'wb') as wav_file:
        wav_file.setnchannels(1) # Single channel audio
        wav_file.setsampwidth(1) # One byte per sample
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(data.tobytes())
    print(f"Wav file {file_name} created.")