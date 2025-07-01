import wave

file_name = "Song.wav"

def create_wav(data):
    with wave.open(file_name, 'wb') as wav_file:
        wav_file.setnchannels(1) # Single channel audio
        wav_file.setsampwidth(1) # One byte per sample
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(data.tobytes())
    print(f"Wav file ""Song.wav"" created.")