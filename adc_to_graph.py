import matplotlib.pyplot as plt

def png_create(data):
    plt.plot(data)
    plt.ylabel('Amplitude')
    plt.xlabel('Time')
    plt.title('Signal amplitude vs time')
    plt.savefig("data.png")

    print("Png ""data.png"" created.")
