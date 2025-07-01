import csv

def csv_write(data):
    # sets up a new file to write to
    with open("data.csv", 'w', newline='') as csvfile:
        # headers
        header = ["Sample rate = 11ksps"]
        writer = csv.DictWriter(csvfile, header)

        # write the headers
        writer.writeheader()

        # write the values
        for value in data:
            writer.writerow({'Sample rate = 11ksps':value})

    print("CSV file ""data.csv"" created.")