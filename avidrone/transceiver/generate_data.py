import csv

mockTransceiver_data = open(
    "mockTransceiver_data.csv", "w", encoding="UTF8", newline=""
)
writer = csv.writer(mockTransceiver_data)

header = ["degree of rotation", "detected", "consistent"]

data = [0, 2, 1]

writer.writerow(header)
writer.writerow(data)

mockTransceiver_data.close()
