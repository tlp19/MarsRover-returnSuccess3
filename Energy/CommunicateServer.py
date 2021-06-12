import serial
import paho.mqtt.client as mqtt

broker_address = "3.8.124.71"
mqtt.Client(client_id="client1")
client = mqtt.Client("client1")
port = 8083
client.connect(broker_address)

arduino_port = "COM4" #serial port of Arduino
baud = 9600 #arduino uno runs at 9600 baud
fileName = "serial-data.csv" #name of the CSV file generated

ser = serial.Serial(arduino_port, baud)
print("Connected to Arduino port:" + arduino_port)
file = open(fileName, "w")
print("Created file")

samples = 200 #how many samples to collect
print_labels = False
line = 0 #start at 0 because our header is 0 (not real data)
while line <= samples:
    # incoming = ser.read(9999)
    # if len(incoming) > 0:
    if print_labels:
        if line==0:
            print("Printing Column Headers")
        else:
            print("Line " + str(line) + ": writing...")
    getData=str(ser.readline())
    data=getData[2:][:-5]
    print(data)
    file = open(fileName, "a")
    file.write(data + "\n") #write data with a newline
    client.publish("status", data)
    line = line+1

print("Data collection complete!")
file.close()