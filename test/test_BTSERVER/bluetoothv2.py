import serial
import time

baudRate = 9600

# Replace 'COM_PORT' with the appropriate COM port of your Bluetooth module on your PC
ser = serial.Serial('/dev/tty.Drone', baudRate, timeout=1)

try:
    while True:
        # Read data from the Bluetooth module
        data_received = ser.readline().decode('utf-8').strip()
        if data_received:
            print(f"Received: {data_received}" + "\n")

        # Send data to the Bluetooth module
        data_to_send = input("Enter data to send: ") + '\n'
        ser.write(data_to_send.encode('utf-8'))

        print("Sending: " + data_to_send, end="")
        print("Sent!")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Closing the connection")
    ser.close()