# TO USE :::: START BY PLUGGING EVERYTHING IN, AND REMOVING DEVICE FROM BLUETOOTH BY FORGETTING IT.
# TO USE :::: PLUG IN ALL OF THE CORDS, AND THEN PUSH THE PROGRAM
# TO USE :::: IF TEENSY IS OFF, AND PROGRAM CANNOT PUSH, UNPLUG AND PLUG IN TEENSY CORD AGAIN
# TO USE :::: RUN THIS COMMAND TO FIND THE TTY PORT: ls /dev/tty.*
# TO USE :::: CONNECT TO BLUETOOTH, AND THEN OPEN THE SERIAL WINDOW WITH 'platformio device monitor --port /dev/tty.HC-06'
# TO USE :::: DETAILS SHOULD BE SHOWN HERE

# Some where combo of power cycling and sending the AT commands works, But I dont know how to make it reliable.

import serial
import time
import threading

baudRate = 9600
# baudRate = 38400
# baudRate = 115200

# Replace 'COM_PORT' with the appropriate COM port of your Bluetooth module on your PC
ser = serial.Serial('/dev/tty.Drone', baudRate, timeout=1)

data_to_send = ''

def get_user_input():
    global data_to_send
    while True:
        data_to_send = input("Enter data to send: ") + '\n'

# Create a separate thread for handling user input
input_thread = threading.Thread(target=get_user_input)
input_thread.daemon = True
input_thread.start()

try:
    while True:
        # Read data from the Bluetooth module
        data_received = ser.readline().decode('utf-8').strip()
        if data_received:
            print(f"Received: {data_received}")

        # Send data to the Bluetooth module
        if data_to_send:
            ser.write(data_to_send.encode('utf-8'))
            print("Sending: " + data_to_send, end="")
            print("Sent!")
            data_to_send = ''

        time.sleep(0.1)  # Poll the Bluetooth controller every 10 ms

except KeyboardInterrupt:
    print("Closing the connection")
    ser.close()
