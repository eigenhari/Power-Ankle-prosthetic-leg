import serial
import time

# Configure the serial port
serial_port = '/dev/ttyUSB0'  # Change this to your serial port
baud_rate = 921600              # Set the baud rate according to your device

# Create a serial connection
with serial.Serial(serial_port, baud_rate, timeout=5) as ser:
    time.sleep(2)  # Wait for connection to initialize

    # Set desired angle
    angle = 180 # Desired angle in degrees (update this if needed)

    # Command loop
    while True:
        # Prepare the hex values to send
        hex_values = [0x02, 0x05, angle, 0x00, 0x00, 0x03, 0xE8, 0x2B, 0x58]  # Adjust based on protocol
        
        # Calculate checksum (example, adjust according to your needs)
        checksum = sum(hex_values) % 256
        hex_values.append(checksum)  # Add checksum to the end
        hex_values.append(0x03)  # End byte (if required)

        byte_data = bytearray(hex_values)

        # Send the data
        ser.write(byte_data)
        print(f"Sent: {byte_data.hex().upper()}")

        # Read the response from the serial port
        time.sleep(0.5)  # Allow time for response
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"Received: {response.hex().upper()}")
        else:
            print("No data received.")

        time.sleep(1)  # Control command frequency
