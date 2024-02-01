import serial

output_file = 'GPS_log.txt'
port = 'COM23'

try:
    # Open serial port
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    print(f"Connected to {port}")
    with open(output_file, 'a') as file:
        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()
            # Check if the line starts with '$GPGLL'
            # if line.startswith('$GPGLL'):
            arr = line.split(',')
            # Write the line to the output file
            data = ','.join(arr[2:6])
            print(data)
            file.write(data + '\n')
except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed.")
