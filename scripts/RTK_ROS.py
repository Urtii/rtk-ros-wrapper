#!/usr/bin/env python3
import serial
import struct
import rospy
import argparse
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, UInt32
from geometry_msgs.msg import Twist

# Argument parser with default serial port settings
parser = argparse.ArgumentParser(description='Read data from a serial device and publish to ROS topics.')
parser.add_argument('-s', '--serial-port', required=False, help='Serial port for the device (e.g. /dev/ttyUSB1)', default='/dev/ttyUSB1')
parser.add_argument('-b', '--baud-rate', type=int, required=False, help='Baud rate for the serial port (e.g. 115200)', default=115200)

args, _ = parser.parse_known_args()

# Data fields as per ArNav S series datasheet (Section 5.1.2, Table 6)
# Each entry: bit flag : (Field Name, Length in bytes, Format string for struct.unpack)
data_fields = {
    0x0001: ("Acceleration(XYZ)", 12, "fff"),
    0x0002: ("Gyro(XYZ)", 12, "fff"),
    0x0004: ("Magnetometer(XYZ)", 12, "fff"),
    0x0008: ("Latitude-Longitude-Elevation", 12, "fff"),
    0x0010: ("Roll-Pitch-Yaw", 12, "fff"),
    0x0020: ("Pressure-Temperature-BaroHeight", 12, "fff"),
    0x0040: ("Time", 4, "f"),
    0x0080: ("Status", 1, "B"),
    0x0100: ("Velocity", 12, "fff"),
    0x0200: ("Scaled-LLh", 12, "iii"),
    0x1000: ("Low-Rate-Messages", 12, "")
}

def process_serial_data(ser: serial.Serial, callback):
    """
    Main loop to read and decode binary messages from the ArNav S series INS sensor.
    The protocol follows the datasheet:
      - Message begins with a 1-byte preamble (0x3D).
      - Followed by a 3-byte Message Id (the third byte is reserved and set to 0).
      - Data fields follow based on the bits set in the Message Id.
      - Ends with 2 bytes of CRC.
    """
    while not rospy.is_shutdown():
        try:
            # Initialize a new bytearray for the incoming message
            message = bytearray()

            # Debug: Inform starting to look for preamble (0x3D)
            #rospy.logdebug("Waiting for preamble 0x3D from sensor...")
            # Read one byte at a time until the preamble is found
            byte = ser.read(1)
            while byte != b'\x3D':
                # Optional: print debug info for each byte read (commented out to reduce noise)
                # rospy.logdebug(f"Discarding byte: {byte.hex()}")
                byte = ser.read(1)
            message += byte
            #rospy.logdebug(f"Preamble found: {byte.hex()}")

            # Read the next 3 bytes which represent the Message Id (1st two bytes for flags; 3rd is reserved)
            for i in range(3):
                next_byte = ser.read(1)
                message += next_byte
            # Extract the Message Id integer using only the first two bytes since the last is reserved (should be 0)
            message_id = int.from_bytes(message[1:3], 'big')
            #rospy.loginfo(f"Raw Message Id bytes: {message[1:4].hex()} | Interpreted Message Id (flags): {hex(message_id)}")

            # Dictionary to hold parsed field data
            data = {
                'Preamble': message[:1].hex(),
                'Message ID': message[1:4].hex(),
            }

            # Starting index in the payload as per protocol (after preamble and Message Id)
            data_start = 4
            # Loop through each potential field (bit flag in Message Id) as described in datasheet
            for bit_position, (field_name, field_length, field_format) in data_fields.items():
                # Check if this field is present in the message based on the Message Id bitmask
                if message_id & bit_position:
                    # Debug: Announce that the field is expected to be in the message
                    #rospy.logdebug(f"Field '{field_name}' (bit 0x{bit_position:04x}) is present. Expecting {field_length} bytes.")
                    
                    # Read the field data from the serial stream
                    field_data = ser.read(field_length)
                    message += field_data

                    # Unpack binary data if a format is provided; otherwise, leave as hex string
                    if field_format:
                        try:
                            unpacked_data = struct.unpack('<' + field_format, field_data)
                        except struct.error as e:
                            rospy.logerr(f"Error unpacking {field_name}: {e}")
                            unpacked_data = None
                    else:
                        unpacked_data = field_data.hex()
                    data[field_name] = unpacked_data

                    # Debug: Print the unpacked field value
                    #rospy.logdebug(f"Field '{field_name}' value: {unpacked_data}")

                    # Move the payload pointer forward
                    data_start += field_length

            # Read the final 2 bytes for the CRC (not currently validated)
            crc_bytes = ser.read(2)
            message += crc_bytes
            data['CRC'] = crc_bytes.hex()
            #rospy.logdebug(f"CRC read: {data['CRC']}")

            # Debug: Optionally, print the complete raw message in hex (uncomment if needed)
            #rospy.logdebug(f"Complete raw message: {message.hex()}")

            # Call the provided callback function with the parsed data
            callback(data)

        except Exception as e:
            rospy.logerr(f"Error processing serial data: {e}")

def my_callback(parsed_data):
    """
    Callback function to process parsed data.
    Depending on the available fields, it publishes the corresponding messages to ROS topics.
    """
    # Debug: Print complete parsed data for terminal monitoring
    #rospy.loginfo(f"Parsed Data: {parsed_data}")
    
    # Publish IMU data if acceleration and gyro measurements are available
    if "Acceleration(XYZ)" in parsed_data and "Gyro(XYZ)" in parsed_data:
        imu_msg = Imu()
        imu_msg.header.frame_id = 'map'
        imu_msg.linear_acceleration.x = parsed_data["Acceleration(XYZ)"][0]
        imu_msg.linear_acceleration.y = parsed_data["Acceleration(XYZ)"][1]
        imu_msg.linear_acceleration.z = parsed_data["Acceleration(XYZ)"][2]
        imu_msg.angular_velocity.x = parsed_data["Gyro(XYZ)"][0]
        imu_msg.angular_velocity.y = parsed_data["Gyro(XYZ)"][1]
        imu_msg.angular_velocity.z = parsed_data["Gyro(XYZ)"][2]
        #rospy.logdebug("Publishing IMU data.")
        imu_pub.publish(imu_msg)

    # Publish GPS fix if position data is available
    if "Latitude-Longitude-Elevation" in parsed_data:
        gps_msg = NavSatFix()
        gps_msg.header.frame_id = 'map'
        gps_msg.latitude = parsed_data["Latitude-Longitude-Elevation"][0]
        gps_msg.longitude = parsed_data["Latitude-Longitude-Elevation"][1]
        gps_msg.altitude = parsed_data["Latitude-Longitude-Elevation"][2]
        #rospy.logdebug("Publishing GPS fix data.")
        gps_pub.publish(gps_msg)

    # Publish status if available
    if "Status" in parsed_data:
    	status_msg = UInt32()
    	# The status field is a tuple with a single element; extract that element.
    	status_value = parsed_data["Status"][0] if isinstance(parsed_data["Status"], tuple) else parsed_data["Status"]
    	status_msg.data = status_value
    	#rospy.logdebug("Publishing status data.")
    	status_pub.publish(status_msg)
        
    # Publish velocity if available
    if "Velocity" in parsed_data:
        velocity_msg = Twist()
        velocity_msg.linear.x = parsed_data["Velocity"][0]
        velocity_msg.linear.y = parsed_data["Velocity"][1]
        velocity_msg.linear.z = parsed_data["Velocity"][2]
        #rospy.logdebug("Publishing velocity data.")
        velocity_pub.publish(velocity_msg)

    # Publish Scaled-LLh if available
    if "Scaled-LLh" in parsed_data:
        scaled_llh_msg = NavSatFix()
        scaled_llh_msg.header.frame_id = 'map'
        scaled_llh_msg.latitude = parsed_data["Scaled-LLh"][0]
        scaled_llh_msg.longitude = parsed_data["Scaled-LLh"][1]
        scaled_llh_msg.altitude = parsed_data["Scaled-LLh"][2]
        #rospy.logdebug("Publishing Scaled-LLh data.")
        scaled_llh_pub.publish(scaled_llh_msg)

# Initialize ROS node with a unique name
rospy.init_node('my_rtk_node', log_level=rospy.DEBUG)

# Create ROS publishers for various data outputs
imu_pub = rospy.Publisher('RTK/imu/data', Imu, queue_size=10)
gps_pub = rospy.Publisher('RTK/gps/fix', NavSatFix, queue_size=10)
time_pub = rospy.Publisher('RTK/device_time', Float32, queue_size=10)
status_pub = rospy.Publisher('RTK/status', UInt32, queue_size=10)
velocity_pub = rospy.Publisher('RTK/velocity', Twist, queue_size=10)
scaled_llh_pub = rospy.Publisher('RTK/scaled_llh', NavSatFix, queue_size=10)

# Establish the serial connection with the sensor
try:
    ser = serial.Serial(args.serial_port, baudrate=args.baud_rate)
    if ser.is_open:
        rospy.loginfo(f"Successfully opened serial port {args.serial_port}")
    else:
        rospy.logerr(f"Failed to open serial port {args.serial_port}")
except Exception as e:
    rospy.logerr(f"Error opening serial port: {e}")
    exit(1)

# Start the serial data processing loop
try:
    process_serial_data(ser, my_callback)
except Exception as e:
    rospy.logerr(f"Fatal error in processing serial data: {e}")

