#!/usr/bin/env python3
import serial
import time
import argparse
import rospy

# Import the required libraries for NTRIP client and NMEA parsing
# pygnssutils and pynmeagps libraries are used to handle GNSS correction streams and NMEA decoding.
from pygnssutils.gnssntripclient import GNSSNTRIPClient, GGALIVE
from pynmeagps import NMEAReader

# Set up the argument parser for the serial port and baud rate configuration.
parser = argparse.ArgumentParser(description='NTRIP client for RTK serial devices.')
parser.add_argument('-s', '--serial-port', required=False, help='Serial port for the device (e.g. /dev/ttyUSB0)', default='/dev/ttyUSB2')
parser.add_argument('-b', '--baud-rate', type=int, required=False, help='Baud rate for the serial port (e.g. 115200)', default=115200)
args, _ = parser.parse_known_args()

class RTKModule:
    """
    RTKModule is used as an application interface for the GNSSNTRIPClient.
    This class reads NMEA sentences (e.g. GGA messages) from the serial interface and extracts
    latitude, longitude, altitude, and separation (sep) values.
    """
    def __init__(self, ser: serial.Serial):
        # Initialize coordinate values to zero.
        self.lat = self.lon = self.alt = self.sep = 0.0
        # Initialize the NMEAReader with the serial connection.
        self.nmr = NMEAReader(ser)

    def get_coordinates(self):
        """
        Reads one NMEA sentence from the GNSS device.
        Logs the raw NMEA sentence to help with debugging.
        If the sentence is a GGA message, extracts the latitude, longitude, altitude, and separation values.
        """
        # Read raw NMEA data and parse it.
        (raw_data, parsed_data) = self.nmr.read()
        # Log the raw NMEA data for debugging.
        #rospy.logdebug(f"NMEA raw data received: {raw_data}")

        # Check if the parsed data has attribute msgID and if it is a GGA sentence.
        if hasattr(parsed_data, 'msgID') and parsed_data.msgID == 'GGA':
            #rospy.logdebug("GGA message detected. Parsing coordinates...")
            self.lat = parsed_data.lat
            self.lon = parsed_data.lon
            self.alt = parsed_data.alt
            self.sep = parsed_data.sep
            # Log the extracted coordinate values.
            #rospy.logdebug(f"Extracted Coordinates -> Lat: {self.lat}, Lon: {self.lon}, Alt: {self.alt}, Sep: {self.sep}")
        else:
            rospy.logdebug("Non-GGA NMEA sentence received or message format unexpected.")

        # Return a tuple with coordinates.
        return None, self.lat, self.lon, self.alt, self.sep

def main():
    """
    Main function that initializes the serial connection, sets up the NTRIP client, and continuously logs connection status.
    """
    # Open the serial port with a timeout.
    rospy.loginfo(f"Attempting to open serial port: {args.serial_port} at {args.baud_rate} baud")
    with serial.Serial(args.serial_port, baudrate=args.baud_rate, timeout=3) as ser:
        rospy.loginfo("Serial port opened successfully.")
        # Setup NTRIP client settings. Adjust the server, port, mountpoint, username, and password as needed.
        ntripc_settings = {
            # The following parameters may be adjusted for your specific RTK correction stream.
            "server": "45.87.173.105",
            "port": 2101,
            "mountpoint": "Hacettepe",
            "ntripuser": "hacettepe",
            "ntrippassword": "hacettepe2023",
            "ggainterval": 60,
            "ggamode": GGALIVE,
            "output": ser,  # Correction stream output is sent to the serial port.
        }

        # Create an instance of RTKModule for GNSS NMEA parsing.
        rtk = RTKModule(ser)
        # Instantiate the GNSSNTRIPClient using the RTKModule instance.
        ntrpc = GNSSNTRIPClient(app=rtk, logtofile=True)

        # Start the NTRIP client. The run() method connects to the NTRIP server and starts streaming corrections.
        streaming = ntrpc.run(**ntripc_settings)
        rospy.loginfo("NTRIP client started, awaiting correction stream...")

        # Continuously log connection and streaming status.
        while not rospy.is_shutdown():
            rospy.logdebug(f"Current Connection Status -> Connected: {ntrpc.connected}, Streaming: {streaming}")
            time.sleep(2)

if __name__ == '__main__':
    # Initialize the ROS node with DEBUG log level to ensure debug messages are printed.
    rospy.init_node('ntrip_client_node', log_level=rospy.DEBUG)
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Fatal error in ntrip_client: {e}")

