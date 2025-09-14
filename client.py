# simple_laptop_client.py
# To be run on the laptop
# This client sends commands to the server and prints the response.

import socket
import logging
import time
from yamspy import MSPy

# --- Configuration ---
COMMAND_PORT = 65000  # The port must match the server's port
SERIAL_PORT = "/dev/ttyACM0" 

# --- Setup logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def get_barometer_reading():
    """
    Connects to the flight controller and retrieves barometer/altitude data.
    """
    print(f"Connecting to FC on {SERIAL_PORT}...")

    # The 'with' statement ensures the connection is properly closed
    # after the block is exited, even if errors occur.
    try:
        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board is None:
                print("Failed to connect to the flight controller.")
                return

            print("Successfully connected to the flight controller.")
            print("Requesting barometer data... Press Ctrl+C to stop.")

            while True:
                # 1. Choose the command to send
                # For altitude data from the barometer, we use 'MSP_ALTITUDE'
                msp_command = 'MSP_ALTITUDE'

                # 2. Send the command to the flight controller
                # The 'send_RAW_msg' method sends the command and waits for a response.
                # It returns the data if successful, otherwise it might return None or raise an exception.
                if board.send_RAW_msg(MSPy.MSPCodes[msp_command], data=[]):
                    
                    # 3. Get the data from the board's attribute
                    # The YAMSPy library stores the received data in an attribute
                    # with the same name as the command (in lowercase).
                    data_handler = board.receive_msg()
                    board.process_msg(data_handler)

                    altitude_data = board.ALTITUDE
                    
                    # 4. Process and display the data
                    # The 'alt' key contains the altitude in centimeters.
                    # The 'vario' key contains the vertical speed in cm/s.
                    if altitude_data:
                        altitude_cm = altitude_data['alt']
                        altitude_m = altitude_cm / 100.0  # Convert to meters
                        vario_cms = altitude_data['vario']

                        print(f"Altitude: {altitude_m:.2f} meters, Vario: {vario_cms} cm/s")
                    else:
                        print("No altitude data received in this cycle.")

                else:
                    print(f"Failed to send {msp_command} command.")

                # Wait for a short period before sending the next request
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping data requests. Exiting.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")

def main():
    """
    Main function to connect to the server and send commands.
    """
    # Get the server's IP address from the user
    pi_ip_address = input("Enter the Raspberry Pi's IP address: ").strip()
    if not pi_ip_address:
        logging.error("No IP address entered. Exiting.")
        return

    # Create a new socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # --- Connect to the server ---
        logging.info(f"Attempting to connect to {pi_ip_address}:{COMMAND_PORT}...")
        client_socket.connect((pi_ip_address, COMMAND_PORT))
        logging.info("Successfully connected to the server.")

        # --- Main command loop ---
        while True:
            # Get command from user input
            command = input("\nEnter command (ping, status, time, bar, or 'exit' to quit): ").strip().lower()

            if not command:
                continue
            
            if command == 'bar':
                get_barometer_reading()
                break
                
            if command == 'exit':
                logging.info("Exit command received. Closing connection.")
                break

            # Send the command to the server
            logging.info(f"Sending command: '{command}'")
            client_socket.sendall(command.encode('utf-8'))

            # Wait to receive the response from the server (up to 1024 bytes)
            response = client_socket.recv(1024)
            
            # Print the decoded response
            print(f"Server response: {response.decode('utf-8')}")

    except socket.gaierror:
        # This error occurs for invalid hostnames or IPs
        logging.error(f"Hostname could not be resolved. Check the IP address: {pi_ip_address}")
    except socket.error as e:
        # This catches other connection errors (e.g., connection refused)
        logging.error(f"Failed to connect or communicate with server: {e}")
    except KeyboardInterrupt:
        logging.info("\nClient is shutting down due to user interrupt (Ctrl+C).")
    finally:
        # --- Clean up the connection ---
        logging.info("Closing socket.")
        client_socket.close()

if __name__ == "__main__":
    main()
