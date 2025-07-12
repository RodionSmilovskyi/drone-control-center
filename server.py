# simple_pi_server.py
# To be run on the Raspberry Pi
# This server listens for simple text commands and sends back a response.

import socket
import logging
from datetime import datetime
import time
from yamspy import MSPy

# --- Configuration ---
HOST_IP = "0.0.0.0"  # Listen on all available network interfaces
COMMAND_PORT = 65000  # The port to listen on
SERIAL_PORT = "/dev/ttyACM0"

rc_channels = {
    "roll": 1500,
    "pitch": 1500,
    "throttle": 1500,  # IMPORTANT: Throttle must be low to arm
    "yaw": 1500,
    "aux1": 1000,  # Disarmed state
    "aux2": 1000,
    "aux3": 1000,
    "aux4": 1800,
}

rc_channel_order = ["roll", "pitch", "throttle", "yaw", "aux1", "aux2", "aux3", "aux4"]


# --- Setup logging ---
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


def get_barometer_reading():
    """
    Connects to the flight controller and retrieves barometer/altitude data.
    """
    print(f"Connecting to FC on {SERIAL_PORT}...")

    # The 'with' statement ensures the connection is properly closed
    # after the block is exited, even if errors occur.
    try:
        with MSPy(device=SERIAL_PORT, loglevel="WARNING", baudrate=115200) as board:
            if board is None:
                print("Failed to connect to the flight controller.")
                return

            print("Successfully connected to the flight controller.")
            print("Requesting barometer data... Press Ctrl+C to stop.")

            while True:
                # 1. Choose the command to send
                # For altitude data from the barometer, we use 'MSP_ALTITUDE'
                msp_command = "MSP_ALTITUDE"

                # 2. Send the command to the flight controller
                # The 'send_RAW_msg' method sends the command and waits for a response.
                # It returns the data if successful, otherwise it might return None or raise an exception.
                if board.send_RAW_msg(MSPy.MSPCodes[msp_command], data=[]):

                    # 3. Get the data from the board's attribute
                    # The YAMSPy library stores the received data in an attribute
                    # with the same name as the command (in lowercase).
                    data_handler = board.receive_msg()
                    board.process_recv_data(data_handler)
                    altitude_data = board.SENSOR_DATA["altitude"]

                    # 4. Process and display the data
                    # The 'alt' key contains the altitude in centimeters.
                    # The 'vario' key contains the vertical speed in cm/s.
                    if altitude_data:
                        return f"Altitude: {altitude_data:.2f} meters"
                    else:
                        return "No altitude data received in this cycle."

                else:
                    return f"Failed to send {msp_command} command."

                # Wait for a short period before sending the next request
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping data requests. Exiting.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")


def arm():
    try:
        with MSPy(device=SERIAL_PORT, loglevel="WARNING", baudrate=115200) as board:
            if board is None:
                print("Failed to connect to the flight controller.")
                return

            print("Successfully connected to the flight controller.")
            print("Arming... Press Ctrl+C to stop.")

            while True:
                try:
                    rc_channels["aux1"] = 1800
                    rc_channels["aux3"] = 1800
                    board.fast_msp_rc_cmd(
                        [rc_channels[channel] for channel in rc_channel_order]
                    )
                except Exception as e:
                    print(f"Error sending RC data: {e}")
                    break

    except KeyboardInterrupt:
        print("\nStopping data requests. Exiting.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")


def disarm():
    try:
        with MSPy(device=SERIAL_PORT, loglevel="WARNING", baudrate=115200) as board:
            if board is None:
                print("Failed to connect to the flight controller.")
                return

            print("Successfully connected to the flight controller.")
            print("Disarming... Press Ctrl+C to stop.")

            while True:
                try:
                    rc_channels["aux3"] = 1000
                    rc_channels["aux1"] = 1000
                    board.fast_msp_rc_cmd(
                        [rc_channels[channel] for channel in rc_channel_order]
                    )
                    # It's crucial to send messages continuously to prevent FC failsafe
                    time.sleep(0.05)
                except Exception as e:
                    print(f"Error sending RC data: {e}")
                    break

    except KeyboardInterrupt:
        print("\nStopping data requests. Exiting.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")


def handle_client_connection(client_socket, client_address):
    """
    Manages the communication with a single connected client.
    """
    logging.info(f"Accepted connection from {client_address}")
    try:
        while True:
            # Wait to receive data from the client (up to 1024 bytes)
            data = client_socket.recv(1024)
            if not data:
                # If no data is received, the client has closed the connection
                logging.info(f"Client {client_address} disconnected.")
                break

            # Decode the received bytes into a string and remove whitespace
            command = data.decode("utf-8").strip().lower()
            logging.info(f"Received command: '{command}' from {client_address}")

            response = ""
            # --- Process the command ---
            if command == "ping":
                response = "pong"
            elif command == "status":
                response = "Server is running and ready for commands."
            elif command == "bar":
                response = get_barometer_reading()
            elif command == "arm":
                arm()
            elif command == "disarm":
                disarm()
            elif command == "time":
                now = datetime.now()
                response = f"Server time is: {now.strftime('%Y-%m-%d %H:%M:%S')}"
            else:
                response = "Error: Unknown command."

            # Encode the response string into bytes and send it back to the client
            client_socket.sendall(response.encode("utf-8"))

    except ConnectionResetError:
        logging.warning(
            f"Connection with {client_address} was forcibly closed by the client."
        )
    except Exception as e:
        logging.error(f"An error occurred with client {client_address}: {e}")
    finally:
        # Ensure the connection is closed when the loop is exited
        logging.info(f"Closing connection with {client_address}")
        client_socket.close()


def main():
    """
    The main function to set up the server and listen for connections.
    """
    # Create a new socket object for TCP/IP communication
    # AF_INET specifies IPv4, SOCK_STREAM specifies TCP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # This option allows the socket to be reused immediately after it's closed
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        # Bind the socket to the host IP and port
        server_socket.bind((HOST_IP, COMMAND_PORT))

        # Enable the server to accept connections, with a queue of up to 5
        server_socket.listen(5)
        logging.info(f"Server is listening on port {COMMAND_PORT}...")

        # Main loop to continuously accept new connections
        while True:
            # Wait for a client to connect. This is a blocking call.
            # It returns a new socket for the client and the client's address.
            client_socket, client_address = server_socket.accept()

            # For simplicity, this server handles one client at a time.
            # For multiple simultaneous clients, you would typically start a new thread here.
            # For example:
            # import threading
            # client_thread = threading.Thread(target=handle_client_connection, args=(client_socket, client_address))
            # client_thread.start()

            # Handle the client connection directly in the main loop
            handle_client_connection(client_socket, client_address)

    except socket.error as e:
        logging.error(f"Socket error: {e}")
    except KeyboardInterrupt:
        logging.info("Server is shutting down due to user interrupt (Ctrl+C).")
    finally:
        # Ensure the server socket is closed when the program ends
        logging.info("Closing server socket.")
        server_socket.close()


if __name__ == "__main__":
    main()
