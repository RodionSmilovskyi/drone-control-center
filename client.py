# simple_laptop_client.py
# To be run on the laptop
# This client sends commands to the server and prints the response.

import socket
import logging


# --- Configuration ---
COMMAND_PORT = 65000  # The port must match the server's port
SERIAL_PORT = "/dev/ttyACM0" 

# --- Setup logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


def main():
    """
    Main function to connect to the server and send commands.
    """
    # Get the server's IP address from the user
    # pi_ip_address = input("Enter the Raspberry Pi's IP address: ").strip()
    pi_ip_address = '10.0.0.131'
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
            command = input("\nEnter command (ping, status, time, bar, arm, disarm, or 'exit' to quit): ").strip().lower()

            if not command:
                continue
            
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
