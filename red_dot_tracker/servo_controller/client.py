# servo_controller/client.py

import socket
import json
import time

class ServoClient:
    def __init__(self, host="10.162.1.106", port=9999):
        self.host = host
        self.port = port
        # self.retry_delay = retry_delay # Retry delay not typically used for UDP
        # Create a UDP socket at initialization
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, axis, degrees):
        # Add timestamp to the payload
        payload_dict = {
            "axis": axis, 
            "degrees": degrees,
            "timestamp": time.time() # Current time in seconds since epoch
        }
        payload = json.dumps(payload_dict)
        try:
            # For UDP, we use sendto and provide the destination address each time
            self.sock.sendto(payload.encode(), (self.host, self.port))
            # Reduced print frequency, or make it conditional for performance
            # print(f"[SEND_UDP] {axis.upper()} -> {degrees} to {self.host}:{self.port}")
        except socket.error as e:
            # Handle socket errors, e.g., if the network is unreachable
            # For basic UDP, often we just print an error and move on, as delivery is not guaranteed
            print(f"[UDP_ERROR] Failed to send packet: {e}")
        # Removed the while True loop and retry logic for ConnectionRefusedError etc.
        # as UDP is connectionless. If a packet is lost, it's lost.

    def close(self): # Add a close method for completeness
        """Close the client socket."""
        print("[CLIENT_SOCKET] Closing UDP socket.")
        self.sock.close()

# Example usage (optional, for testing client directly):
if __name__ == '__main__':
    client = ServoClient() # Uses default host/port
    try:
        client.send("pitch", 45.5)
        time.sleep(1)
        client.send("yaw", -30.0)
        time.sleep(1)
        client.send("stop", 0) # Degrees value is ignored for stop, but required by send method signature
    finally:
        client.close() # Ensure socket is closed
