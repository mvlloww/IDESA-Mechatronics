import socket
import struct

class UDPComm:
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip = ip
        self.port = port

    def send(self, dy, dx, theta):
        msg = struct.pack('<iif', int(dy), int(dx), float(theta))
        self.sock.sendto(msg, (self.ip, self.port))
