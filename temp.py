import socket
import pynmeagps

stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
stream.connect(("localhost", 2121))

for raw_data, parsed_data in pynmeagps.NMEAReader(stream).iterate():
    print(parsed_data)