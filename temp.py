import socket
import pynmeagps
import json
import base64

stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
stream.connect(("localhost", 2121))

for raw_data, parsed_data in pynmeagps.NMEAReader(stream).iterate():
    print(json.dumps({
        "nmea": base64.b64encode(raw_data).decode('ascii')
    }))