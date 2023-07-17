# Get MAC of server
# from uuid import getnode as get_mac
# mac = get_mac()
# mac = '%012x' % mac
# print(mac)

# Server
# import socket
# hostMACAddress = '08:5b:d6:64:f2:2f'
# port = 4
# backlog = 1
# size = 1024
# s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
# s.bind((hostMACAddress,port))
# s.listen(backlog)
# try:
#     client, address = s.accept()
#     while 1:
#         data = client.recv(size)
#         if data:
#             print(data.decode('ascii'))
#             client.send(bytes("yash","UTF-8"))
# except:	
#     print("Closing socket")	
#     client.close()
#     s.close()

# Client
import socket
import json
import time

serverMACAddress = 'd8:c0:a6:9b:9b:3e'
port = 4
s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.connect((serverMACAddress,port))
while 1:
    text = {"x": 1.2, "y": 3.4}
    text = json.dumps(text)
    print(text)
    if text == "quit":
        break
    a = s.send(bytes(text, 'UTF-8'))
    a = s.recv(1024)
    print(a)
    time.sleep(1)
s.close()
