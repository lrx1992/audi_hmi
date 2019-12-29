import socket
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while 1:
    data = input('input:')
    data = "data:"+data
    s.sendto(data.encode(),("192.168.1.66",9090))
    s.sendto(data.encode(),("192.168.1.65",9090))
