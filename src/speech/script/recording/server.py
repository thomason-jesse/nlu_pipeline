import socket

s = socket.socket()
host = ''
port = 5000
s.bind((host, port))

s.listen(1)

while True:
    c, addr = s.accept()
    print "got connection from " + str(addr)
    c.send("Thank you for connecting")
    c.close()
