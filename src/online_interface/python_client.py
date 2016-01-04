import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

host = '128.83.120.240'
port = 9999
s.connect((host, port))                               

while True :
    try :
        m = s.recv(1024)
        print 'Received ', m
        print 'Reply :',
        v = raw_input()
        if v == 'stop' :
            break
        s.send(v)    
    except socket.error, v:
        errorcode=v[0]
        print 'Error: ', v
        

s.close()
