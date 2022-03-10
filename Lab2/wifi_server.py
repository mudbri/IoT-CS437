import socket
import picar_4wd as fc
import time
import os
import subprocess

HOST = "192.168.1.23" # IP address of your Raspberry PI
PORT = 65432          # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()

    try:
        
        while 1:
            
            client, clientInfo = s.accept()
            print("server recv from: ", clientInfo)
            data = client.recv(1024)      # receive 1024 Bytes of message in binary format
            
            if data != b"":
                print(data) 
                if (data == b"stop\r\n"):
                    print("car will stop")  
                    fc.stop()
                    
                if (data == b"up\r\n"):
                    print("car will move forward")  
                    fc.forward(10)
                    
                if (data == b"right\r\n"):
                    print("car will move right")  
                    fc.turn_right(10)
                if (data == b"left\r\n"):
                    print("car will move left") 
                    fc.turn_left(10) 
                if (data == b"down\r\n"):
                    print("car will move back")  
                    fc.backward(10)
                
                temp = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True)
                print(temp)
            
                speed = b"10\r\n"
             
                client.sendall(data + speed + temp.stdout) # Echo back to client
            
                client.sendall(speed)

             
    except: 
        print("Closing socket")
        client.close()
        s.close()    