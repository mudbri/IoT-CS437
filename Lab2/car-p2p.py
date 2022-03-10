import sys
import bluetooth
import threading
import time
import os
import subprocess

def start_server():
    hostMACAddress = "E4:5F:01:77:8B:EF" 
    port = 0
    backlog = 1
    size = 1024
    s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    s.bind((hostMACAddress, port))
    s.listen(backlog)
    print("listening on port ", port)
    try:
        client, clientInfo = s.accept()
        while 1:
            print("server recv from: ", clientInfo)
            data = client.recv(size)
            if data != b"":
                print(data) 
                if (data == b'stop'):
                    print("car will stop")  
                    client.send("Car stopped")
                    fc.stop()                    
                elif (data == b'up'):
                    print("car will move forward")  
                    client.send("Car moved forward")
                    fc.forward(10)
                elif (data == b'right'):
                    print("car will move right")  
                    client.send("Car turned towards the right")
                    fc.turn_right(10)
                elif (data == b'left'):
                    print("car will move left") 
                    client.send("Car turned towards the left")
                    fc.turn_left(10) 
                elif (data == b'down'):
                    print("car will move back")  
                    client.send("Car moved backward")
                    fc.backward(10)   
                else:
                    print("Incorrect command given")  
                    print(data)  
                    client.send("Incorrect command given")

    except: 
        print("Closing socket")
        client.close()
        s.close()

def start_client():
    host = "58:FB:84:4B:8E:98" 
    port = 1
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    time.sleep(5)
    sock.connect((host, port))
    while 1:
        time.sleep(5)
        temp = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True)
        sock.send("The temperature is " + temp.stdout.decode("utf-8"))
    sock.close()

sth = threading.Thread(target=start_server)
cth = threading.Thread(target=start_client)

sth.start()
cth.start()

cth.join()
sth.join()

#print("Success, terminating")
