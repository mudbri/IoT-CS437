import sys
import bluetooth
import threading
import time

def start_server():
    hostMACAddress = "58:FB:84:4B:8E:98" 
    port = 0
    backlog = 1
    size = 1024
    s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    s.bind((hostMACAddress, port))
    s.listen(backlog)
    print("listening on port ", port)
    try:
        client, clientInfo = s.accept()
        # print("server recv from: ", clientInfo)
        while 1:   
            data = client.recv(size)
            if data:
                print()
                print("============Temperature reading===========")
                print(data)
                print("==========================================")
                # print("Enter your message: ", end = '') # for readability
            #     client.send(data) # Echo back to client
    except: 
        print("Closing socket")
        client.close()
        s.close()

def start_client():
    host = "E4:5F:01:77:8B:EF" 
    port = 1
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((host, port))
    while 1:
        print()
        text = input("Enter your message: ")
        if text == "quit":
            break
        sock.send(text)

        data = sock.recv(1024)
        print("from car: ", data)

    sock.close()

sth = threading.Thread(target=start_server)
cth = threading.Thread(target=start_client)

sth.start()
cth.start()

cth.join()
sth.join()