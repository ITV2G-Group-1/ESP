import socket
import json

HOST = '0.0.0.0'  # Standard loopback interface address (localhost)
PORT = 17021        # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    while True:
        s.listen()
        conn, addr = s.accept()
        with conn:
            print('[+] Connected by', addr)
            data = b""
            while True:
                new = conn.recv(1024)
                if not new:
                    break
                
                data += new
        
        print(json.loads(data))
        print("[-] Connection closed")
