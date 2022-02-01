"""
Temporary server for testing sockets and JSON format.
"""
import socket
import json

HOST = '0.0.0.0'
PORT = 17021

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    print(f"Listening on {HOST}:{PORT}")
    s.listen()
    s.settimeout(0.5)
    try:
        while True:
            try:
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
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        print("Exiting...")
