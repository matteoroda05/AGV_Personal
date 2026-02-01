import socket

HOST = '127.0.0.1' # Localhost
PORT = 9090 # Port of the C++ server (to be created)

try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: 
        # Choosing INTERNET (AF_INET) and TCP (SOCK_STREAM) connection–seemed the most appropriate from video proposed
        s.connect((HOST, PORT))
        print("Connected to C++ server.\n")
        
        messaggio = "Hello World! \n From Python Client."
        s.sendall(messaggio.encode()) # Send message encoded with default ?utf-8?
        print("Messaggio inviato.\n")
        
except ConnectionRefusedError:
    print("Errore: Il server C++ non è attivo o la porta è sbagliata.\n")