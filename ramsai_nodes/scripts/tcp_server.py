import socket

HOST = 'localhost'
PORT = 7778
with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)

    print(f"[TCP] En attente de connexion sur le {PORT}...")
    conn, addr = s.accept()

    print(f"[TCP] Connecté à {addr}")

