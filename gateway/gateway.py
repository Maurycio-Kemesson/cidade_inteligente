import socket
import threading
import time
from mensagem_pb2 import DispositivoInfo, Comando, ListaDispositivos

dispositivos = {}

def discovery_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", 5000))
    mreq = socket.inet_aton("224.0.0.1") + socket.inet_aton("0.0.0.0")
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    while True:
        data, addr = sock.recvfrom(1024)
        info = DispositivoInfo()
        info.ParseFromString(data)
        dispositivos[info.id] = (info, addr, time.time())
        print(f"[+] Dispositivo descoberto: {info.id} - {info.tipo}")

def monitoramento():
    while True:
        now = time.time()
        for id, (info, addr, last_seen) in list(dispositivos.items()):
            if now - last_seen > 30:
                print(f"[ALERTA] {id} inativo. Removendo...")
                dispositivos.pop(id)
        time.sleep(10)

def handle_client(conn):
    while True:
        data = conn.recv(1024)
        if not data:
            break
        cmd = Comando()
        cmd.ParseFromString(data)
        if cmd.id == "LISTAR":
            resposta = ListaDispositivos()
            for d in dispositivos.values():
                resposta.dispositivos.append(d[0])
            conn.send(resposta.SerializeToString())
        elif cmd.id in dispositivos:
            _, (ip, _) , _ = dispositivos[cmd.id]
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((ip, 6000))
                s.send(data)
                resposta = s.recv(1024)  # Recebe a resposta do dispositivo
                conn.send(resposta)  # Encaminha a resposta ao cliente Flutter
                dispositivos[cmd.id] = (dispositivos[cmd.id][0], dispositivos[cmd.id][1], time.time())

def client_listener():
    server = socket.socket()
    server.bind(("0.0.0.0", 9000))
    server.listen()
    while True:
        conn, _ = server.accept()
        threading.Thread(target=handle_client, args=(conn,), daemon=True).start()

threading.Thread(target=discovery_listener, daemon=True).start()
threading.Thread(target=client_listener, daemon=True).start()
threading.Thread(target=monitoramento, daemon=True).start()

print("[GATEWAY] Online.")
while True:
    time.sleep(10)
