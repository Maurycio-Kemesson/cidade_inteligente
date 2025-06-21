import socket
import threading
import time
from mensagem_pb2 import DispositivoInfo, Comando, ListaDispositivos, Estado

dispositivos = {}  # id: (DispositivoInfo, addr, last_seen)
estados_dispositivos = {}  # id: estado_atual

# Discovery via UDP multicast
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
        estados_dispositivos[info.id] = info.estado
        print(f"[+] Dispositivo descoberto: {info.id} - {info.tipo}")

# Recebe atualizações de estado dos dispositivos via UDP
def receber_estados_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 7000))  # Porta esperada pelo ESP32
    while True:
        data, _ = sock.recvfrom(1024)
        try:
            estado = Estado()
            estado.ParseFromString(data)
            estados_dispositivos[estado.id] = estado.estado
            print(f"[ESTADO] {estado.id} => {estado.estado}")
        except Exception as e:
            print("[ERRO estado UDP]:", e)

# Remove dispositivos inativos
def monitoramento():
    while True:
        now = time.time()
        for id, (info, addr, last_seen) in list(dispositivos.items()):
            if now - last_seen > 30:
                print(f"[ALERTA] {id} inativo. Removendo...")
                dispositivos.pop(id)
                estados_dispositivos.pop(id, None)
        time.sleep(10)

# Comunicação TCP com app Flutter
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
        elif cmd.acao == "estado":
            estado = Estado(id=cmd.id, estado=estados_dispositivos.get(cmd.id, "desconhecido"))
            conn.send(estado.SerializeToString())
        elif cmd.id in dispositivos:
            info, addr, _ = dispositivos[cmd.id]
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((info.ip, info.porta))
                    s.send(data)
                    dispositivos[cmd.id] = (info, addr, time.time())
            except Exception as e:
                print(f"[ERRO TCP para {cmd.id}] {e}")

def client_listener():
    server = socket.socket()
    server.bind(("0.0.0.0", 9000))
    server.listen()
    while True:
        conn, _ = server.accept()
        threading.Thread(target=handle_client, args=(conn,), daemon=True).start()

# Inicialização
threading.Thread(target=discovery_listener, daemon=True).start()
threading.Thread(target=client_listener, daemon=True).start()
threading.Thread(target=monitoramento, daemon=True).start()
threading.Thread(target=receber_estados_udp, daemon=True).start()

print("[GATEWAY] Online.")
while True:
    time.sleep(10)
