import socket
import time
import datetime
from mensagem_pb2 
import DispositivoInfo, LeituraSensor

ID = "sensor_temp1"
FALHA = False  # Simular falha

def announce():
    info = DispositivoInfo(tipo="sensor", id=ID, ip="127.0.0.1", porta=6001, estado="ativo")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    sock.sendto(info.SerializeToString(), ("224.0.0.1", 5000))

def enviar_leitura():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        if FALHA:
            print("[sensor_temp1] FALHA: parando envio.")
            break
        leitura = LeituraSensor(
            id=ID,
            tipo="temperatura",
            valor="24.7",
            timestamp=str(datetime.datetime.now())
        )
        s.sendto(leitura.SerializeToString(), ("127.0.0.1", 7000))
        time.sleep(15)

announce()
enviar_leitura()
