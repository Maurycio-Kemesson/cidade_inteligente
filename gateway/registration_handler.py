import time
import json
import socket
import logging
import threading
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
from messages_pb2 import Address, JoinRequest, JoinReply, DeviceType


def multicast_location(args):
    logger = logging.getLogger('MULTICASTER')
    logger.info(
        'Multicasting endereço de registro de dispositivos: (%s, %s)',
        args.host_ip, args.registration_port
    )
    addrs = Address(ip=args.host_ip, port=args.registration_port)
    addrs = addrs.SerializeToString()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        try:
            while not args.stop_flag.is_set():
                sock.sendto(addrs, (args.multicast_ip, args.multicast_port))
                time.sleep(args.multicast_interval)
        except Exception as e:
            logger.error('Erro durante multicast: (%s) %s', type(e).__name__, e)


def registration_handler(args, sock, addrs):
    logger = logging.getLogger(f'REGISTRATION_HANDLER_{threading.get_ident()}')
    logger.info(
        'Processando requisição de ingresso de %s', addrs
    )
    try:
        sock.settimeout(args.base_timeout)
        req = JoinRequest()
        req.ParseFromString(sock.recv(1024))
        if req.device_info.type is DeviceType.DT_SENSOR:
            report_port = args.sensors_port
            with args.db_sensors_lock:
                args.db.register_sensor(
                    name=req.device_info.name,
                    address=(req.device_address.ip, req.device_address.port),
                    metadata=json.loads(req.device_info.metadata),
                )
        elif req.device_info.type is DeviceType.DT_ACTUATOR:
            report_port = args.actuators_port
            with args.db_actuators_lock:
                args.db.register_actuator(
                    name=req.device_info.name,
                    address=(req.device_address.ip, req.device_address.port),
                    state=json.loads(req.device_info.state),
                    metadata=json.loads(req.device_info.metadata),
                    timestamp=datetime.fromisoformat(req.device_info.timestamp),
                )
        else:
            raise RuntimeError('Invalid device type')
        reply = JoinReply(report_port=report_port)
        sock.send(reply.SerializeToString())
        logger.info(
            'Ingresso bem-sucedido do dispositivo %s em %s',
            req.device_info.name, addrs
        )
    except Exception as e:
        logger.error(
            'Erro no processamento da requisição de %s: (%s) %s',
            addrs, type(e).__name__, e
        )
    finally:
        sock.shutdown(socket.SHUT_RDWR)
        sock.close()


def registration_listener(args):
    logger = logging.getLogger('REGISTRATION_LISTENER')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', args.registration_port))
        sock.listen()
        logger.info(
            'Escutando requisições de registro em (%s, %s)',
            args.host_ip, args.registration_port
        )
        sock.settimeout(args.base_timeout)
        with ThreadPoolExecutor(max_workers=10) as executor:
            while not args.stop_flag.is_set():
                try:
                    conn, addrs = sock.accept()
                except TimeoutError:
                    continue
                executor.submit(registration_handler, args, conn, addrs)
