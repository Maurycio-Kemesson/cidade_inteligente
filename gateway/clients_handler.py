import time
import json
import socket
import logging
import datetime
import threading
from struct import pack
from concurrent.futures import ThreadPoolExecutor
from messages_pb2 import ConnectRequest
from messages_pb2 import RequestType, ClientRequest
from messages_pb2 import ReplyStatus, ClientReply
from messages_pb2 import SensorReading, SensorData
from messages_pb2 import ActuatorUpdate, SendNextReport


def transmit_sensors_reports(args, address, stop_transmission_flag):
    logger = logging.getLogger(f'TRANSMIT_SENSORS_REPORTS_{address}')
    logger.info(
        'Preparando envio de relatórios para o cliente em %s', address
    )
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.settimeout(args.reports_timeout)
        try:
            sock.connect(address)
        except Exception as e:
            stop_transmission_flag.set()
            logger.error(
                'Erro ao estabelecer conexão para '
                'envio de relatórios: (%s) %s',
                type(e).__name__,
                e,
            )
            raise e
        last_report_sent = 0
        while not stop_transmission_flag.is_set():
            with args.db_sensors_report_lock:
                report_number, report = args.db.sensors_report
            if report_number <= last_report_sent:
                time.sleep(1.0)
                continue
            try:
                sock.sendall(report)
            except Exception as e:
                stop_transmission_flag.set()
                logger.error(
                    'Erro ao tentar enviar relatório #%d', report_number
                )
                raise e
            fail_count = 0
            while True:
                try:
                    msg = sock.recv(1024)
                    break
                except Exception as e:
                    if isinstance(e, TimeoutError):
                        fail_count += 1
                        if fail_count < 3:
                            continue
                    stop_transmission_flag.set()
                    logger.error(
                        'Erro no recebimento da confirmação '
                        'de envio do relatório #%d',
                        report_number,
                    )
                    raise e
            confirmation = SendNextReport()
            confirmation.ParseFromString(msg)
            last_report_sent = report_number


def transmit_actuators_reports(args, address, stop_transmission_flag):
    logger = logging.getLogger(f'TRANSMIT_ACTUATORS_REPORTS_{address}')
    logger.info(
        'Preparando envio de relatórios para o cliente em %s', address
    )
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.settimeout(args.reports_timeout)
        try:
            sock.connect(address)
        except Exception as e:
            stop_transmission_flag.set()
            logger.error(
                'Erro ao estabelecer conexão para '
                'envio de relatórios: (%s) %s',
                type(e).__name__,
                e,
            )
            raise e
        last_report_sent = 0
        while not stop_transmission_flag.is_set():
            with args.db_actuators_report_lock:
                report_number, report = args.db.actuators_report
            if report_number <= last_report_sent:
                time.sleep(1.0)
                continue
            try:
                sock.sendall(report)
            except Exception as e:
                stop_transmission_flag.set()
                logger.error(
                    'Erro ao tentar enviar relatório #%d', report_number
                )
                raise e
            fail_count = 0
            while True:
                try:
                    msg = sock.recv(1024)
                    break
                except Exception as e:
                    if isinstance(e, TimeoutError):
                        fail_count += 1
                        if fail_count < 3:
                            continue
                    stop_transmission_flag.set()
                    logger.error(
                        'Erro no recebimento da confirmação '
                        'de envio do relatório #%d',
                        report_number,
                    )
                    raise e
            confirmation = SendNextReport()
            confirmation.ParseFromString(msg)
            last_report_sent = report_number


def init_transmissions(
    args,
    client_ip,
    sensors_port,
    actuators_port,
    stop_transmission_flag
):
    sensors_thread = threading.Thread(
        target=transmit_sensors_reports,
        args=(args, (client_ip, sensors_port), stop_transmission_flag)
    )
    actuators_thread = threading.Thread(
        target=transmit_actuators_reports,
        args=(args, (client_ip, actuators_port), stop_transmission_flag)
    )
    sensors_thread.start()
    actuators_thread.start()
    return sensors_thread, actuators_thread


def process_client_request(request):
    reply = ClientReply()
    return reply


def client_handler(args, sock, address):
    logger = logging.getLogger(f'CLIENT_HANDLER_{address}')
    logger.info('Gerenciando conexão com o cliente em %s', address)
    trans_stop_flag = threading.Event()
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.settimeout(args.client_timeout)
        try:
            conn_req = ConnectRequest()
            conn_req.ParseFromString(sock.recv(1024))
        except Exception as e:
            logger.error(
                'Não foi possível estabelecer conexão com o cliente: (%s) %s',
                type(e).__name__,
                e,
            )
            raise e
        sensors_trans, actuators_trans = init_transmissions(
            args,
            address[0],
            conn_req.sensors_report_port,
            conn_req.actuators_report_port,
            trans_stop_flag
        )
        while not (args.stop_flag.is_set() or trans_stop_flag.is_set()):
            try:
                request = ClientRequest()
                request.ParseFromString(sock.recv(1024))
            except TimeoutError:
                continue
            except Exception as e:
                logger.error(
                    'Erro durante o recebimento da '
                    'requisição do cliente: (%s) %s',
                    type(e).__name__,
                    e,
                )
                raise e
            try:
                reply = process_client_request(request)
            except Exception as e:
                logger.error(
                    'Erro durante o processamento da '
                    'requisição do cliente: (%s) %s',
                    type(e).__name__,
                    e,
                )
                raise e
            try:
                sock.sendall(reply.SerializeToString())
            except Exception as e:
                logger.error(
                    'Erro ao enviar resposta ao cliente: (%s) %s',
                    type(e).__name__,
                    e,
                )
                raise e
    finally:
        trans_stop_flag.set()
        sensors_trans.join()
        actuators_trans.join()
        sock.shutdown(socket.SHUT_RDWR)
        sock.close()


def clients_listener(args):
    logger = logging.getLogger('CLIENTS_LISTENER')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', args.clients_port))
        sock.listen()
        logger.info(
            'Escutando pedidos de conexão dos clientes em (%s, %s)',
            args.host_ip, args.clients_port
        )
        sock.settimeout(args.base_timeout)
        with ThreadPoolExecutor(max_workers=10) as executor:
            while not args.stop_flag.is_set():
                try:
                    conn, addrs = sock.accept()
                except TimeoutError:
                    continue
                except Exception as e:
                    logger.error(
                        'Erro ao tentar conexão com um novo cliente: (%s) %s',
                        type(e).__name__, e
                    )
                    continue
                executor.submit(client_handler, args, conn, addrs)
