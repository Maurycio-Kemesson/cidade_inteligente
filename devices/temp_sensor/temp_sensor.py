import sys
import json
import socket
import time
import datetime
import random
import threading
import logging
from functools import wraps
import pika
from google.protobuf.message import DecodeError
from messages_pb2 import Address, SensorReading
from messages_pb2 import DeviceType, DeviceInfo, JoinRequest, JoinReply


def discoverer(args):
    logger = logging.getLogger('DISCOVERER')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(args.multicast_timeout)
        sock.bind(('', args.multicast_port))
        sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(args.multicast_ip) + socket.inet_aton('0.0.0.0'),
        )
        logger.info(
            'Escutando no grupo multicast (%s, %s)',
            args.multicast_ip,
            args.multicast_port,
        )
        seq_fails = 0
        while not args.stop_flag.is_set():
            try:
                msg = sock.recv(1024)
            except TimeoutError:
                seq_fails += 1
                if (
                    args.gateway_ip is not None
                    and seq_fails >= args.disconnect_after
                ):
                    logger.warning(
                        'Gateway em %s está offline. Desconectando...',
                        args.gateway_ip,
                    )
                    args.gateway_ip = None
                    disconnect_broker(args)
                continue
            addresses = Address()
            try:
                addresses.ParseFromString(msg)
            except DecodeError:
                continue
            seq_fails = 0
            if addresses.ip != args.gateway_ip:
                result = try_registration(args, addresses.ip, addresses.port)
                if result:
                    logger.info(
                        'Registro bem-sucedido com Gateway em %s',
                        args.gateway_ip,
                    )
                else:
                    logger.warning(
                        'Falha durante registro em (%s, %d)',
                        addresses.ip,
                        addresses.port,
                    )
                    disconnect_broker(args)
                    continue
            if (
                addresses.broker_ip != args.message_broker_ip
                or addresses.broker_port != args.message_broker_port
                or addresses.publish_exchange != args.publish_exchange
            ):
                logger.info(
                    'Novo Broker encontrado: (%s:%d, %s). Reconectando...',
                    addresses.broker_ip,
                    addresses.broker_port,
                    addresses.publish_exchange,
                )
                disconnect_broker(args)
                register_broker(
                    args,
                    addresses.broker_ip,
                    addresses.broker_port,
                    addresses.publish_exchange,
                )


def disconnect_broker(args):
    with args.broker_lock:
        args.message_broker_ip = None
        args.message_broker_port = None
        args.publish_exchange = None
    args.disconnect_flag.set()


def register_broker(args, broker_ip, broker_port, publish_exchange):
    with args.broker_lock:
        args.message_broker_ip = broker_ip
        args.message_broker_port = broker_port
        args.publish_exchange = publish_exchange


def try_registration(args, gateway_ip, registration_port):
    sensor_info = DeviceInfo(
        type=DeviceType.DT_SENSOR,
        name=args.name,
        metadata=json.dumps(args.metadata),
    )
    sensor_address = Address(ip=args.host_ip)
    join_request = JoinRequest(
        device_info=sensor_info,
        device_address=sensor_address,
    )
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.settimeout(args.base_timeout)
            sock.connect((gateway_ip, registration_port))
            sock.send(join_request.SerializeToString())
            join_reply = JoinReply()
            join_reply.ParseFromString(sock.recv(1024))
        except Exception:
            args.gateway_ip = None
            return False
    args.gateway_ip = gateway_ip
    return True


def get_reading(args):
    temp = args.temperature + random.random() - 0.5
    temp = min(max(temp, args.min_temperature), args.max_temperature)
    args.temperature = temp
    return temp


def readings_publisher(args):
    logger = logging.getLogger('READINGS_PUBLISHER')
    while not args.stop_flag.is_set():
        args.disconnect_flag.clear()
        with args.broker_lock:
            broker_ip = args.message_broker_ip
            broker_port = args.message_broker_port
        if broker_ip is None:
            logger.info('Sem conexão com o Broker')
            time.sleep(1.0)
            continue
        connection = pika.BlockingConnection(
            pika.ConnectionParameters(
                host=broker_ip,
                port=broker_port,
                socket_timeout=args.base_timeout,
            )
        )
        channel = connection.channel()
        try:
            logger.info(
                'Conexão bem-sucedida com Broker em (%s, %d)',
                broker_ip,
                broker_port,
            )
            channel.exchange_declare(
                exchange=args.publish_exchange,
                exchange_type='fanout',
            )
            while (
                not args.stop_flag.is_set()
                and not args.disconnect_flag.is_set()
            ):
                reading = SensorReading(
                    device_name=args.name,
                    reading_value=get_reading(args),
                    timestamp=datetime.datetime.now(datetime.UTC).isoformat(),
                )
                channel.basic_publish(
                    exchange=args.publish_exchange,
                    routing_key='',
                    body=reading.SerializeToString(),
                )
                time.sleep(args.report_interval)
        finally:
            logger.info(
                'Terminando conexão com o Broker em (%s, %d)',
                broker_ip,
                broker_port,
            )
            channel.close()
            connection.close()


def stop_wrapper(func, stop_flag):
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        finally:
            stop_flag.set()
    return wrapper


def _run(args):
    try:
        publisher = threading.Thread(
            target=stop_wrapper(readings_publisher, args.stop_flag),
            args=(args,)
        )
        publisher.start()
        discoverer(args)
    except KeyboardInterrupt:
        print('\nSHUTTING DOWN...')
    finally:
        args.stop_flag.set()
        publisher.join()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Sensor de temperatura.')

    parser.add_argument(
        '--id', type=int, default=1,
        help='Id que unicamente identifica o sensor de temperatura.'
    )

    parser.add_argument(
        '--multicast_ip', type=str, default='224.0.1.0',
        help='IP multicast para descobrimento do Gateway.'
    )

    parser.add_argument(
        '--multicast_port', type=int, default=50333,
        help='Porta na qual escutar por mensagens do grupo multicast.'
    )

    parser.add_argument(
        '--report_interval', type=float, default=5.0,
        help='Intervalo entre o envio de leituras.'
    )

    parser.add_argument(
        '--temperature', type=float, default=25.0,
        help='Temperatura inicial do sensor em °C.'
    )

    parser.add_argument(
        '--max_temperature', type=float, default=40.0,
        help='Temperatura máximo do sensor em °C.'
    )

    parser.add_argument(
        '--min_temperature', type=float, default=20.0,
        help='Temperatura mínima do sensor em °C.'
    )

    parser.add_argument(
        '--disconnect_after', type=int, default=3,
        help='Número de falhas sequenciais necessárias para desconectar o dispositivo.'
    )

    parser.add_argument(
        '-l', '--level', type=str, default='INFO',
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR'],
        help='Nível do logging.'
    )

    args = parser.parse_args()

    # Logging
    logging.basicConfig(
        level=args.level,
        handlers=(logging.StreamHandler(sys.stdout),),
        format='[%(levelname)s %(asctime)s] %(name)s\n  %(message)s',
    )

    # Device name
    args.name = f'temperature-{args.id}'

    # Timeouts
    args.base_timeout = 2.0
    args.multicast_timeout = 5.0

    # Host IP
    args.host_ip = socket.gethostbyname('localhost')

    # Gateway IP
    args.gateway_ip = None

    # Broker
    args.broker_lock = threading.Lock()
    args.disconnect_flag = threading.Event()
    args.message_broker_ip = None
    args.message_broker_port = None
    args.publish_exchange = None

    # Metadata
    args.metadata = {
        'UnitName': 'Celsius',
        'UnitSymbol': '°C',
        'Location': {'Latitude': -3.733486, 'Longitude': -38.570860},
    }

    # Stop flag for clean termination
    args.stop_flag = threading.Event()

    return _run(args)


if __name__ == '__main__':
    main()
