import time
import json
import socket
import logging
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
from db.repositories import get_sensors_repository, get_actuators_repository
from messages_pb2 import Address, JoinRequest, JoinReply, DeviceType


def multicast_location(
    stop_flag,
    host_ip,
    multicast_ip,
    multicast_port,
    multicast_interval,
    registration_port,
):
    logger = logging.getLogger('MULTICASTER')
    logger.info(
        'Enviando endereço de registro para grupo multicast (%s, %s)',
        multicast_ip,
        multicast_port,
    )
    addrs = Address(ip=host_ip, port=registration_port)
    addrs = addrs.SerializeToString()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        while not stop_flag.is_set():
            try:
                sock.sendto(addrs, (multicast_ip, multicast_port))
            except Exception as e:
                logger.error(
                    'Erro ao enviar mensagem para o grupo multicast: (%s) %s',
                    type(e).__name__,
                    e,
                )
                raise e
            time.sleep(multicast_interval)


def registration_handler(
    sensors_port,
    sensors_tolerance,
    actuators_port,
    actuators_tolerance,
    sock,
    address,
):
    try:
        logger = logging.getLogger(f'REGISTRATION_HANDLER_{address}')
        logger.info('Processando requisição de registro')
        msg = sock.recv(1024)
        request = JoinRequest()
        request.ParseFromString(msg)
        device_info = request.device_info
        device_addrs = request.device_address
        device_category, device_id = device_info.name.split('-')
        device_id = int(device_id)
        match request.device_info.type:
            case DeviceType.DT_SENSOR:
                sensors_repository = get_sensors_repository()
                report_port = sensors_port
                metadata = json.loads(device_info.metadata)
                sensors_repository.add_sensor(
                    sensor_id=device_id,
                    sensor_category=device_category,
                    ip_address=device_addrs.ip,
                    device_metadata=metadata,
                    availability_tolerance=sensors_tolerance,
                )
            case DeviceType.DT_ACTUATOR:
                actuators_repository = get_actuators_repository()
                report_port = actuators_port
                state = json.loads(device_info.state)
                metadata = json.loads(device_info.metadata)
                timestamp = datetime.fromisoformat(device_info.timestamp)
                actuators_repository.add_actuator(
                    actuator_id=device_id,
                    actuator_category=device_category,
                    ip_address=device_addrs.ip,
                    communication_port=device_addrs.port,
                    device_state=state,
                    device_metadata=metadata,
                    timestamp=timestamp,
                    availability_tolerance=actuators_tolerance,
                )
            case _:
                raise ValueError('Invalid DeviceType')
        reply = JoinReply(report_port=report_port)
        sock.send(reply.SerializeToString())
        logger.info('Ingresso bem-sucedido: %s', device_info.name)
    except Exception as e:
        logger.error(
            'Erro durante registro do dispositivo %s: (%s) %s',
            device_info.name,
            type(e).__name__,
            e,
        )
        raise e
    finally:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            logger.error(
                'Erro ao tentar enviar FIN para %s',
                address,
            )
        finally:
            sock.close()


def registration_listener(
    stop_flag,
    registration_port,
    sensors_port,
    sensors_tolerance,
    actuators_port,
    actuators_tolerance,
):
    logger = logging.getLogger('REGISTRATION_LISTENER')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', registration_port))
        sock.listen()
        logger.info(
            'Escutando por requisições de registro na porta %d',
            registration_port,
        )
        sock.settimeout(1.0)
        with ThreadPoolExecutor(max_workers=5) as executor:
            while not stop_flag.is_set():
                try:
                    conn, addrs = sock.accept()
                except TimeoutError:
                    continue
                try:
                    conn.settimeout(sock.gettimeout())
                    executor.submit(
                        registration_handler,
                        sensors_port,
                        sensors_tolerance,
                        actuators_port,
                        actuators_tolerance,
                        conn,
                        addrs,
                    )
                except Exception:
                    try:
                        conn.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        logger.error(
                            'Erro ao tentar enviar FIN para %s',
                            addrs,
                        )
                    finally:
                        conn.close()
                    raise
