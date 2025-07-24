import sys
import time
import json
import socket
import logging
import threading
from numbers import Real
from datetime import datetime, UTC
from functools import wraps
from concurrent.futures import ThreadPoolExecutor
from google.protobuf.message import DecodeError
from messages_pb2 import Address
from messages_pb2 import DeviceType, DeviceInfo, JoinRequest, JoinReply
from messages_pb2 import ActuatorUpdate
from messages_pb2 import CommandType, ActuatorCommand
from messages_pb2 import ComplyStatus, ActuatorComply


def gateway_discoverer(args):
    logger = logging.getLogger('GATEWAY_DISCOVERER')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', args.multicast_port))
        sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(args.multicast_ip) + socket.inet_aton('0.0.0.0'),
        )
        logger.info(
            'Procurando pelo Gateway no grupo multicast (%s, %s)',
            args.multicast_ip,
            args.multicast_port,
        )
        sock.settimeout(args.multicast_timeout)
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
                        'Gateway em %s está offline: falhou %d '
                        'transmissões em sequência. Desconectando...',
                        args.gateway_ip,
                        args.disconnect_after,
                    )
                    disconnect_device(args)
                continue
            gateway_addrs = Address()
            try:
                gateway_addrs.ParseFromString(msg)
            except DecodeError:
                continue
            seq_fails = 0
            if gateway_addrs.ip == args.gateway_ip:
                continue
            if args.gateway_ip is not None:
                logger.warning(
                    'Gateway realocado de %s para %s. Desconectando...',
                    args.gateway_ip,
                    gateway_addrs.ip,
                )
                disconnect_device(args)
            try_to_register(args, (gateway_addrs.ip, gateway_addrs.port), logger)


def disconnect_device(args):
    with args.connection_lock:
        args.gateway_ip = None
        args.transmission_port = None
    return


def try_to_register(args, address, logger):
    logger.info('Tentando registro no endereço %s', address)
    with args.state_lock:
        state = json.dumps(args.state)
        timestamp = datetime.now(UTC).isoformat()
    actuator_info = DeviceInfo(
        type=DeviceType.DT_ACTUATOR,
        name=args.name,
        state=state,
        metadata=json.dumps(args.metadata),
        timestamp=timestamp,
    )
    actuator_address = Address(ip=args.host_ip, port=args.port)
    join_request = JoinRequest(
        device_info=actuator_info, device_address=actuator_address,
    )
    join_request = join_request.SerializeToString()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(args.base_timeout)
        try:
            sock.connect(address)
            sock.send(join_request)
            join_reply = JoinReply()
            join_reply.ParseFromString(sock.recv(1024))
        except Exception as e:
            logger.warning(
                'Erro durante tentativa de registro em %s: (%s) %s',
                address,
                type(e).__name__,
                e,
            )
            return
    with args.connection_lock:
        args.gateway_ip = address[0]
        args.transmission_port = join_reply.report_port
    logger.info('Registro bem-sucedido com o Gateway em %s', address[0])
    return


def build_update_message(args, state, timestamp):
    return ActuatorUpdate(
        device_name=args.name,
        state=state,
        metadata=json.dumps(args.metadata),
        timestamp=timestamp,
    )


def process_set_state_command(args, state_string):
    new_state = json.loads(state_string)
    unknown_states = set(new_state) - set(args.state)
    if unknown_states:
        return None
    if 'Phase' in new_state:  # 'Phase' is read-only
        return None
    for period in ('GreenPeriod', 'YellowPeriod', 'RedPeriod'):
        try:
            period_value = new_state[period]
        except KeyError:
            continue
        if not isinstance(period_value, Real):
            return None
        period_value = float(period_value)
        new_state[period] = period_value
        if period_value < 5.0:  # Período mínimo de 5 segundos
            return None
    with args.state_lock:
        args.state.update(new_state)
        args.state_change.set()
        state = json.dumps(args.state)
        timestamp = datetime.now(UTC).isoformat()
    return build_update_message(args, state, timestamp)


def process_command(args, command, logger):
    match command.type:
        case CommandType.CT_ACTION:
            logger.debug('Comando do tipo CT_ACTION recebido')
            status = ComplyStatus.CS_UNKNOWN_ACTION
            result = None
        case CommandType.CT_GET_STATE:
            logger.debug('Comando do tipo CT_GET_STATE recebido')
            status = ComplyStatus.CS_OK
            result = None
        case CommandType.CT_SET_STATE:
            logger.debug('Comando do tipo CT_SET_STATE recebido')
            result = process_set_state_command(args, command.body)
            if result is None:
                logger.debug('Comando CT_SET_STATE inválido')
                status = ComplyStatus.CS_INVALID_STATE
            else:
                logger.debug('Comando CT_SET_STATE bem-sucedido')
                status = ComplyStatus.CS_OK
    if result is None:
        with args.state_lock:
            state = json.dumps(args.state)
            timestamp = datetime.now(UTC).isoformat()
        result = build_update_message(args, state, timestamp)
    return ActuatorComply(status=status, update=result).SerializeToString()


def command_handler(args, sock, address):
    try:
        logger = logging.getLogger(f'COMMAND_HANDLER_{address}')
        msg = sock.recv(1024)
        command = ActuatorCommand()
        command.ParseFromString(msg)
        comply = process_command(args, command, logger)
        sock.send(comply)
    except Exception as e:
        logger.error(
            'Erro durante processamento de um comando: (%s) %s',
            type(e).__name__,
            e,
        )
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


def command_listener(args):
    logger = logging.getLogger('COMMAND_LISTENER')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(args.base_timeout)
        try:
            sock.bind(('', args.port))
            sock.listen()
            logger.info(
                'Escutando por comandos do Gateway em (%s, %s)',
                args.host_ip,
                args.port,
            )
        except Exception as e:
            logger.error(
                'Erro as iniciar canal de escuta '
                'de comandos em (%s, %s): (%s) %s',
                args.host_ip,
                args.port,
                type(e).__name__,
                e,
            )
            raise e
        with ThreadPoolExecutor(max_workers=5) as executor:
            while not args.stop_flag.is_set():
                try:
                    conn, addrs = sock.accept()
                except TimeoutError:
                    continue
                except Exception as e:
                    logger.error(
                        'Erro ao aceitar conexão: (%s) %s',
                        type(e).__name__,
                        e,
                    )
                    continue
                try:
                    with args.connection_lock:
                        gateway_ip = args.gateway_ip
                    if addrs[0] != gateway_ip:
                        try:
                            conn.shutdown(socket.SHUT_RDWR)
                        except OSError:
                            logger.error(
                                'Erro ao tentar enviar FIN para %s',
                                addrs,
                            )
                        finally:
                            conn.close()
                        logger.warning('Conexão desconhecida rejeitada')
                        continue
                    else:
                        conn.settimeout(sock.gettimeout())
                        executor.submit(command_handler, args, conn, addrs)
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


def state_change_reporter(args):
    logger = logging.getLogger('STATE_CHANGE_REPORTER')
    logger.info('Iniciando thread de divulgação de atualizações')
    idle_time = 0
    while not args.stop_flag.is_set():
        with args.connection_lock:
            transmission_addrs = (args.gateway_ip, args.transmission_port)
        if transmission_addrs[0] is None:
            logger.info('Transmissão interrompida. Sem conexão com o Gateway')
            time.sleep(2.0)
            continue
        if not args.state_change.is_set() and idle_time < args.update_interval:
            time.sleep(1.0)
            idle_time += 1
            continue
        idle_time = 0
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            try:
                sock.settimeout(args.base_timeout)
                sock.connect(transmission_addrs)
            except Exception as e:
                logger.error(
                    'Conexão com o Gateway em %s falhou: (%s) %s',
                    transmission_addrs,
                    type(e).__name__,
                    e,
                )
                time.sleep(2.0)
                continue
            try:
                with args.state_lock:
                    state = json.dumps(args.state)
                    args.state_change.clear()
                    timestamp = datetime.now(UTC).isoformat()
                update = build_update_message(args, state, timestamp)
                sock.send(update.SerializeToString())
                logger.debug(
                    'Atualização de estado enviada para %s',
                    transmission_addrs,
                )
            except Exception as e:
                args.state_change.set()
                logger.error(
                    'Erro ao enviar atualização para %s: (%s) %s',
                    transmission_addrs,
                    type(e).__name__,
                    e,
                )
                continue


def phase_generator(args):
    while True:
        with args.state_lock:
            args.state['Phase'] = 'Red'
            args.state_change.set()
            phase_period = args.state['RedPeriod']
        yield 'Red', phase_period
        with args.state_lock:
            args.state['Phase'] = 'Green'
            args.state_change.set()
            phase_period = args.state['GreenPeriod']
        yield 'Green', phase_period
        with args.state_lock:
            args.state['Phase'] = 'Yellow'
            args.state_change.set()
            phase_period = args.state['YellowPeriod']
        yield 'Yellow', phase_period


def simulator(args):
    logger = logging.getLogger('SIMULATOR')
    logger.info('Iniciando simulação de um semáforo')
    phases = phase_generator(args)
    while not args.stop_flag.is_set():
        phase, period = next(phases)
        logger.debug(f'Fase "{phase}" começou: duração de {period} secs')
        time.sleep(period)


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
        reporter = threading.Thread(
            target=stop_wrapper(state_change_reporter, args.stop_flag),
            args=(args,)
        )
        listener = threading.Thread(
            target=stop_wrapper(command_listener, args.stop_flag),
            args=(args,)
        )
        discoverer = threading.Thread(
            target=stop_wrapper(gateway_discoverer, args.stop_flag),
            args=(args,)
        )
        reporter.start()
        listener.start()
        discoverer.start()
        simulator(args)
    except KeyboardInterrupt:
        print('\nSHUTTING DOWN...')
    finally:
        args.stop_flag.set()
        reporter.join()
        listener.join()
        discoverer.join()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Simulador de semáforo.')

    parser.add_argument(
        '--id', type=int, default=1,
        help='Id que unicamente identifica o semáforo.'
    )

    parser.add_argument(
        '--port', type=int, default=60000,
        help='Porta na qual o Gateway envia comandos ao atuador.'
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
        '--disconnect_after', type=int, default=3,
        help='Número de falhas sequenciais necessárias para desconectar o Gateway.'
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
    args.name = f'semaphore-{args.id}'

    # Timeouts
    args.base_timeout = 2.0
    args.multicast_timeout = 5.0

    # Host IP
    args.host_ip = socket.gethostbyname('localhost')

    # Send update after `update_interval` secs without communication
    args.update_interval = 5

    # Gateway
    args.gateway_ip = None
    args.transmission_port = None

    # State and metadata
    args.state = {
        'GreenPeriod': 20.0,
        'YellowPeriod': 5.0,
        'RedPeriod': 40.0,
        'Phase': 'Unset'
    }
    args.metadata = {
        'Location': {'Latitude': -3.734431, 'Longitude': -38.568971},
        'Target': "R. Licurgo Montenegro X Av. Governador Parsifal Barroso",
        'Phases': ['Unset', 'Green', 'Yellow', 'Red'],
        'Actions': [],
    }

    # Events and locks
    args.stop_flag = threading.Event()
    args.state_change = threading.Event()
    args.connection_lock = threading.Lock()
    args.state_lock = threading.Lock()

    return _run(args)


if __name__ == '__main__':
    main()
