import sys
import time
import json
import socket
import logging
import datetime
import threading
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
            socket.inet_aton(args.multicast_ip) + socket.inet_aton('0.0.0.0')
        )
        logger.info(
            'Procurando pelo Gateway no grupo multicast (%s, %s)',
            args.multicast_ip, args.multicast_port
        )
        sock.settimeout(args.multicast_timeout)
        fail_counter = 0
        while not args.stop_flag.is_set():
            try:
                msg = sock.recv(1024)
            except TimeoutError:
                if args.gateway_ip is not None:
                    logger.warning(
                        'Falha de transmissão do Gateway em %s', args.gateway_ip
                    )
                    fail_counter += 1
                    if fail_counter >= args.disconnect_after:
                        logger.warning(
                            'Gateway offline: falhou %d transmissões '
                            'em sequência. Desconectando...',
                            args.disconnect_after
                        )
                        disconnect_device(args)
                continue
            gateway_addrs = Address()
            gateway_addrs.ParseFromString(msg)
            if args.gateway_ip is not None:
                if gateway_addrs.ip == args.gateway_ip:
                    continue
                else:
                    logger.warning(
                        'Gateway realocado de %s para %s. '
                        'Desconectando...',
                        args.gateway_ip, gateway_addrs.ip,
                    )
                    disconnect_device(args)
            if try_to_connect(args, (gateway_addrs.ip, gateway_addrs.port)):
                fail_counter = 0


def disconnect_device(args):
    args.gateway_ip = None
    args.transmission_port = None
    return


def try_to_connect(args, addrs):
    logger = logging.getLogger('TRY_CONNECTION')
    logger.info('Tentando conexão com %s', addrs)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(args.base_timeout)
        try:
            sock.connect(addrs)
            device_info = DeviceInfo(
                type=DeviceType.DT_ACTUATOR,
                name=args.name,
                state=json.dumps(args.state),
                metadata=json.dumps(args.metadata),
                timestamp=datetime.datetime.now(datetime.UTC).isoformat(),
            )
            device_address = Address(ip=args.host_ip, port=args.port)
            join_request = JoinRequest(
                device_info=device_info,
                device_address=device_address,
            )
            sock.send(join_request.SerializeToString())
            join_reply = JoinReply()
            join_reply.ParseFromString(sock.recv(1024))
        except Exception as e:
            logger.warning(
                'Erro durante tentativa de conexão com %s: (%s) %s',
                addrs, type(e).__name__, e
            )
            return False
    args.gateway_ip = addrs[0]
    args.transmission_port = join_reply.report_port
    logger.info(
        'Conexão bem-sucedida com o Gateway em (%s, %s)',
        addrs[0], join_reply.report_port
    )
    return True


def get_update_message(args):
    return ActuatorUpdate(
        device_name=args.name,
        state=json.dumps(args.state),
        metadata=json.dumps(args.metadata),
        timestamp=datetime.datetime.now(datetime.UTC).isoformat(),
    )


def process_set_state_command(args, state_string):
    new_state = json.loads(state_string)
    unknown_states = set(new_state) - set(args.state)
    if unknown_states:
        return False
    if 'Phase' in new_state:
        return False
    for period in ('GreenPeriod', 'YellowPeriod', 'RedPeriod'):
        try:
            period = new_state[period]
        except KeyError:
            continue
        if not isinstance(period, float):
            return False
        # Período mínimo de 5 segundos
        if period < 5.0:
            return False
    args.state.update(new_state)
    args.state_change.set()
    return True, 'OK'


def process_command(args, command, logger):
    match command.type:
        case CommandType.CT_ACTION:
            logger.info('Semáforo recebeu uma requisição do tipo CT_ACTION')
            comply = ActuatorComply(
                status=ComplyStatus.CS_UNKNOWN_ACTION,
                update=get_update_message(args),
            )
        case CommandType.CT_GET_STATE:
            logger.info('Semáforo recebeu uma requisição do tipo CT_GET_STATE')
            comply = ActuatorComply(
                status=ComplyStatus.CS_OK,
                update=get_update_message(args),
            )
        case CommandType.CT_SET_STATE:
            result = process_set_state_command(args, command.body)
            if result:
                logger.info('Requisição CT_SET_STATE respondida com sucesso')
                status = ComplyStatus.CS_OK
            else:
                logger.info('Requisição CT_SET_STATE gerou CS_INVALID_STATE')
                status = ComplyStatus.CS_INVALID_STATE
            comply = ActuatorComply(
                status=status, update=get_update_message(args)
            )
    return comply.SerializeToString()


def command_listener(args):
    logger = logging.getLogger('COMMAND_LISTENER')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('', args.port))
            sock.listen()
            logger.info(
                'Escutando por comandos do Gateway em (%s, %s)',
                args.host_ip, args.clients_port
            )
        except Exception as e:
            logger.error(
                'Erro as iniciar socket de escuta '
                'de comandos em (%s, %s): (%s) %s',
                args.host_ip,
                args.port,
                type(e).__name__,
                e
            )
        sock.settimeout(args.base_timeout)
        while not args.stop_flag.is_set():
            try:
                conn, addrs = sock.accept()
            except TimeoutError:
                continue
            except Exception as e:
                logger.error(
                    'Erro ao tentar conexão com o Gateway: (%s) %s',
                    type(e).__name__,
                    e,
                )
                continue
            try:
                if addrs[0] != args.gateway_ip:
                    logger.warning('Conexão indesejada rejeitada')
                    conn.shutdown(socket.SHUT_RDWR)
                    conn.close()
                    continue
                command = ActuatorCommand()
                command.ParseFromString(conn.recv(1024))
            except Exception as e:
                logger.error(
                    'Erro ao receber comando do Gateway: (%s) %s',
                    type(e).__name__,
                    e,
                )
                conn.shutdown(socket.SHUT_RDWR)
                conn.close()
                continue
            try:
                comply = process_command(args, command, logger)
            except Exception as e:
                logger.error(
                    'Erro ao processar comando do Gateway: (%s) %s',
                    type(e).__name__,
                    e,
                )
                conn.shutdown(socket.SHUT_RDWR)
                conn.close()
                continue
            try:
                conn.send(comply)
            except Exception as e:
                logger.error(
                    'Erro ao enviar mensagem de '
                    'cumprimento de comando: (%s) %s',
                    type(e).__name__,
                    e,
                )
                continue
            finally:
                conn.shutdown(socket.SHUT_RDWR)
                conn.close()


def state_change_reporter(args):
    logger = logging.getLogger('STATE_CHANGE_REPORTER')
    logger.info('Iniciando thread de divulgação de atualizações')
    while not args.stop_flag.is_set():
        if not args.state_change.is_set():
            time.sleep(1.0)
            continue
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(args.gateway_timeout)
            try:
                sock.connect((args.gateway_ip, args.transmission_port))
            except Exception as e:
                if args.gateway_ip is not None:
                    logger.error(
                        'Conexão com (%s, %s) falhou: (%s) %s',
                        args.gateway_ip,
                        args.transmission_port,
                        type(e).__name__,
                        e,
                    )
                continue
            try:
                sock.send(get_update_message(args).SerializeToString())
                args.state_change.clear()
            except Exception as e:
                if args.gateway_ip is not None:
                    logger.error(
                        'Erro ao enviar atualização para (%s, %s): (%s, %s)',
                        args.gateway_ip,
                        args.transmission_port,
                        type(e).__name__,
                        e,
                    )
            finally:
                sock.shutdown(socket.SHUT_RDWR)


def phase_generator(args):
    while True:
        args.state_change.set()
        yield 'Red', args.state['RedPeriod']
        args.state_change.set()
        yield 'Green', args.state['GreenPeriod']
        args.state_change.set()
        yield 'Yellow', args.state['YellowPeriod']


def simulator(args):
    logger = logging.getLogger('SIMULATOR')
    logger.info('Iniciando simulação de um semáforo')
    phases = phase_generator(args)
    while not args.stop_flag.is_set():
        phase, period = next(phases)
        logger.debug(f'Fase "{phase}" começou: duranção de {period} secs')
        time.sleep(period)


def _run(args):
    logging.basicConfig(
        level=args.level,
        handlers=(logging.StreamHandler(sys.stdout),),
        format='[%(levelname)s %(asctime)s] %(name)s\n  %(message)s',
    )
    try:
        reporter = threading.Thread(
            target=state_change_reporter, args=(args,)
        )
        listener = threading.Thread(
            target=command_listener, args=(args,)
        )
        discoverer = threading.Thread(
            target=gateway_discoverer, args=(args,)
        )
        reporter.start()
        listener.start()
        discoverer.start()
        simulator(args)
    except BaseException as e:
        args.stop_flag.set()
        if isinstance(e, KeyboardInterrupt):
            print('\nDESLIGANDO...')
        else:
            raise e
    finally:
        discoverer.join()
        listener.join()
        reporter.join()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Simulador de semáforo')

    parser.add_argument(
        '--name', type=str, default='01',
        help='Nome que unicamente identifica o semáforo.'
    )

    parser.add_argument(
        '--port', type=int, default=5001,
        help='Porta na qual o Gateway envia comandos ao atuador.'
    )

    parser.add_argument(
        '--multicast_ip', type=str, default='224.0.1.0',
        help='IP multicast para descobrimento do Gateway.'
    )

    parser.add_argument(
        '--multicast_port', type=int, default=12345,
        help='Porta na qual escutar por mensagens do grupo multicast.'
    )

    parser.add_argument(
        '--disconnect_after', type=int, default=3,
        help='Número de falhas sequenciais necessárias para desconectar o Gateway.'
    )

    parser.add_argument(
        '-v', '--verbose', action='store_true',
        help='Torna o Gateway verboso ao logar informações.'
    )

    parser.add_argument(
        '-l', '--level', type=str, default='INFO',
        help='Nível do logging. Valores permitidos são "DEBUG", "INFO", "WARN", "ERROR".'
    )

    args = parser.parse_args()

    # Logging
    lvl = args.level.strip().upper()
    args.level = lvl if lvl in ('DEBUG', 'WARN', 'ERROR') else 'INFO'
    if args.level == 'DEBUG':
        args.verbose = True
    
    # Identifier
    args.name = f'Semaphore-{args.name}'

    # Timeouts
    args.base_timeout = 2.5
    args.multicast_timeout = 5.0
    args.gateway_timeout = 2.0

    # Host IP
    args.host_ip = socket.gethostbyname(socket.gethostname())

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
        'Phases': ('Unset', 'Green', 'Yellow', 'Red'),
    }

    # Stop flag
    args.stop_flag = threading.Event()
    args.state_change = threading.Event()

    return _run(args)


if __name__ == '__main__':
    main()
