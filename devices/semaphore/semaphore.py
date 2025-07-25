import sys
import time
import json
import socket
import logging
import threading
import grpc
from numbers import Real
from datetime import datetime, UTC
from functools import wraps
from concurrent.futures import ThreadPoolExecutor
from messages_pb2 import Address
from messages_pb2 import DeviceType, DeviceInfo, JoinRequest, JoinReply
from messages_pb2 import ActuatorUpdate
from messages_pb2 import CommandType, ActuatorCommand
from messages_pb2 import ComplyStatus, ActuatorComply
from messages_pb2_grpc import GatewayServiceStub



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
            gateway_addrs.ParseFromString(msg)
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
    
    # Cria canal gRPC e stub
    channel = grpc.insecure_channel(f"{address[0]}:{address[1]}")
    stub = GatewayServiceStub(channel)
    
    try:
        # Chamada gRPC para registro
        join_reply = stub.RegisterDevice(join_request)
    except grpc.RpcError as e:
        logger.warning(
            'Erro durante registro em %s: (%s) %s',
            address,
            e.code().name,
            e.details(),
        )
        return
    except Exception as e:
        logger.warning(
            'Erro durante registro em %s: (%s) %s',
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

def command_handler(args, context, command):
    logger = logging.getLogger('COMMAND_HANDLER')
    try:
        comply = process_command(args, command, logger)
        return comply
    except Exception as e:
        logger.error(
            'Erro durante processamento de comando: (%s) %s',
            type(e).__name__,
            e,
        )
        return ActuatorComply(
            status=ComplyStatus.CS_FAIL,
            update=build_update_message(args, json.dumps(args.state), datetime.now(UTC).isoformat())
        )

class GatewayCommandServicer(messages_pb2_grpc.GatewayServiceServicer):
    def __init__(self, args):
        self.args = args

    def SendActuatorCommand(self, request, context):
        return command_handler(self.args, context, request)

def run_grpc_server(args):
    logger = logging.getLogger('GRPC_SERVER')
    server = grpc.server(ThreadPoolExecutor(max_workers=5))
    messages_pb2_grpc.add_GatewayServiceServicer_to_server(
        GatewayCommandServicer(args), server
    )
    server.add_insecure_port(f'0.0.0.0:{args.port}')
    server.start()
    logger.info('Servidor gRPC iniciado na porta %d', args.port)
    return server

def state_change_reporter(args):
    logger = logging.getLogger('STATE_CHANGE_REPORTER')
    logger.info('Iniciando thread de divulgação de atualizações')
    idle_time = 0
    channel = None
    stub = None
    
    while not args.stop_flag.is_set():
        # Reconecta se necessário
        if channel is None or channel._channel.check_connectivity_state(True) != grpc.ChannelConnectivity.READY:
            if channel:
                channel.close()
            with args.connection_lock:
                if not args.gateway_ip:
                    time.sleep(2.0)
                    continue
                try:
                    channel = grpc.insecure_channel(f"{args.gateway_ip}:{args.transmission_port}")
                    stub = GatewayServiceStub(channel)
                    grpc.channel_ready_future(channel).result(timeout=args.base_timeout)
                except Exception as e:
                    logger.error('Falha ao conectar com Gateway: %s', e)
                    time.sleep(2.0)
                    continue

        # Envia atualização
        try:
            with args.state_lock:
                state = json.dumps(args.state)
                args.state_change.clear()
                timestamp = datetime.now(UTC).isoformat()
            update = build_update_message(args, state, timestamp)
            stub.GetActuatorUpdate(update)  # Método alternativo para envio
            logger.debug('Atualização de estado enviada via gRPC')
            idle_time = 0
        except grpc.RpcError as e:
            logger.error('Erro no envio de atualização: (%s) %s', e.code().name, e.details())
            args.state_change.set()
        except Exception as e:
            logger.error('Erro geral no envio: %s', e)
            args.state_change.set()
        
        # Espera inteligente
        sleep_time = 1.0 if args.state_change.is_set() else min(5.0, args.update_interval - idle_time)
        time.sleep(sleep_time)
        idle_time += sleep_time

def _run(args):
    logging.basicConfig(
        level=args.level,
        handlers=(logging.StreamHandler(sys.stdout),),
        format='[%(levelname)s %(asctime)s] %(name)s\n  %(message)s',
    )
    try:
        # Inicia servidor gRPC
        grpc_server = run_grpc_server(args)
        
        reporter = threading.Thread(
            target=stop_wrapper(state_change_reporter, args.stop_flag),
            args=(args,)
        )
        discoverer = threading.Thread(
            target=stop_wrapper(gateway_discoverer, args.stop_flag),
            args=(args,)
        )
        
        reporter.start()
        discoverer.start()
        simulator(args)
    except KeyboardInterrupt:
        print('\nSHUTTING DOWN...')
    finally:
        args.stop_flag.set()
        grpc_server.stop(0)
        discoverer.join()
        reporter.join()

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Simulador de semáforo')

    parser.add_argument(
        '--name', type=str, default='01',
        help='Nome que unicamente identifica o semáforo.'
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
        '--multicast_port', type=int, default=50444,
        help='Porta na qual escutar por mensagens do grupo multicast.'
    )

    parser.add_argument(
        '--disconnect_after', type=int, default=3,
        help='Número de falhas sequenciais necessárias para desconectar o Gateway.'
    )

    parser.add_argument(
        '-l', '--level', type=str, default='INFO',
        help='Nível do logging. Valores permitidos são "DEBUG", "INFO", "WARN", "ERROR".'
    )

    args = parser.parse_args()

    # Logging
    lvl = args.level.strip().upper()
    args.level = lvl if lvl in ('DEBUG', 'WARN', 'ERROR') else 'INFO'
    
    # Identifier
    args.name = f'Sema-{args.name}'

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
