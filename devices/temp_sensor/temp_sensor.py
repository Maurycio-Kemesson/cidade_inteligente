import sys
import json
import time
import datetime
import random
import threading
import logging
import socket
import grpc
from functools import wraps

# Importar módulos gRPC
from messages_pb2 import (
    DeviceType, DeviceInfo, JoinRequest, 
    SensorReading, Address, Empty
)
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
            except socket.timeout:
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
            except Exception as e:
                logger.error('Erro no multicast: %s', e)
                continue
            
            gateway_addrs = Address()
            try:
                gateway_addrs.ParseFromString(msg)
                seq_fails = 0
            except Exception as e:
                logger.error('Erro ao analisar mensagem multicast: %s', e)
                continue
            
            if gateway_addrs.ip == args.gateway_ip:
                continue
                
            if args.gateway_ip is not None:
                logger.warning(
                    'Gateway realocado de %s para %s. Desconectando...',
                    args.gateway_ip,
                    gateway_addrs.ip,
                )
                disconnect_device(args)
                
            try_to_register(args, gateway_addrs, logger)


def disconnect_device(args):
    with args.connection_lock:
        args.gateway_ip = None
        args.grpc_channel = None
        args.grpc_stub = None
    return


def try_to_register(args, gateway_addr, logger):
    logger.info('Tentando registro no Gateway em %s:%s', 
                gateway_addr.ip, gateway_addr.port)
    
    # Criar informações do dispositivo
    sensor_info = DeviceInfo(
        type=DeviceType.DT_SENSOR,
        name=args.name,
        metadata=json.dumps(args.metadata),
    )
    
    # Criar endereço do dispositivo
    sensor_addrs = Address(ip=args.host_ip, port=0)
    
    # Criar requisição de registro
    join_request = JoinRequest(
        device_info=sensor_info, 
        device_address=sensor_addrs,
    )
    
    try:
        # Criar canal gRPC
        channel = grpc.insecure_channel(f'{gateway_addr.ip}:{gateway_addr.port}')
        stub = GatewayServiceStub(channel)
        
        # Registrar dispositivo
        join_reply = stub.RegisterDevice(join_request)
        
        with args.connection_lock:
            args.gateway_ip = gateway_addr.ip
            args.grpc_channel = channel
            args.grpc_stub = stub
            
        logger.info('Registro bem-sucedido com o Gateway em %s', gateway_addr.ip)
        return True
        
    except grpc.RpcError as rpc_error:
        logger.error('Erro gRPC durante registro: %s', rpc_error.details())
        return False
    except Exception as e:
        logger.error('Erro durante registro: %s', e)
        return False


def get_reading(args):
    temp = args.temperature + random.random() - 0.5
    temp = min(max(temp, args.min_temperature), args.max_temperature)
    args.temperature = temp
    return temp


def transmit_readings(args):
    logger = logging.getLogger('READINGS_TRANSMITER')
    logger.info('Começando a transmissão de leituras para o Gateway')
    
    while not args.stop_flag.is_set():
        # Verificar se temos conexão com o Gateway
        with args.connection_lock:
            if args.grpc_stub is None:
                logger.info('Transmissão interrompida. Sem conexão com o Gateway')
                time.sleep(2.0)
                continue
                
            stub = args.grpc_stub
        
        # Criar leitura
        reading = SensorReading(
            device_name=args.name,
            reading_value=get_reading(args),
            is_online=True,
            timestamp=datetime.datetime.now(datetime.UTC).isoformat(),
            metadata=json.dumps(args.metadata),
        )
        
        try:
            # Enviar leitura via gRPC (usando método existente)
            # Como não temos método específico, usaremos GetSensorData
            # como placeholder (deveria ser criado um método adequado no .proto)
            response = stub.GetSensorData(reading)
            logger.debug('Leitura de temperatura enviada para o Gateway')
        except grpc.RpcError as rpc_error:
            logger.error('Erro gRPC ao enviar leitura: %s', rpc_error.details())
            if rpc_error.code() == grpc.StatusCode.UNAVAILABLE:
                disconnect_device(args)
        except Exception as e:
            logger.error('Erro ao enviar leitura: %s', e)
        
        time.sleep(args.report_interval)


def stop_wrapper(func, stop_flag):
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        finally:
            stop_flag.set()
    return wrapper


def _run(args):
    logging.basicConfig(
        level=args.level,
        handlers=(logging.StreamHandler(sys.stdout),),
        format='[%(levelname)s %(asctime)s] %(name)s\n  %(message)s',
    )
    
    try:
        transmiter = threading.Thread(
            target=stop_wrapper(transmit_readings, args.stop_flag),
            args=(args,)
        )
        transmiter.start()
        gateway_discoverer(args)
    except KeyboardInterrupt:
        print('\nSHUTTING DOWN...')
    finally:
        args.stop_flag.set()
        transmiter.join()
        
        # Fechar canal gRPC ao finalizar
        if args.grpc_channel:
            args.grpc_channel.close()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Sensor de temperatura')

    parser.add_argument(
        '--name', type=str, default='01',
        help='Nome que unicamente identifica o sensor de temperatura.'
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
    args.name = f'Temp-{args.name}'

    # Timeouts
    args.base_timeout = 2.0
    args.multicast_timeout = 5.0

    # Host IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        args.host_ip = s.getsockname()[0]
    except Exception:
        args.host_ip = '127.0.0.1'
    finally:
        s.close()

    # Gateway
    args.gateway_ip = None
    args.grpc_channel = None
    args.grpc_stub = None

    # Metadata
    args.metadata = {
        'UnitName': 'Celsius',
        'UnitSymbol': '°C',
        'Location': {'Latitude': -3.733486, 'Longitude': -38.570860},
    }

    # Events and locks
    args.stop_flag = threading.Event()
    args.connection_lock = threading.Lock()

    return _run(args)


if __name__ == '__main__':
    main()