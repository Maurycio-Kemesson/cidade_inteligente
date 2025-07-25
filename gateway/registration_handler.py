import time
import json
import socket
import logging
import grpc
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
from messages_pb2 import Address, JoinRequest, JoinReply, DeviceType
from messages_pb2_grpc import GatewayServiceServicer, add_GatewayServiceServicer_to_server


def multicast_location(args):
    logger = logging.getLogger('MULTICASTER')
    logger.info(
        'Enviando endereço de registro para grupo multicast (%s, %s)',
        args.multicast_ip, args.multicast_port
    )
    addrs = Address(ip=args.host_ip, port=args.registration_port)
    addrs = addrs.SerializeToString()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        while not args.stop_flag.is_set():
            try:
                sock.sendto(addrs, (args.multicast_ip, args.multicast_port))
            except Exception as e:
                logger.error(
                    'Erro ao enviar mensagem para o grupo multicast: (%s) %s',
                    type(e).__name__,
                    e,
                )
                raise e
            time.sleep(args.multicast_interval)


class GatewayRegistrationServicer(GatewayServiceServicer):
    def __init__(self, args):
        self.args = args

    def RegisterDevice(self, request, context):
        logger = logging.getLogger('REGISTRATION_HANDLER_GRPC')
        logger.info('Processando requisição de registro via gRPC')
        device_info = request.device_info
        device_addrs = request.device_address
        
        try:
            match device_info.type:
                case DeviceType.DT_SENSOR:
                    report_port = self.args.sensors_port
                    metadata = json.loads(device_info.metadata)
                    with self.args.db_sensors_lock:
                        self.args.db.register_sensor(
                            name=device_info.name,
                            address=(device_addrs.ip, device_addrs.port),
                            metadata=metadata,
                        )
                case DeviceType.DT_ACTUATOR:
                    report_port = self.args.actuators_port
                    state = json.loads(device_info.state)
                    metadata = json.loads(device_info.metadata)
                    timestamp = datetime.fromisoformat(device_info.timestamp)
                    with self.args.db_actuators_lock:
                        self.args.db.register_actuator(
                            name=device_info.name,
                            address=(device_addrs.ip, device_addrs.port),
                            state=state,
                            metadata=metadata,
                            timestamp=timestamp,
                        )
                case _:
                    context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
                    context.set_details('Invalid DeviceType')
                    return JoinReply()
            
            logger.info('Ingresso bem-sucedido: %s', device_info.name)
            return JoinReply(report_port=report_port)
        
        except Exception as e:
            logger.error(
                'Erro durante registro do dispositivo %s: (%s) %s',
                device_info.name,
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return JoinReply()


def run_registration_server(args):
    logger = logging.getLogger('REGISTRATION_SERVER_GRPC')
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    servicer = GatewayRegistrationServicer(args)
    add_GatewayServiceServicer_to_server(servicer, server)
    server.add_insecure_port(f'[::]:{args.registration_port}')
    server.start()
    logger.info(
        'Servidor gRPC de registro iniciado na porta %s',
        args.registration_port,
    )
    return server