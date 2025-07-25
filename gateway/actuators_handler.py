import json
import time
import logging
import grpc
from datetime import datetime, date
from concurrent.futures import ThreadPoolExecutor
from messages_pb2 import ActuatorUpdate, ActuatorsReport, Empty
from messages_pb2 import CommandType, ActuatorCommand, ActuatorComply
from messages_pb2_grpc import GatewayServiceServicer, add_GatewayServiceServicer_to_server


def actuators_report_generator(args):
    logger = logging.getLogger('ACTUATORS_REPORT_GENERATOR')
    logger.info('Iniciando o gerador de relatórios dos atuadores')
    idle_time = 0
    while not args.stop_flag.is_set():
        if (
            not args.pending_actuators_updates.is_set()
            and idle_time < args.reports_gen_interval
        ):
            time.sleep(1.0)
            idle_time += 1
            continue
        idle_time = 0
        with args.db_actuators_lock:
            actuators = args.db.get_actuators_summary()
            args.pending_actuators_updates.clear()
        today = date.today()
        now_clock = time.monotonic()
        tolerance = args.actuators_tolerance
        for i, actuator in enumerate(actuators):
            last_seen = actuator['last_seen']
            is_online = (
                last_seen[0] == today
                and (now_clock - last_seen[1]) <= tolerance
            )
            actuators[i] = ActuatorUpdate(
                device_name=actuator['name'],
                state=json.dumps(actuator['state']),
                metadata=json.dumps(actuator['metadata']),
                timestamp=actuator['timestamp'].isoformat(),
                is_online=is_online,
            )
        logger.debug(
            'Novo relatório gerado: %d atuadores reportados',
            len(actuators),
        )
        report = ActuatorsReport(devices=actuators).SerializeToString()
        with args.db_actuators_report_lock:
            args.db.actuators_report = report


class ActuatorServicer(GatewayServiceServicer):
    def __init__(self, args):
        self.args = args
        self.logger = logging.getLogger('ACTUATOR_SERVICER')

    def GetActuatorsReport(self, request, context):
        try:
            self.logger.debug('Solicitado relatório de atuadores')
            with self.args.db_actuators_report_lock:
                report_data = self.args.db.actuators_report
            if not report_data:
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details('Relatório de atuadores não disponível')
                return ActuatorsReport()
            return ActuatorsReport.FromString(report_data)
        except Exception as e:
            self.logger.error(
                'Erro ao gerar relatório de atuadores: (%s) %s',
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ActuatorsReport()

    def GetActuatorUpdate(self, request, context):
        try:
            device_name = request.device_name
            self.logger.debug('Solicitada atualização do atuador: %s', device_name)
            
            with self.args.db_actuators_lock:
                actuator = self.args.db.get_actuator(device_name)
            
            if actuator is None:
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details(f'Atuador {device_name} não encontrado')
                return ActuatorUpdate()
            
            return ActuatorUpdate(
                device_name=device_name,
                state=json.dumps(actuator['state']),
                metadata=json.dumps(actuator['metadata']),
                timestamp=actuator['timestamp'].isoformat(),
                is_online=actuator['is_online'],
            )
        except Exception as e:
            self.logger.error(
                'Erro ao recuperar atualização do atuador %s: (%s) %s',
                device_name,
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ActuatorUpdate()

    def SendActuatorCommand(self, request, context):
        try:
            self.logger.debug('Recebido comando para atuador: %s', request)
            
            # Verificar se o atuador está registrado
            if not self.args.db.is_actuator_registered(request.device_name):
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details(f'Atuador {request.device_name} não encontrado')
                return ActuatorComply(
                    status=ComplyStatus.CS_FAIL,
                    update=ActuatorUpdate()
                )
            
            # Enviar o comando para o atuador
            comply_msg = self.send_command_to_actuator(
                request.device_name,
                request.type,
                request.body
            )
            
            if comply_msg is None:
                context.set_code(grpc.StatusCode.UNAVAILABLE)
                context.set_details('Falha ao comunicar com o atuador')
                return ActuatorComply(
                    status=ComplyStatus.CS_FAIL,
                    update=ActuatorUpdate()
                )
            
            return comply_msg
            
        except Exception as e:
            self.logger.error(
                'Erro ao processar comando para atuador: (%s) %s',
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ActuatorComply(
                status=ComplyStatus.CS_FAIL,
                update=ActuatorUpdate()
            )

    def build_command_message(self, command_type, body):
        match command_type:
            case CommandType.CT_ACTION | CommandType.CT_SET_STATE:
                msg = ActuatorCommand(type=command_type, body=body)
            case CommandType.CT_GET_STATE:
                msg = ActuatorCommand(type=command_type)
            case _:
                return None
        return msg.SerializeToString()

    def send_command_to_actuator(self, actuator_name, command_type, command_body):
        command = self.build_command_message(command_type, command_body)
        if command is None:
            return None
        
        with self.args.db_actuators_lock:
            address = self.args.db.get_actuator_address_by_name(actuator_name)
        
        if address is None:
            return None
        
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(self.args.base_timeout)
                sock.connect(address)
                sock.send(command)
                msg = sock.recv(1024)
        except Exception as e:
            self.logger.error(
                'Erro ao enviar comando para atuador %s: (%s) %s',
                actuator_name,
                type(e).__name__,
                e,
            )
            with self.args.db_actuators_lock:
                self.args.db.mark_actuator_as_offline(actuator_name)
                self.args.pending_actuators_updates.set()
            return None
        
        reply = ActuatorComply()
        try:
            reply.ParseFromString(msg)
        except Exception as e:
            self.logger.error(
                'Erro ao desserializar resposta do atuador %s: (%s) %s',
                actuator_name,
                type(e).__name__,
                e,
            )
            return None
        
        # Atualizar o banco de dados com a resposta
        state = json.loads(reply.update.state)
        metadata = json.loads(reply.update.metadata)
        timestamp = datetime.fromisoformat(reply.update.timestamp)
        
        with self.args.db_actuators_lock:
            self.args.db.add_actuator_update(
                actuator_name,
                state,
                metadata,
                timestamp,
            )
            self.args.pending_actuators_updates.set()
        
        return reply


def run_actuator_server(args):
    logger = logging.getLogger('ACTUATOR_SERVER_GRPC')
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    servicer = ActuatorServicer(args)
    add_GatewayServiceServicer_to_server(servicer, server)
    server.add_insecure_port(f'[::]:{args.actuators_port}')
    server.start()
    logger.info(
        'Servidor gRPC para atuadores iniciado na porta %s',
        args.actuators_port,
    )
    return server