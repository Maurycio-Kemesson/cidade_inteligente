import json
import time
import logging
import grpc
import datetime
from concurrent.futures import ThreadPoolExecutor
from actuators_handler import send_actuator_command
from messages_pb2 import SensorData, ActuatorUpdate
from messages_pb2 import RequestType, ClientReply, Empty, Address
from messages_pb2 import SensorsReport, ActuatorsReport
from messages_pb2 import CommandType, ActuatorCommand, ActuatorComply, ReplyStatus
from messages_pb2_grpc import GatewayServiceServicer, add_GatewayServiceServicer_to_server


def get_sensors_report(args):
    with args.db_sensors_report_lock:
        return SensorsReport.FromString(args.db.sensors_report)


def get_actuators_report(args):
    with args.db_actuators_report_lock:
        return ActuatorsReport.FromString(args.db.actuators_report)


def build_sensor_data(args, device_name):
    with args.db_sensors_lock:
        sensor = args.db.get_sensor(device_name)
    if sensor is None:
        return None
    readings = [
        SensorData.SimpleReading(
            timestamp=timestamp.isoformat(), reading_value=reading,
        )
        for timestamp, reading in sensor['data']
    ]
    ls_day, ls_clock = sensor['last_seen']
    is_online = (
        ls_day == datetime.date.today()
        and (time.monotonic() - ls_clock) <= args.sensors_tolerance
    )
    return SensorData(
        device_name=device_name,
        metadata=json.dumps(sensor['metadata']),
        readings=readings,
        is_online=is_online,
    )


def build_actuator_update(args, device_name):
    with args.db_actuators_lock:
        actuator = args.db.get_actuator(device_name)
    if actuator is None:
        return None
    return ActuatorUpdate(
        device_name=device_name,
        state=json.dumps(actuator['state']),
        metadata=json.dumps(actuator['metadata']),
        timestamp=actuator['timestamp'].isoformat(),
        is_online=actuator['is_online'],
    )


class GatewayClientServicer(GatewayServiceServicer):
    def __init__(self, args):
        self.args = args
        self.logger = logging.getLogger('GATEWAY_CLIENT_SERVICER')

    def GetAddress(self, request, context):
        return Address(
            ip=self.args.host_ip,
            port=self.args.clients_port,
        )

    def GetSensorsReport(self, request, context):
        try:
            self.logger.debug('Solicitado relatório de sensores')
            return get_sensors_report(self.args)
        except Exception as e:
            self.logger.error(
                'Erro ao gerar relatório de sensores: (%s) %s',
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return SensorsReport()

    def GetActuatorsReport(self, request, context):
        try:
            self.logger.debug('Solicitado relatório de atuadores')
            return get_actuators_report(self.args)
        except Exception as e:
            self.logger.error(
                'Erro ao gerar relatório de atuadores: (%s) %s',
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ActuatorsReport()

    def GetSensorData(self, request, context):
        try:
            device_name = request.device_name
            self.logger.debug('Solicitados dados do sensor: %s', device_name)
            data = build_sensor_data(self.args, device_name)
            if data is None:
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details(f'Sensor {device_name} não encontrado')
                return SensorData()
            return data
        except Exception as e:
            self.logger.error(
                'Erro ao recuperar dados do sensor %s: (%s) %s',
                device_name,
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return SensorData()

    def GetActuatorUpdate(self, request, context):
        try:
            device_name = request.device_name
            self.logger.debug('Solicitada atualização do atuador: %s', device_name)
            update = build_actuator_update(self.args, device_name)
            if update is None:
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details(f'Atuador {device_name} não encontrado')
                return ActuatorUpdate()
            return update
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
            # Implementação será adicionada abaixo
            pass
        except Exception as e:
            self.logger.error(
                'Erro ao processar comando para atuador: (%s) %s',
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ActuatorComply()


def process_set_actuator_state(args, device_name, state_string):
    if not args.db.is_actuator_registered(device_name):
        return ActuatorComply(
            status=ComplyStatus.CS_FAIL,
            update=ActuatorUpdate()
        )
    
    comply_msg = send_actuator_command(
        args=args,
        actuator_name=device_name,
        command_type=CommandType.CT_SET_STATE,
        command_body=state_string,
    )
    
    if comply_msg is None:
        return ActuatorComply(
            status=ComplyStatus.CS_FAIL,
            update=ActuatorUpdate()
        )
    
    return comply_msg


def process_run_actuator_action(args, device_name, action_name):
    if not args.db.is_actuator_registered(device_name):
        return ActuatorComply(
            status=ComplyStatus.CS_FAIL,
            update=ActuatorUpdate()
        )
    
    comply_msg = send_actuator_command(
        args=args,
        actuator_name=device_name,
        command_type=CommandType.CT_ACTION,
        command_body=action_name,
    )
    
    if comply_msg is None:
        return ActuatorComply(
            status=ComplyStatus.CS_FAIL,
            update=ActuatorUpdate()
        )
    
    return comply_msg


def run_client_server(args):
    logger = logging.getLogger('CLIENT_SERVER_GRPC')
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    servicer = GatewayClientServicer(args)
    add_GatewayServiceServicer_to_server(servicer, server)
    server.add_insecure_port(f'[::]:{args.clients_port}')
    server.start()
    logger.info(
        'Servidor gRPC para clientes iniciado na porta %s',
        args.clients_port,
    )
    return server