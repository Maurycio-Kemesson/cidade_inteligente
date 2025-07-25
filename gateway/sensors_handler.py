import json
import time
import logging
import grpc
from datetime import date, datetime
from concurrent.futures import ThreadPoolExecutor
from messages_pb2 import SensorReading, SensorsReport, Empty
from messages_pb2_grpc import GatewayServiceServicer, add_GatewayServiceServicer_to_server
from google.protobuf.message import DecodeError


def sensors_report_generator(args):
    logger = logging.getLogger('SENSORS_REPORT_GENERATOR')
    logger.info('Iniciando o gerador de relatórios dos sensores')
    while not args.stop_flag.is_set():
        with args.db_sensors_lock:
            sensors = args.db.get_sensors_summary()
        today = date.today()
        now_clock = time.monotonic()
        tolerance = args.sensors_tolerance
        for i, sensor_summary in enumerate(sensors):
            last_seen = sensor_summary['last_seen']
            is_online = (
                last_seen[0] == today
                and (now_clock - last_seen[1]) <= tolerance
            )
            sensors[i] = SensorReading(
                device_name=sensor_summary['device_name'],
                reading_value=sensor_summary['reading_value'],
                timestamp=sensor_summary['timestamp'].isoformat(),
                metadata=json.dumps(sensor_summary['metadata']),
                is_online=is_online,
            )
        logger.debug(
            'Novo relatório gerado: %d sensores reportados',
            len(sensors),
        )
        report = SensorsReport(devices=sensors).SerializeToString()
        with args.db_sensors_report_lock:
            args.db.sensors_report = report
        time.sleep(args.reports_gen_interval)


class SensorDataServicer(GatewayServiceServicer):
    def __init__(self, args):
        self.args = args
        self.logger = logging.getLogger('SENSOR_DATA_SERVICER')

    def GetSensorsReport(self, request, context):
        try:
            self.logger.debug('Solicitado relatório de sensores')
            with self.args.db_sensors_report_lock:
                report_data = self.args.db.sensors_report
            if not report_data:
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details('Relatório de sensores não disponível')
                return SensorsReport()
            return SensorsReport.FromString(report_data)
        except Exception as e:
            self.logger.error(
                'Erro ao gerar relatório de sensores: (%s) %s',
                type(e).__name__,
                e,
            )
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return SensorsReport()

    def GetSensorData(self, request, context):
        try:
            device_name = request.device_name
            self.logger.debug('Solicitados dados do sensor: %s', device_name)
            
            with self.args.db_sensors_lock:
                sensor = self.args.db.get_sensor(device_name)
            
            if sensor is None:
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details(f'Sensor {device_name} não encontrado')
                return SensorData()
            
            readings = [
                SensorData.SimpleReading(
                    timestamp=timestamp.isoformat(), reading_value=reading,
                )
                for timestamp, reading in sensor['data']
            ]
            
            ls_day, ls_clock = sensor['last_seen']
            is_online = (
                ls_day == date.today()
                and (time.monotonic() - ls_clock) <= self.args.sensors_tolerance
            )
            
            return SensorData(
                device_name=device_name,
                metadata=json.dumps(sensor['metadata']),
                readings=readings,
                is_online=is_online,
            )
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


def run_sensor_server(args):
    logger = logging.getLogger('SENSOR_SERVER_GRPC')
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    servicer = SensorDataServicer(args)
    add_GatewayServiceServicer_to_server(servicer, server)
    server.add_insecure_port(f'[::]:{args.sensors_port}')
    server.start()
    logger.info(
        'Servidor gRPC de dados de sensores iniciado na porta %s',
        args.sensors_port,
    )
    return server