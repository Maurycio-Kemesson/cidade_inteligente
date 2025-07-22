import socket
import logging
import datetime
from db.repositories import get_sensors_repository
from messages_pb2 import SensorReading


def sensors_listener(stop_flag, sensors_port):
    sensors_repository = get_sensors_repository()
    logger = logging.getLogger('SENSORS_LISTENER')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', sensors_port))
        logger.info(
            'Escutando por dados sensoriais na porta %d',
            sensors_port,
        )
        sock.settimeout(1.0)
        while not stop_flag.is_set():
            try:
                msg, addrs = sock.recvfrom(1024)
            except TimeoutError:
                continue
            reading = SensorReading()
            reading.ParseFromString(msg)
            sensor_category, sensor_id = reading.device_name.split('-')
            sensor_id = int(sensor_id)
            sensor = sensors_repository.get_sensor(sensor_id, sensor_category)
            if sensor is None:
                logger.warning(
                    'Recebendo leituras de um sensor '
                    'não registrado localizado em %s',
                    addrs[0],
                )
                continue
            logger.debug(
                'Leitura de sensor recebida: (%s, %s, %.6f)',
                reading.timestamp,
                reading.device_name,
                reading.reading_value,
            )
            timestamp = datetime.datetime.fromisoformat(reading.timestamp)
            sensors_repository.register_sensor_reading(
                sensor.id, sensor.category, reading.reading_value, timestamp,
            )
