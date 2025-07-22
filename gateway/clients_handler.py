import json
import socket
import logging
from struct import pack
from actuators_handler import send_actuator_command
from concurrent.futures import ThreadPoolExecutor
from db.repositories import get_sensors_repository, get_actuators_repository
from messages_pb2 import SensorReading, SensorData
from messages_pb2 import SensorsReport, ActuatorsReport
from messages_pb2 import RequestType, ClientRequest
from messages_pb2 import ReplyStatus, ClientReply
from messages_pb2 import ActuatorUpdate, CommandType, ComplyStatus


def get_sensors_report():
    sensors_repository = get_sensors_repository()
    sensors = sensors_repository.get_all_sensors()
    sensors_summary = []
    for sensor in sensors:
        reading = sensors_repository.get_sensor_last_reading(sensor.id, sensor.category)
        if reading is None:
            continue
        sensors_summary.append(SensorReading(
            device_name=f'{sensor.category}-{sensor.id}',
            reading_value=reading.value,
            timestamp=reading.timestamp.isoformat(),
            metadata=json.dumps(sensor.device_metadata),
            is_online=sensor.is_online(),
        ))
    return SensorsReport(devices=sensors_summary).SerializeToString()


def get_actuators_report():
    actuators_repository = get_actuators_repository()
    actuators_summary = [
        ActuatorUpdate(
            device_name=f'{actuator.category}-{actuator.id}',
            state=json.dumps(actuator.device_state),
            metadata=json.dumps(actuator.device_metadata),
            timestamp=actuator.timestamp.isoformat(),
            is_online=actuator.is_online(),
        )
        for actuator in actuators_repository.get_all_actuators()
    ]
    return ActuatorsReport(devices=actuators_summary).SerializeToString()


def get_sensor_data(device_name):
    sensors_repository = get_sensors_repository()
    sensor_category, sensor_id = device_name.split('-')
    sensor_id = int(sensor_id)
    sensor = sensors_repository.get_sensor(sensor_id, sensor_category)
    if sensor is None:
        return None
    readings = sensors_repository.get_sensor_readings(sensor.id, sensor.category)
    readings = [
        SensorData.SimpleReading(
            timestamp=reading.timestamp.isoformat(),
            reading_value=reading.value,
        )
        for reading in readings
    ]
    return SensorData(
        device_name=device_name,
        metadata=json.dumps(sensor.device_metadata),
        readings=readings,
        is_online=sensor.is_online(),
    )


def get_actuator_update(device_name):
    actuators_repository = get_actuators_repository()
    actuator_category, actuator_id = device_name.split('-')
    actuator_id = int(actuator_id)
    actuator = actuators_repository.get_actuator(actuator_id, actuator_category)
    if actuator is None:
        return None
    return ActuatorUpdate(
        device_name=device_name,
        state=json.dumps(actuator.device_state),
        metadata=json.dumps(actuator.device_metadata),
        timestamp=actuator.timestamp.isoformat(),
        is_online=actuator.is_online(),
    )


def process_set_actuator_state(device_name, state_string):
    actuators_repository = get_actuators_repository()
    actuator_category, actuator_id = device_name.split('-')
    actuator_id = int(actuator_id)
    actuator = actuators_repository.get_actuator(actuator_id, actuator_category)
    if actuator is None:
        return ClientReply(
            status=ReplyStatus.RS_UNKNOWN_DEVICE,
            reply_to=RequestType.RT_SET_ACTUATOR_STATE,
        )
    comply_msg = send_actuator_command(
        actuator_id=actuator_id,
        actuator_category=actuator_category,
        command_type=CommandType.CT_SET_STATE,
        command_body=state_string,
    )
    if comply_msg is None or comply_msg.status is ComplyStatus.CS_FAIL:
        return ClientReply(
            status=ReplyStatus.RS_FAIL,
            reply_to=RequestType.RT_SET_ACTUATOR_STATE,
        )
    if comply_msg.status is ComplyStatus.CS_INVALID_STATE:
        return ClientReply(
            status=ReplyStatus.RS_INVALID_STATE,
            reply_to=RequestType.RT_SET_ACTUATOR_STATE,
        )
    comply_msg.update.is_online = True
    return ClientReply(
        status=ReplyStatus.RS_OK,
        reply_to=RequestType.RT_SET_ACTUATOR_STATE,
        data=comply_msg.update.SerializeToString(),
    )


def process_run_actuator_action(device_name, action_name):
    actuators_repository = get_actuators_repository()
    actuator_category, actuator_id = device_name.split('-')
    actuator_id = int(actuator_id)
    actuator = actuators_repository.get_actuator(actuator_id, actuator_category)
    if actuator is None:
        return ClientReply(
            status=ReplyStatus.RS_UNKNOWN_DEVICE,
            reply_to=RequestType.RT_RUN_ACTUATOR_ACTION,
        )
    comply_msg = send_actuator_command(
        actuator_id=actuator_id,
        actuator_category=actuator_category,
        command_type=CommandType.CT_ACTION,
        command_body=action_name,
    )
    if comply_msg is None or comply_msg.status is ComplyStatus.CS_FAIL:
        return ClientReply(
            status=ReplyStatus.RS_FAIL,
            reply_to=RequestType.RT_RUN_ACTUATOR_ACTION,
        )
    if comply_msg.status is ComplyStatus.CS_UNKNOWN_ACTION:
        return ClientReply(
            status=ReplyStatus.RS_UNKNOWN_ACTION,
            reply_to=RequestType.RT_RUN_ACTUATOR_ACTION,
        )
    return ClientReply(
        status=ReplyStatus.RS_OK,
        reply_to=RequestType.RT_RUN_ACTUATOR_ACTION,
        data=comply_msg.update.SerializeToString(),
    )


def process_client_request(request):
    try:
        match request.type:
            case RequestType.RT_GET_SENSORS_REPORT:
                return ClientReply(
                    status=ReplyStatus.RS_OK,
                    reply_to=request.type,
                    data=get_sensors_report(),
                )
            case RequestType.RT_GET_ACTUATORS_REPORT:
                return ClientReply(
                    status=ReplyStatus.RS_OK,
                    reply_to=request.type,
                    data=get_actuators_report(),
                )
            case RequestType.RT_GET_SENSOR_DATA:
                data = get_sensor_data(request.device_name)
                if data is None:
                    return ClientReply(
                        status=ReplyStatus.RS_UNKNOWN_DEVICE,
                        reply_to=request.type,
                    )
                return ClientReply(
                    status=ReplyStatus.RS_OK,
                    reply_to=request.type,
                    data=data.SerializeToString(),
                )
            case RequestType.RT_GET_ACTUATOR_UPDATE:
                update = get_actuator_update(request.device_name)
                if update is None:
                    return ClientReply(
                        status=ReplyStatus.RS_UNKNOWN_DEVICE,
                        reply_to=request.type,
                    )
                return ClientReply(
                    status=ReplyStatus.RS_OK,
                    reply_to=request.type,
                    data=update.SerializeToString(),
                )
            case RequestType.RT_SET_ACTUATOR_STATE:
                return process_set_actuator_state(
                    device_name=request.device_name,
                    state_string=request.body,
                )
            case RequestType.RT_RUN_ACTUATOR_ACTION:
                return process_run_actuator_action(
                    device_name=request.device_name,
                    action_name=request.body,
                )
            case _:
                return ClientReply(
                    status=ReplyStatus.RS_FAIL,
                    reply_to=RequestType.RT_UNSPECIFIED,
                )
    except Exception:
        return ClientReply(
            status=ReplyStatus.RS_FAIL,
            reply_to=request.type,
        )


def frame_message(message):
    msg_size = len(message)
    msg_size = pack('!I', msg_size)
    return msg_size + message


def client_handler(sock, address):
    try:
        logger = logging.getLogger(f'CLIENT_HANDLER_{address}')
        logger.info('Tratando requisição de um cliente em %s', address)
        try:
            msg = sock.recv(1024)
            request = ClientRequest()
            request.ParseFromString(msg)
        except Exception as e:
            logger.error(
                'Error ao tentar receber requisição do cliente: (%s) %s',
                type(e).__name__,
                e,
            )
            raise e
        try:
            reply = process_client_request(request)
            reply = reply.SerializeToString()
        except Exception as e:
            logger.error(
                'Erro durante o processamento da requisição: (%s) %s',
                type(e).__name__,
                e,
            )
            raise e
        try:
            sock.sendall(frame_message(reply))
        except Exception as e:
            logger.error(
                'Erro ao enviar resposta ao cliente: (%s) %s',
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


def clients_listener(stop_flag, clients_port):
    logger = logging.getLogger('CLIENTS_LISTENER')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', clients_port))
        sock.listen()
        logger.info(
            'Escutando por requisições dos clientes na porta %d',
            clients_port,
        )
        sock.settimeout(1.0)
        with ThreadPoolExecutor(max_workers=5) as executor:
            while not stop_flag.is_set():
                try:
                    conn, addrs = sock.accept()
                except TimeoutError:
                    continue
                except Exception as e:
                    logger.error(
                        'Erro ao tentar conexão com um novo cliente: (%s) %s',
                        type(e).__name__,
                        e,
                    )
                    raise e
                try:
                    conn.settimeout(sock.gettimeout())
                    executor.submit(client_handler, conn, addrs)
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
