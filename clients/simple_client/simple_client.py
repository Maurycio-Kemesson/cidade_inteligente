import sys
import json
import socket
import logging
from struct import unpack
from messages_pb2 import ActuatorUpdate
from messages_pb2 import SensorsReport, ActuatorsReport
from messages_pb2 import RequestType, ClientRequest
from messages_pb2 import ReplyStatus, ClientReply
from messages_pb2 import SensorReading, SensorData


def print_sensor_reading(sensor: SensorReading):
    metadata = json.loads(sensor.metadata)
    print('[INFO]')
    print('  Sensor    : %s' %sensor.device_name)
    print('  Reading   : %.4f' %sensor.reading_value)
    print('  Timestamp : %s' %sensor.timestamp)
    print('  Status    : %s' %('ONLINE' if sensor.is_online else 'OFFLINE'))
    if metadata:
        max_key_length = max(len(k) for k in metadata)
        print('[METADATA]')
        for key, value in metadata.items():
            print(f'  {key.ljust(max_key_length)} : {value}')
    print()


def print_actuator_update(actuator: ActuatorUpdate):
    state = json.loads(actuator.state)
    metadata = json.loads(actuator.metadata)
    print('[INFO]')
    print('  Actuator  : %s' %actuator.device_name)
    print('  Timestamp : %s' %actuator.timestamp)
    print('  Status    : %s' %('ONLINE' if actuator.is_online else 'OFFLINE'))
    if state:
        max_key_length = max(len(k) for k in state)
        print('[STATE]')
        for key, value in state.items():
            print(f'  {key.ljust(max_key_length)} : {value}')
    if metadata:
        max_key_length = max(len(k) for k in metadata)
        print('[METADATA]')
        for key, value in metadata.items():
            print(f'  {key.ljust(max_key_length)} : {value}')
    print()


def print_sensor_data(data: SensorData):
    metadata = json.loads(data.metadata)
    print('[INFO]')
    print('  Sensor : %s' %data.device_name)
    print('  Status : %s' %('ONLINE' if data.is_online else 'OFFLINE'))
    if metadata:
        max_key_length = max(len(k) for k in metadata)
        print('[METADATA]')
        for key, value in metadata.items():
            print(f'  {key.ljust(max_key_length)} : {value}')
    if data.readings:
        print('[DATA]')
        for reading in data.readings:
            print(f'  {reading.timestamp} : {reading.reading_value:.4f}')
    print()


def print_sensors_report(report: SensorsReport):
    if not report.devices:
        print('No sensors to report')
        return
    for sensor in report.devices:
        print_sensor_reading(sensor)


def print_actuators_report(report: ActuatorsReport):
    if not report.devices:
        print('No actuators to report')
        return
    for actuator in report.devices:
        print_actuator_update(actuator)


def recv_exaclty(sock, n, timeout_tolerance=3):
    msg_chunks = []
    remaining = n
    while remaining:
        try:
            chunk = sock.recv(remaining)
        except TimeoutError as e:
            if timeout_tolerance < 1:
                raise e
            timeout_tolerance -= 1
            continue
        if not chunk:
            raise EOFError('Socket closed before receiving all expected data')
        msg_chunks.append(chunk)
        remaining -= len(chunk)
    return b''.join(msg_chunks)


def recv_reply(sock):
    msg_size = recv_exaclty(sock, 4)
    msg_size = unpack('!I', msg_size)[0]
    return recv_exaclty(sock, msg_size)


def get_sensors_report(args):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(args.base_timeout)
        try:
            sock.connect(args.gateway)
        except Exception:
            print('[!] Unable to connect to Gateway')
            return
        try:
            request = ClientRequest(type=RequestType.RT_GET_SENSORS_REPORT)
            request = request.SerializeToString()
            sock.send(request)
        except Exception:
            print('[!] Error when sending request to Gateway')
            return
        try:
            msg = recv_reply(sock)
            reply = ClientReply()
            reply.ParseFromString(msg)
        except Exception:
            print('[!] Error when receiving Gateway response')
            return
    if reply.status is not ReplyStatus.RS_OK:
        print('[!] Gateway failure')
        return
    try:
        report = SensorsReport()
        report.ParseFromString(reply.data)
        return report
    except Exception:
        print('[!] Unable to understand Gateway response')
        return


def get_actuators_report(args):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(args.base_timeout)
        try:
            sock.connect(args.gateway)
        except Exception:
            print('[!] Unable to connect to Gateway')
            return
        try:
            request = ClientRequest(type=RequestType.RT_GET_ACTUATORS_REPORT)
            request = request.SerializeToString()
            sock.send(request)
        except Exception:
            print('[!] Error when sending request to Gateway')
            return
        try:
            msg = recv_reply(sock)
            reply = ClientReply()
            reply.ParseFromString(msg)
        except Exception:
            print('[!] Error when receiving Gateway response')
            return
    if reply.status is not ReplyStatus.RS_OK:
        print('[!] Gateway failure')
        return
    try:
        report = ActuatorsReport()
        report.ParseFromString(reply.data)
        return report
    except Exception:
        print('[!] Unable to understand Gateway response')
        return


def send_action_to_actuator(args, device_name, action_name):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(args.base_timeout)
        try:
            sock.connect(args.gateway)
        except Exception:
            print('[!] Unable to connect to Gateway')
            return
        try:
            request = ClientRequest(
                type=RequestType.RT_RUN_ACTUATOR_ACTION,
                device_name=device_name,
                body=action_name,
            )
            request = request.SerializeToString()
            sock.send(request)
        except Exception:
            print('[!] Error when sending request to Gateway')
            return
        try:
            msg = recv_reply(sock)
            reply = ClientReply()
            reply.ParseFromString(msg)
        except Exception:
            print('[!] Error when receiving Gateway response')
            return
    if reply.status is ReplyStatus.RS_UNKNOWN_DEVICE:
        print(f'[!] Unknown actuator with name "{device_name}"')
        return
    if reply.status is ReplyStatus.RS_UNKNOWN_ACTION:
        print(f'[!] Actuator "{device_name}" do not have action "{action_name}"')
        return
    if reply.status is not ReplyStatus.RS_OK:
        print('[!] Something went wrong...')
        return
    try:
        update = ActuatorUpdate()
        update.ParseFromString(reply.data)
        return update
    except Exception:
        print('[!] Unable to understand Gateway response')
        return


def send_set_state_to_actuator(args, device_name, state_key, state_value):
    state_string = '{"%s": %s}' %(state_key, state_value)
    try:
        _ = json.loads(state_string)
    except json.JSONDecodeError:
        print(f'[!] An invalid JSON was assembled: {state_string}')
        return
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(args.base_timeout)
        try:
            sock.connect(args.gateway)
        except Exception:
            print('[!] Unable to connect to Gateway')
            return
        try:
            request = ClientRequest(
                type=RequestType.RT_SET_ACTUATOR_STATE,
                device_name=device_name,
                body=state_string,
            )
            request = request.SerializeToString()
            sock.send(request)
        except Exception:
            print('[!] Error when sending request to Gateway')
            return
        try:
            msg = recv_reply(sock)
            reply = ClientReply()
            reply.ParseFromString(msg)
        except Exception:
            print('[!] Error when receiving Gateway response')
            return
    if reply.status is ReplyStatus.RS_UNKNOWN_DEVICE:
        print(f'[!] Unknown actuator with name "{device_name}"')
        return
    if reply.status is ReplyStatus.RS_INVALID_STATE:
        print(f'[!] Invalid state for actuator "{device_name}": {state_string}')
        return
    if reply.status is not ReplyStatus.RS_OK:
        print('[!] Something went wrong...')
        return
    try:
        update = ActuatorUpdate()
        update.ParseFromString(reply.data)
        return update
    except Exception:
        print('[!] Unable to understand Gateway response')
        return


def get_sensor_data(args, device_name):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(args.base_timeout)
        try:
            sock.connect(args.gateway)
        except Exception:
            print('[!] Unable to connect to Gateway')
            return
        try:
            request = ClientRequest(
                type=RequestType.RT_GET_SENSOR_DATA,
                device_name=device_name,
            )
            request = request.SerializeToString()
            sock.send(request)
        except Exception:
            print('[!] Error when sending request to Gateway')
            return
        try:
            msg = recv_reply(sock)
            reply = ClientReply()
            reply.ParseFromString(msg)
        except Exception:
            print('[!] Error when receiving Gateway response')
            return
    if reply.status is ReplyStatus.RS_UNKNOWN_DEVICE:
        print(f'[!] Unknown sensor with name "{device_name}"')
        return
    if reply.status is not ReplyStatus.RS_OK:
        print('[!] Something went wrong...')
        return
    try:
        data = SensorData()
        data.ParseFromString(reply.data)
        return data
    except Exception:
        print('[!] Unable to understand Gateway response')
        return


def print_help(bad_command=False):
    if bad_command:
        print('[!] Command not understood', end='\n\n')
    print('The following commands are available:')
    print('  help      : Show this help message')
    print('  sensors   : List sensors devices')
    print('  actuators : List actuators devices')
    print('  sensor <name>')
    print('            : Show all available data of sensor <name>')
    print('  actuator <name> <command>')
    print('            : Send command <command> to actuator <name>')
    print('            : <name> and <command> must not be enclosed in double quotes')
    print('  actuator <name> <key> <value>')
    print('            : Set state <key> to <value> for actuator <name>')
    print('            : <value> must be a valid stringfyed JSON value')
    print('            : <key> must not be enclosed in double quotes')
    print('            : If <value> is a string, it must be enclosed in double quotes', end='\n\n')


def app(args):
    while True:
        command = input('>>> ').strip()
        if not command:
            print_help()
            continue
        command, *params = command.split()
        command = command.lower()
        match command:
            case 'help':
                print_help()
            case 'sensors':
                if params:
                    print_help(True)
                else:
                    report = get_sensors_report(args)
                    if report is not None:
                        print_sensors_report(report)
            case 'actuators':
                if params:
                    print_help(True)
                else:
                    report = get_actuators_report(args)
                    if report is not None:
                        print_actuators_report(report)
            case 'sensor':
                if len(params) == 1:
                    data = get_sensor_data(args, params[0])
                    if data is not None:
                        print_sensor_data(data)
                else:
                    print_help(True)
            case 'actuator':
                if len(params) == 2:
                    update = send_action_to_actuator(args, *params)
                    if update is not None:
                        print_actuator_update(update)
                elif len(params) == 3:
                    update = send_set_state_to_actuator(args, *params)
                    if update is not None:
                        print_actuator_update(update)
                else:
                    print_help(True)
            case _:
                print_help(True)


def _run(args):
    logging.basicConfig(
        level=args.level,
        handlers=(logging.StreamHandler(sys.stdout),),
        format='[%(levelname)s %(asctime)s] %(name)s\n  %(message)s',
    )
    try:
        app(args)
    except KeyboardInterrupt:
        print('\nSHUTTING DOWN...')


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Cliente simples em Python')

    parser.add_argument(
        '--gateway_ip', type=str, default='localhost',
        help='IP do Gateway (servidor).'
    )

    parser.add_argument(
        '--gateway_port', type=int, default=50000,
        help='Porta do Gateway para comunicação com clientes.'
    )

    parser.add_argument(
        '-l', '--level', type=str, default='INFO',
        help='Nível do logging. Valores permitidos são "DEBUG", "INFO", "WARN", "ERROR".'
    )

    args = parser.parse_args()

    # Gateway address
    args.gateway = (args.gateway_ip, args.gateway_port)

    # Logging
    lvl = args.level.strip().upper()
    args.level = lvl if lvl in ('DEBUG', 'WARN', 'ERROR') else 'INFO'

    # Timeouts
    args.base_timeout = 2.0

    return _run(args)

if __name__ == '__main__':
    main()
