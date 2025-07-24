import sys
import socket
import threading
import logging
from functools import wraps
from api import app, ApiServerThread
from registration_handler import multicast_locations, registration_listener
from sensors_handler import sensors_consumer
from actuators_handler import actuators_listener
from clients_handler import clients_listener
from db.sessions import init_db


def stop_wrapper(func, stop_flag):
    @wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        finally:
            stop_flag.set()
    return wrapper


def load_configs():
    import yaml
    from pathlib import Path
    from types import SimpleNamespace
    config_file = Path(__file__).resolve().parent / 'config.yaml'
    with config_file.open('r') as f:
        configs = SimpleNamespace(**yaml.safe_load(f))
    configs.host_ip = socket.gethostbyname('localhost')
    if configs.broker_ip == 'localhost':
        configs.broker_ip = configs.host_ip
    return configs


def _run():
    stop_flag = threading.Event()
    try:
        configs = load_configs()
        se_consumer = threading.Thread(
            target=stop_wrapper(sensors_consumer, stop_flag),
            args=(
                stop_flag,
                configs.broker_ip,
                configs.broker_port,
                configs.publish_exchange,
            ),
        )
        ac_listener = threading.Thread(
            target=stop_wrapper(actuators_listener, stop_flag),
            args=(
                stop_flag,
                configs.actuators_port,
            ),
        )
        cl_listener = threading.Thread(
            target=stop_wrapper(clients_listener, stop_flag),
            args=(
                stop_flag,
                configs.clients_port,
            ),
        )
        re_listener = threading.Thread(
            target=stop_wrapper(registration_listener, stop_flag),
            args=(
                stop_flag,
                configs.registration_port,
                configs.sensors_tolerance,
                configs.actuators_port,
                configs.actuators_tolerance,
            ),
        )
        multicaster = threading.Thread(
            target=stop_wrapper(multicast_locations, stop_flag),
            args=(
                stop_flag,
                configs.multicast_ip,
                configs.multicast_port,
                configs.multicast_interval,
                configs.host_ip,
                configs.registration_port,
                configs.broker_ip,
                configs.broker_port,
                configs.publish_exchange,
            ),
        )
        api_server = ApiServerThread(
            stop_flag,
            configs.host_ip,
            configs.api_port,
            app,
        )
        se_consumer.start()
        ac_listener.start()
        cl_listener.start()
        re_listener.start()
        multicaster.start()
        api_server.start()
        stop_flag.wait()
    except KeyboardInterrupt:
        print('\nSHUTTING DOWN...')
    finally:
        stop_flag.set()
        se_consumer.join()
        ac_listener.join()
        cl_listener.join()
        re_listener.join()
        multicaster.join()
        api_server.shutdown()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Gateway central.')

    parser.add_argument(
        '-l', '--level', type=str, default='INFO',
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR'],
        help='Nível do logging.'
    )

    parser.add_argument(
        '-c', '--clear', action='store_true',
        help='Limpar o banco de dados ao iniciar.'
    )

    args = parser.parse_args()

    # Logging
    logging.basicConfig(
        level=args.level,
        handlers=(logging.StreamHandler(sys.stdout),),
        format='[%(levelname)s %(asctime)s] %(name)s\n  %(message)s',
    )

    # Database
    init_db(args.clear)

    return _run()


if __name__ == '__main__':
    main()
