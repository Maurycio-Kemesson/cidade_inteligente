import sys
import socket
import threading
import logging
from functools import wraps
from api import app
from registration_handler import multicast_location, registration_listener
from sensors_handler import sensors_listener
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
    return configs


def _run():
    try:
        configs = load_configs()
        stop_flag = threading.Event()
        rlistener = threading.Thread(
            target=stop_wrapper(registration_listener, stop_flag),
            args=(
                stop_flag,
                configs.registration_port,
                configs.sensors_port,
                configs.sensors_tolerance,
                configs.actuators_port,
                configs.actuators_tolerance,
            ),
        )
        slistener = threading.Thread(
            target=stop_wrapper(sensors_listener, stop_flag),
            args=(
                stop_flag,
                configs.sensors_port,
            ),
        )
        alistener = threading.Thread(
            target=stop_wrapper(actuators_listener, stop_flag),
            args=(
                stop_flag,
                configs.actuators_port,
            ),
        )
        clistener = threading.Thread(
            target=stop_wrapper(clients_listener, stop_flag),
            args=(
                stop_flag,
                configs.clients_port,
            ),
        )
        multicaster = threading.Thread(
            target=stop_wrapper(multicast_location, stop_flag),
            args=(
                stop_flag,
                configs.host_ip,
                configs.multicast_ip,
                configs.multicast_port,
                configs.multicast_interval,
                configs.registration_port,
            ),
        )
        rlistener.start()
        slistener.start()
        alistener.start()
        clistener.start()
        multicaster.start()
        app.run(port=8080)
    finally:
        stop_flag.set()
        rlistener.join()
        slistener.join()
        alistener.join()
        clistener.join()
        multicaster.join()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Gateway')

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
