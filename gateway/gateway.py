import sys
import time
import socket
import threading
import logging
from db import Database
from registration_handler import multicast_location, registration_listener
from sensors_handler import sensors_listener, sensors_report_generator
from actuators_handler import actuators_listener, actuators_report_generator
from clients_handler import clients_listener


def _run(args):
    logging.basicConfig(
        level=args.level,
        handlers=(logging.StreamHandler(sys.stdout),),
        format='[%(levelname)s %(asctime)s] %(name)s\n  %(message)s',
    )
    try:
        jlistener = threading.Thread(
            target=registration_listener, args=(args,)
        )
        slistener = threading.Thread(
            target=sensors_listener, args=(args,)
        )
        alistener = threading.Thread(
            target=actuators_listener, args=(args,)
        )
        multicaster = threading.Thread(
            target=multicast_location, args=(args,)
        )
        sgenerator = threading.Thread(
            target=sensors_report_generator, args=(args,)
        )
        agenerator = threading.Thread(
            target=actuators_report_generator, args=(args,)
        )
        jlistener.start()
        slistener.start()
        alistener.start()
        multicaster.start()
        sgenerator.start()
        agenerator.start()
        clients_listener(args)
    except BaseException as e:
        args.stop_flag.set()
        if isinstance(e, KeyboardInterrupt):
            print('\nDESLIGANDO...')
        else:
            raise e
    finally:
        jlistener.join()
        slistener.join()
        alistener.join()
        multicaster.join()
        sgenerator.join()
        agenerator.join()
        args.db.persist()


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Gateway')

    parser.add_argument(
        '--clients_port', type=int, default=5000,
        help='Porta para comunicação com os clientes. Usa TCP.'
    )

    parser.add_argument(
        '--registration_port', type=int, default=50111,
        help='Porta em que os dispositivos se registram no Gateway. Usa TCP.'
    )

    parser.add_argument(
        '--sensors_port', type=int, default=50222,
        help='Porta de recebimento de dados sensoriais. Usa UDP.'
    )

    parser.add_argument(
        '--actuators_port', type=int, default=50333,
        help='Porta de recebimento dos dados dos atuadores. Usa UDP.'
    )

    parser.add_argument(
        '--multicast_ip', type=str, default='224.0.1.0',
        help='IP para multicast do endereço do Gateway.'
    )

    parser.add_argument(
        '--multicast_port', type=int, default=12345,
        help='Porta para multicast do endereço do Gateway.'
    )

    parser.add_argument(
        '--multicast_interval', type=float, default=2.5,
        help='Intervalo de envio do endereço do Gateway para o grupo multicast.'
    )

    parser.add_argument(
        '--sensors_tolerance', type=float, default=10.0,
        help='Quantos segundos sem receber dados de um sensor para considerá-lo offline.'
    )

    parser.add_argument(
        '-v', '--verbose', action='store_true',
        help='Torna o Gateway verboso ao logar informações.'
    )

    parser.add_argument(
        '-l', '--level', type=str, default='INFO',
        help='Nível do logging. Valores permitidos são "DEBUG", "INFO", "WARN" e "ERROR".'
    )

    parser.add_argument(
        '-c', '--clear', action='store_true',
        help='Limpar o banco de dados ao iniciar.'
    )

    args = parser.parse_args()

    # Logging
    lvl = args.level.strip().upper()
    args.level = lvl if lvl in ('DEBUG', 'WARN', 'ERROR') else 'INFO'
    if args.level == 'DEBUG':
        args.verbose = True

    # Timeouts
    args.base_timeout = 2.5
    args.client_timeout = 5.0
    args.actuators_timeout = 5.0
    args.reports_timeout = 10.0

    # Host IP
    args.host_ip = socket.gethostbyname(socket.gethostname())

    # Database and stop event
    args.db = Database(clear=args.clear)
    args.stop_flag = threading.Event()

    # Sensors utilities
    args.db_sensors_lock = threading.Lock()
    args.db_sensors_report_lock = threading.Lock()
    args.sensors_gen_interval = 10.0

    # Actuators utilities
    args.pending_actuators_updates = threading.Event()
    args.db_actuators_lock = threading.Lock()
    args.db_actuators_report_lock = threading.Lock()

    return _run(args)


if __name__ == '__main__':
    main()
