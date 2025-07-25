import os
import time
import pickle
import datetime
from copy import deepcopy
from types import SimpleNamespace
from sortedcontainers import SortedList
from messages_pb2 import SensorsReport, ActuatorsReport


class Database:

    def __init__(self, db_file='db.pickle', clear=False):
        self.db_file = db_file
        if clear and os.path.isfile(self.db_file):
            os.remove(self.db_file)
        if os.path.isfile(self.db_file):
            with open(self.db_file, mode='br') as db:
                self.data = pickle.load(db)
            for actuator in self.data.actuators:
                self.data.actuators[actuator]['is_online'] = False
        else:
            self.data = self.empty_db()

    @staticmethod
    def empty_db():
        return SimpleNamespace(
            sensors={},
            actuators={},
            sensors_report=SensorsReport(devices=[]).SerializeToString(),
            actuators_report=ActuatorsReport(devices=[]).SerializeToString(),
        )

    def register_sensor(self, name, address, metadata):
        sensor = self.data.sensors.setdefault(name, {'name': name})
        sensor['address'] = address
        sensor['metadata'] = metadata
        sensor['last_seen'] = (datetime.date.today(), time.monotonic())
        sensor.setdefault('data', SortedList())

    def register_actuator(self, name, address, state, metadata, timestamp):
        actuator = self.data.actuators.setdefault(name, {'name': name})
        actuator['address'] = address
        actuator['state'] = state
        actuator['metadata'] = metadata
        actuator['timestamp'] = timestamp
        actuator['is_online'] = True
        actuator['last_seen'] = (datetime.date.today(), time.monotonic())

    def persist(self):
        with open(self.db_file, mode='bw') as db:
            pickle.dump(self.data, db)

    def get_sensor(self, name):
        try:
            return deepcopy(self.data.sensors[name])
        except KeyError:
            return None

    def get_actuator(self, name):
        try:
            return deepcopy(self.data.actuators[name])
        except KeyError:
            return None

    def is_sensor_registered(self, name):
        return name in self.data.sensors

    def get_sensor_name_by_ip(self, ip):
        for sensor_name, sensor_data in self.data.sensors.items():
            if sensor_data['address'][0] == ip:
                return sensor_name
        return None

    def get_actuator_name_by_ip(self, ip):
        for actuator_name, actuator_data in self.data.actuators.items():
            if actuator_data['address'][0] == ip:
                return actuator_name
        return None

    def is_actuator_registered(self, name):
        return name in self.data.actuators

    def get_actuator_address_by_name(self, name):
        for actuator in self.data.actuators:
            if actuator == name:
                return self.data.actuators[name]['address']
        return None

    def add_sensor_reading(self, name, value, metadata, timestamp):
        try:
            sensor = self.data.sensors[name]
        except KeyError:
            return False
        sensor['data'].add((timestamp, value))
        if sensor['data'][-1][0] == timestamp:
            sensor['metadata'] = metadata
            sensor['last_seen'] = (datetime.date.today(), time.monotonic())
        return True
    
    def add_actuator_update(self, name, state, metadata, timestamp):
        try:
            actuator = self.data.actuators[name]
        except KeyError:
            return False
        if timestamp < actuator['timestamp']:
            return False
        actuator['state'] = state
        actuator['metadata'] = metadata
        actuator['timestamp'] = timestamp
        actuator['is_online'] = True
        actuator['last_seen'] = (datetime.date.today(), time.monotonic())
        return True

    def mark_actuator_as_offline(self, name):
        try:
            self.data.actuators[name]['is_online'] = False
            return True
        except KeyError:
            return False

    def get_sensors_summary(self):
        summary = []
        for sensor_name, sensor_data in self.data.sensors.items():
            try:
                timestamp, reading_value = sensor_data['data'][-1]
            except IndexError:
                continue
            summary.append({
                'device_name': sensor_name,
                'reading_value': reading_value,
                'metadata': deepcopy(sensor_data['metadata']),
                'timestamp': timestamp,
                'last_seen': deepcopy(sensor_data['last_seen']),
            })
        return summary
    
    def get_actuators_summary(self):
        return [deepcopy(actuator) for actuator in self.data.actuators.values()]
