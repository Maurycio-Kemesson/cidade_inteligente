import os
import time
import pickle
import datetime
from copy import deepcopy
from sortedcontainers import SortedList


class Database:

    def __init__(self, db_file='db.pickle', clear=False):
        self.db_file = db_file
        if clear and os.path.isfile(self.db_file):
            os.remove(self.db_file)
        if os.path.isfile(self.db_file):
            with open(self.db_file, mode='br') as db:
                self.devices = pickle.load(db)
            today = datetime.date.today()
            for name, sensor in self.devices[0].items():
                if sensor['last_seen'][0] < today:
                    self.devices[0].pop(name)
            for name, actuator in self.devices[1].items():
                if actuator['last_seen'][0] < today:
                    self.devices[1].pop(name)
                else:
                    self.devices[1][name]['is_online'] = False
        else:
            self.devices = ({}, {})
        self.sensors_report = (0, None)
        self.actuators_report = (0, None)

    def register_sensor(self, name, address, metadata):
        sensor = self.devices[0].setdefault(name, {'name': name})
        sensor['address'] = address
        sensor['metadata'] = metadata
        sensor['last_seen'] = (datetime.date.today(), time.monotonic())
        sensor.setdefault('data', SortedList())

    def register_actuator(self, name, address, state, metadata, timestamp):
        actuator = self.devices[1].setdefault(name, {'name': name})
        actuator['address'] = address
        actuator['state'] = state
        actuator['metadata'] = metadata
        actuator['timestamp'] = timestamp
        actuator['is_online'] = True
        actuator['last_seen'] = (datetime.date.today(), time.monotonic())

    def att_sensors_report(self, report):
        self.sensors_report = (self.sensors_report[0] + 1, report)

    def att_actuators_report(self, report):
        self.actuators_report = (self.actuators_report[0] + 1, report)

    def persist(self):
        with open(self.db_file, mode='bw') as db:
            pickle.dump(self.devices, db)
    
    def get_sensor(self, name):
        try:
            return deepcopy(self.devices[0][name])
        except KeyError:
            return None
    
    def get_actuator(self, name):
        try:
            return deepcopy(self.devices[1][name])
        except KeyError:
            return None

    def get_actuator_name_by_address(self, address):
        for actuator in self.devices[1].values():
            if actuator['address'] == address:
                return actuator['name']
        return None

    def add_sensor_reading(self, name, value, metadata, timestamp):
        try:
            sensor = self.devices[0][name]
        except KeyError:
            return False
        sensor['data'].add((timestamp, value))
        if sensor['data'][-1][0] == timestamp:
            sensor['metadata'] = metadata
            sensor['last_seen'] = (datetime.date.today(), time.monotonic())
        return True
    
    def add_actuator_update(self, name, state, metadata, timestamp):
        try:
            actuator = self.devices[1][name]
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
            self.devices[1][name]['is_online'] = False
            return True
        except KeyError:
            return False

    def get_sensors_summary(self):
        summary = []
        for sensor_name, sensor_data in self.devices[0].items():
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
        return [deepcopy(actuator) for actuator in self.devices[1].values()]
