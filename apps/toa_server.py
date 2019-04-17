#!/usr/bin/env python3

import argparse
import config_file_parser
import signal
import sys
import numpy as np
import apsw
import datetime
from collections import deque
import zmq
import pickle
import map_helpers
from mpl_toolkits.basemap import Basemap
from zmq_manager import zmq_manager
import chan_ho_algorithm
from threading import Lock
import copy
import math

class toa_server():
    def __init__(self, cfg, args):
        self.args = args

        # Read from config file
        self.reference_rx_id = cfg["common"]["reference_rx_id"]
        self.beacon_id = cfg["common"]["beacon_id"]
        self.tag_ids = cfg["common"]["tag_ids"]
        self.bbox_map = cfg["common"]["bbox_map"]
        self.geo_coordinates_rx = dict(cfg["common"]["geo_coordinates_rx"])
        self.geo_coordinates_beacon = cfg["common"]["geo_coordinates_beacon"]
        self.const_c = cfg["common"]["const_c"]
        self.rx_sample_rate = cfg["receiver"]["sample_rate"]
        self.zmq_rx_data_port_base = cfg["receiver"]["zmq_rx_data_port_base"]
        self.zmq_server_port_toads = cfg["server"]["zmq_server_port_toads"]
        self.zmq_server_port_positions = cfg["server"]["zmq_server_port_positions"]
        self.record_to_database = cfg["server"]["record_to_database"]
        self.database = cfg["server"]["database"]
        self.toa_buffer_len = cfg["server"]["toa_buffer_len"]

        # Initialize clock correction algorithm
        self.beta = dict()
        self.phi = dict()
        for idx in self.geo_coordinates_rx:
            self.beta[idx] = 1.0
            self.phi[idx] = 0.0

        # ZeroMQ connection (to receivers)
        self.zmq_manager = zmq_manager()
        for rx_id in self.geo_coordinates_rx:
            self.zmq_manager.add_socket(rx_id, \
                "tcp://localhost:"+ str(self.zmq_rx_data_port_base + rx_id), self.rx_callback)

        self.mutex = Lock()

        # ZeroMQ publishers for TOADs and positions
        self.zmq_context = zmq.Context()
        self.socket_toads = self.zmq_context.socket(zmq.PUB)
        self.socket_toads.bind("tcp://*:" + str(self.zmq_server_port_toads))
        self.socket_positions = self.zmq_context.socket(zmq.PUB)
        self.socket_positions.bind("tcp://*:" + str(self.zmq_server_port_positions))

        # Buffer for TOAs of each tag/receiver combination
        self.toa_buffer = dict()
        for tag_id in self.tag_ids:
            self.toa_buffer[tag_id] = dict()
            for rx_id in self.geo_coordinates_rx:
                self.toa_buffer[tag_id][rx_id] = deque()
        
        # Dictionaries for TOAs and TOA differences (TOADs)
        self.toas = dict()
        self.toas_old = dict()
        self.toads = dict()
        for tag_id in self.tag_ids:
            self.toas[tag_id] = dict()
            self.toas_old[tag_id] = dict()
            if self.beacon_id != tag_id:
                self.toads[tag_id] = dict()

        # Get reference UTM grid
        lon_0, lat_0 = map_helpers.get_utm_lon_lat_0(self.bbox_map[0], self.bbox_map[1])

        # Set basemap
        self.basemap = Basemap(llcrnrlon=self.bbox_map[0], llcrnrlat=self.bbox_map[1],
                        urcrnrlon=self.bbox_map[2], urcrnrlat=self.bbox_map[3],
                        projection='tmerc', lon_0=lon_0, lat_0=lat_0)
        
        # Dictionaries for positions
        self.pos_tags = dict()
        self.pos_rx = dict()
        xy_sum = [0, 0]
        for rx_id in self.geo_coordinates_rx:
            lon = self.geo_coordinates_rx[rx_id][0]
            lat = self.geo_coordinates_rx[rx_id][1]
            xy = list(self.basemap(lon, lat))
            self.pos_rx[rx_id] = xy
            xy_sum[0] += xy[0]
            xy_sum[1] += xy[1]

        # Calculate sensor to beacon time of flights
        self.tofs_sensors_beacon = dict()
        x_b, y_b = self.basemap(self.geo_coordinates_beacon[0], self.geo_coordinates_beacon[1])
        for rx_id in self.geo_coordinates_rx:
            x_s, y_s = self.basemap(self.geo_coordinates_rx[rx_id][0], self.geo_coordinates_rx[rx_id][1])
            self.tofs_sensors_beacon[rx_id] = math.sqrt((x_b-x_s)**2+(y_b-y_s)**2) / self.const_c
        
        if args.record_to_database:
            # Database connection (for results)
            print("Connect to database:", self.database)
            self.db_connection = apsw.Connection(self.database)
            self.db_connection.setbusyhandler(self.db_busy_handler)
            self.db_cursor = self.db_connection.cursor()
            #self.db_cursor.execute("pragma journal_mode=wal")
            self.db_cursor.execute("create table if not exists " \
                "TOAs(timestamp, rx_id, tag_id, toa_ns, correlation, databit)")

        # Start regular watcher thread to get data from receivers
        self.zmq_manager.start_watcher(100)

    def sigint_handler(self, sig, frame):
        self.zmq_manager.stop_watcher()
        sys.exit(0)

    def db_busy_handler(self, tries):
        # Always keep retrying
        return True

    def rx_callback(self, rx_id, msg_data):
        # Get UTC timestamp
        timestamp = int(datetime.datetime.utcnow().strftime("%s"))
        # Deserialize data
        # tag id | TOA nanoseconds | corrleation peak value | databit
        tag_id = np.frombuffer(msg_data[0:2], dtype=np.uint16)[0].item()
        toa_ns = np.frombuffer(msg_data[2:10], dtype=np.float64)[0].item()
        correlation = np.frombuffer(msg_data[10:14], dtype=np.float32)[0].item()
        databit = np.frombuffer(msg_data[14:15], dtype=np.uint8)[0].item()
#        print("Received [timestamp, rx_id, tag_id, TOA nanoseconds, correlation, databit]:", \
#            timestamp, rx_id ,tag_id, toa_ns, correlation, databit)
        if args.record_to_database:
            # Insert into database
            self.db_cursor.execute("insert into TOAs values(?,?,?,?,?,?)", \
                (timestamp, rx_id, tag_id, toa_ns, correlation, databit))
        
        self.update_toa_buffer(tag_id, rx_id, (timestamp, toa_ns, abs(correlation), databit))

    def update_toa_buffer(self, tag_id, rx_id, toa):
        self.mutex.acquire()
        self.toa_buffer[tag_id][rx_id].appendleft(toa)
        if self.toa_buffer_len < len(self.toa_buffer[tag_id][rx_id]):
            self.toa_buffer[tag_id][rx_id].pop()
        self.mutex.release()
        if 0 == tag_id and 0 == rx_id:
            self.update_results()

    def update_results(self):
        self.mutex.acquire()
        self.update_latest_associated_toas()
        self.update_toads()
        self.update_positions()
        self.mutex.release()
        self.socket_toads.send(pickle.dumps(self.toads))
        self.socket_positions.send(pickle.dumps(self.pos_tags))

    # To associate the received transmissions of the same tag use timestamp and databit
    def update_latest_associated_toas(self):
        latest = 0
        previous = 1
        idx_timestamp = 0
        idx_toa = 1
        idx_databit = 3
        for tag_id in self.tag_ids:
            # Make sure to clear old results since number of available TOAs can change
            self.toas_old[tag_id] = copy.deepcopy(self.toas[tag_id])
            self.toas[tag_id].clear()
            # First find latest timestamp
            max_timestamp = 0
            association_databit = 0
            for rx_id in self.geo_coordinates_rx:
                # We need at least two beacon receptions from the sensor
                if 2 <= len(self.toa_buffer[tag_id][rx_id]):
                    if max_timestamp < self.toa_buffer[tag_id][rx_id][previous][idx_timestamp]:
                        max_timestamp = self.toa_buffer[tag_id][rx_id][previous][idx_timestamp]
                        association_databit = self.toa_buffer[tag_id][rx_id][previous][idx_databit]
            # Then perform association
            for rx_id in self.geo_coordinates_rx:
                # We need at least two tag receptions from the sensor
                if 2 <= len(self.toa_buffer[tag_id][rx_id]):
                    # Check if the latest or the second latest buffer elements match
                    # At most an offset of 1 s is permissible in timestamps
                    if 1 >= abs(max_timestamp - self.toa_buffer[tag_id][rx_id][previous][idx_timestamp]) \
                        and association_databit == self.toa_buffer[tag_id][rx_id][previous][idx_databit]:
                            self.toas[tag_id][rx_id] = self.toa_buffer[tag_id][rx_id][previous][idx_toa]
                    else:
                        if 1 >= abs(max_timestamp - self.toa_buffer[tag_id][rx_id][latest][idx_timestamp]) \
                            and association_databit == self.toa_buffer[tag_id][rx_id][latest][idx_databit]:
                                self.toas[tag_id][rx_id] = self.toa_buffer[tag_id][rx_id][latest][idx_toa]

        print("TOAs:",self.toas)

    def update_toads(self):
        # tag_id 0 is a stationary beacons
        # Use clock error correction method 3) from
        # Saeed Shojaee, Johannes Schmitz, Sivan Toledo, Rudolf Mathar:
        #   On the Accuracy of Passive Hyperbolic Localization in the Presence of Clock Drift
        # (time index of deque inverted compared to paper)
        if self.reference_rx_id in self.toas_old[self.beacon_id]:
            for rx_id in self.geo_coordinates_rx:
                if rx_id in self.toas[self.beacon_id] \
                    and rx_id in self.toas_old[self.beacon_id] \
                    and self.reference_rx_id in self.toas[self.beacon_id]:
                    if self.reference_rx_id != rx_id:
                        # Equation (13)
                        beta_new = ( self.toas[self.beacon_id][rx_id] \
                            - self.toas_old[self.beacon_id][rx_id] ) \
                            / ( self.toas[self.beacon_id][self.reference_rx_id] \
                            - self.toas_old[self.beacon_id][self.reference_rx_id] )
                        if 0.0001 > abs(1.0 - beta_new):
                            self.beta[rx_id] = beta_new
            
                if rx_id in self.toas[self.beacon_id]:
                    # Equation (14)
                    self.phi[rx_id] = self.toas[self.beacon_id][rx_id] \
                        - self.beta[rx_id] * self.tofs_sensors_beacon[rx_id]
            print("beta", self.beta)
            print("phi", self.phi)
            for tag_id in self.tag_ids:
                for rx_id in self.geo_coordinates_rx:
                    if self.reference_rx_id != rx_id and self.beacon_id != tag_id:
                        if self.reference_rx_id in self.toas[tag_id] and rx_id in self.toas[tag_id]:
                            # Equation (15)
                            # self.toads[tag_id][rx_id] = self.toas[tag_id][rx_id] \
                            #     - self.toas[tag_id][self.reference_rx_id]
                            self.toads[tag_id][rx_id] = ( self.toas[tag_id][rx_id] \
                                - self.phi[rx_id] ) / self.beta[rx_id] \
                                - ( self.toas[tag_id][self.reference_rx_id] \
                                - self.phi[self.reference_rx_id] )

    def update_positions(self):
        for tag_id in self.tag_ids:
            if self.beacon_id != tag_id:
                if 2 <= len(self.toads[tag_id]):
                    d = list()
                    for toad_idx in self.toads[tag_id]:
                        d.append(self.toads[tag_id][toad_idx] * self.const_c)
                    print("d",d)
                    chan_result = chan_ho_algorithm.locate(list(self.pos_rx.values()), d)
                    self.pos_tags[tag_id] = chan_result


###################################################################################################

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--database", type=str, default="toa_localization_db.sqlite",
                        help="Name of the database with TOAs and localization results.")
    parser.add_argument("-r", "--record-to-database", action="store_true",
                        help="Activate recording of TOAs and results to database.")
    return parser.parse_args()

if __name__ == "__main__":
    cfg = config_file_parser.parse_config_file("toa_config.ini")
    args = parse_args()
    server = toa_server(cfg, args)
    signal.signal(signal.SIGINT, server.sigint_handler)
    signal.pause()