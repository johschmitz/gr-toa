#!/usr/bin/env python3

import argparse
from toa import config_file_parser, map_helpers
import signal
import sys
import zmq
from apscheduler.schedulers.background import BackgroundScheduler
from mpl_toolkits.basemap import Basemap
import numpy as np
import math


class toa_receivers_test():
    def __init__(self, cfg):
        
        # Settings from config file
        self.bbox_map = cfg["common"]["bbox_map"]
        self.geo_coordinates_rx = dict(cfg["common"]["geo_coordinates_rx"])
        self.geo_coordinates_beacon = cfg["common"]["geo_coordinates_beacon"]
        self.reference_rx_id = cfg["common"]["reference_rx_id"]
        self.const_c = cfg["common"]["const_c"]
        self.tag_ids = cfg["common"]["tag_ids"]
        self.beacon_id = cfg["common"]["beacon_id"]
        self.zmq_rx_data_port_base = cfg["receiver"]["zmq_rx_data_port_base"]
        self.test_geo_coordinates_tags = cfg["test"]["test_geo_coordinates_tags"]
        self.test_tx_interval_seconds = cfg["test"]["test_tx_interval_seconds"]
        self.test_num_tx_intervals = cfg["test"]["test_num_tx_intervals"]
        self.test_toa_noise_sigma = cfg["test"]["test_toa_noise_sigma"]

        # Get reference UTM grid
        lon_0, lat_0 = map_helpers.get_utm_lon_lat_0(self.bbox_map[0], self.bbox_map[1])

        # Set basemap
        self.basemap = Basemap(llcrnrlon=self.bbox_map[0], llcrnrlat=self.bbox_map[1],
                        urcrnrlon=self.bbox_map[2], urcrnrlat=self.bbox_map[3],
                        projection='tmerc', lon_0=lon_0, lat_0=lat_0)

        self.pos_rx = dict()
        self.pos_tags = dict()
        for rx_id in self.geo_coordinates_rx:
            lon = self.geo_coordinates_rx[rx_id][0]
            lat = self.geo_coordinates_rx[rx_id][1]
            x, y = self.basemap(lon, lat)
            self.pos_rx[rx_id] = list(self.basemap(lon, lat))
        # Treat beacon as additional tag
        self.test_geo_coordinates_tags[self.beacon_id] = self.geo_coordinates_beacon
        for tag_id in self.tag_ids:
                lon = self.test_geo_coordinates_tags[tag_id][0]
                lat = self.test_geo_coordinates_tags[tag_id][1]
                self.pos_tags[tag_id] = list(self.basemap(lon, lat))

        # Initialize receiver datastructures
        self.phis = dict()
        self.epsilons = dict()
        self.clocks = dict()
        for rx_id in self.geo_coordinates_rx:
            self.phis[rx_id] = rx_id
            if self.reference_rx_id == rx_id:
                self.epsilons[rx_id] = 1.0
            else:
                self.epsilons[rx_id] = 1.00001
            # Initialize clocks with offsets
            self.clocks[rx_id] = self.phis[rx_id]
        print("phis:",self.phis)
        print("epsilons:",self.epsilons)

        self.toas = dict()
        self.correlations = dict()
        self.databits = dict()
        for tag_id in self.tag_ids:
            self.toas[tag_id] = dict()
            self.correlations[tag_id] = dict()
            self.databits[tag_id] = 0
            for rx_id in self.geo_coordinates_rx:
                # Set corrlation value to 1
                self.correlations[tag_id][rx_id] = 1

        # ZeroMQ publishers for TOAs
        self.zmq_context = zmq.Context()
        self.socket_toas = dict()
        for rx_id in self.geo_coordinates_rx:
            self.socket_toas[rx_id] = self.zmq_context.socket(zmq.PUB)
            self.socket_toas[rx_id].bind("tcp://*:" + str(self.zmq_rx_data_port_base + rx_id))

        # Configure scheduler
        self.scheduler = BackgroundScheduler()
        self.scheduler.configure(timezone="UTC")
        self.scheduler.add_job(self.send_toas, 'interval', seconds=self.test_tx_interval_seconds, id="toa_sender")
        self.scheduler_count = 0
        self.scheduler.start()

    def sigint_handler(self, sig, frame):
        sys.exit(0)

    def send_toas(self):
        self.update_clocks()
        self.update_databits()
        self.update_positions()
        self.update_toas()
        for tag_id in self.tag_ids:
            for rx_id in self.geo_coordinates_rx:
                timestamp = self.clocks[rx_id] - self.phis[rx_id]
                messagedata = np.array(tag_id, dtype=np.uint16).tostring() \
                    + np.array(timestamp, dtype=np.float64).tostring() \
                    + np.array(self.toas[tag_id][rx_id], dtype=np.float64).tostring() \
                    + np.array(self.correlations[tag_id][rx_id], dtype=np.float32).tostring() \
                    + np.array(self.databits[tag_id], dtype=np.uint8).tostring()
                self.socket_toas[rx_id].send(messagedata)
        self.scheduler_count += 1
        if self.test_num_tx_intervals == self.scheduler_count:
            self.scheduler.remove_job("toa_sender")

    def update_clocks(self):
        for rx_id in self.geo_coordinates_rx:
            self.clocks[rx_id] += self.epsilons[rx_id] * self.test_tx_interval_seconds

    def update_databits(self):
        for tag_id in self.tag_ids:
            # Toggle databit
            self.databits[tag_id] ^= 1

    def update_positions(self):
        for tag_id in self.tag_ids:
            if self.beacon_id != tag_id:
                self.pos_tags[tag_id][0] += 2
                self.pos_tags[tag_id][1] += 4
                print("Tag position:", self.pos_tags[tag_id])

    def update_toas(self):
        # Calculate sensor to beacon time of flights
        for tag_id in self.tag_ids:
            for rx_id in self.geo_coordinates_rx:
                self.toas[tag_id][rx_id] = self.clocks[rx_id] \
                    + math.sqrt( (self.pos_tags[tag_id][0] - self.pos_rx[rx_id][0])**2 \
                    + ( self.pos_tags[tag_id][1] - self.pos_rx[rx_id][1])**2 ) / self.const_c \
                    + np.random.normal(0,self.test_toa_noise_sigma)
        print("self.toas",self.toas)


if __name__ == "__main__":
    cfg = config_file_parser.parse_config_file("toa_config_test.ini")
    receivers = toa_receivers_test(cfg)
    signal.signal(signal.SIGINT, receivers.sigint_handler)
    signal.pause()
    pass