#!/usr/bin/env python3

###############################################################################
# Imports
###############################################################################
import argparse
import sys
import os
import signal
from optparse import OptionParser
from PyQt4 import Qt, QtGui, QtCore, uic
import qwt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg
from mpl_toolkits.basemap import Basemap
from PIL import Image, PngImagePlugin
import apsw
from toa import config_file_parser, gui_helpers, map_helpers, osm_tile_download
from toa.zmq_manager import zmq_manager
import pickle
from collections import deque


class gui(QtGui.QMainWindow):
    def __init__(self, cfg, args, parent=None):
        self.args = args

        # Settings from config file
        self.bbox_map = cfg["common"]["bbox_map"]
        self.geo_coordinates_rx = cfg["common"]["geo_coordinates_rx"]
        self.geo_coordinates_beacon = cfg["common"]["geo_coordinates_beacon"]
        self.tag_ids = cfg["common"]["tag_ids"]
        self.beacon_id = cfg["common"]["beacon_id"]
        self.reference_rx_id = cfg["common"]["reference_rx_id"]
        self.zmq_server_port_toads = cfg["server"]["zmq_server_port_toads"]
        self.zmq_server_port_positions = cfg["server"]["zmq_server_port_positions"]
        self.toads_history_len = cfg["gui"]["toads_history_len"]
        self.locations_history_len = cfg["gui"]["locations_history_len"]

        QtGui.QMainWindow.__init__(self, parent)

        # Timer for plot updates
        self.update_timer = Qt.QTimer()

        # Give Ctrl+C back to system
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.gui = uic.loadUi(os.path.join(os.path.dirname(__file__),'../gui/toa_localization_gui.ui'), self)
        self.gui.setWindowTitle("TOA Localization GUI")

        # Map plot configuration
        self.tab_map = 0
        self.set_gui_bbox_map()
        self.locations_history = dict()
        for tag_id in self.tag_ids:
            if self.beacon_id != tag_id:
                self.locations_history[tag_id] = deque()
        self.figure_map = plt.figure()
        self.canvas = FigureCanvasQTAgg(self.figure_map)
        self.toolbar = gui_helpers.NavigationToolbar2QT(self.canvas, self)
        self.verticalLayoutMap.addWidget(self.toolbar)
        self.verticalLayoutMap.addWidget(self.canvas)
        # Create plot for basemap
        self.map_axes = self.figure_map.add_subplot(111)
        self.map_axes.axis('off')
        self.set_bbox_map()
        # Dictionary for tag markers
        self.tag_markers = dict()

        # TOA differences plot configuration
        self.tab_toads = 1
        self.toads_history = dict()
        for tag_id in self.tag_ids:
            self.toads_history[tag_id] = dict()
            for rx_id in self.geo_coordinates_rx:
                if self.beacon_id != tag_id and self.reference_rx_id != rx_id:
                    self.toads_history[tag_id][rx_id] = deque()
        pen = Qt.QPen(Qt.Qt.DotLine)
        pen.setColor(Qt.Qt.black)
        pen.setWidth(0)
        grid = qwt.QwtPlotGrid()
        grid.setPen(pen)
        grid.attach(self.gui.qwtPlotTOADs)
        title = qwt.QwtText("Samples")
        title.setFont(Qt.QFont("Helvetica", 14, Qt.QFont.Bold))
        self.gui.qwtPlotTOADs.setAxisTitle(qwt.QwtPlot.xBottom, title)
        title = qwt.QwtText("Time of arrival difference [s]")
        title.setFont(Qt.QFont("Helvetica", 10, Qt.QFont.Bold))
        self.gui.qwtPlotTOADs.setAxisTitle(qwt.QwtPlot.yLeft, title)
        self.gui.qwtPlotTOADs.insertLegend(qwt.QwtLegend(), qwt.QwtPlot.RightLegend)
        self.toad_curves = dict()
        for tag_id in self.tag_ids:
            self.toad_curves[tag_id] = dict()
            for rx_id in self.geo_coordinates_rx:
                if self.reference_rx_id != rx_id:
                    self.toad_curves[tag_id][rx_id] = qwt.QwtPlotCurve("TOAD_" \
                        + str(tag_id) + "_" + str(rx_id) + "_" + str(self.reference_rx_id))
                    self.toad_curves[tag_id][rx_id].setPen(Qt.QPen(Qt.QColor(100*rx_id,0,100*tag_id),2,Qt.Qt.SolidLine))
                    self.toad_curves[tag_id][rx_id].attach(self.gui.qwtPlotTOADs)

        # ZeroMQ connection (to server)
        self.zmq_manager = zmq_manager()
        self.zmq_manager.add_socket("TOADs", \
                "tcp://localhost:"+ str(self.zmq_server_port_toads), self.toads_callback)
        self.zmq_manager.add_socket("locations", \
                "tcp://localhost:"+ str(self.zmq_server_port_positions), self.locations_callback)

        # Qt Signals
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.zmq_manager.poll_socket)
        self.connect(self.gui.pushButtonSetBbox, QtCore.SIGNAL("clicked()"), self.set_bbox_map)
        self.shortcut_start = QtGui.QShortcut(Qt.QKeySequence("Ctrl+S"), self.gui)
        self.shortcut_stop = QtGui.QShortcut(Qt.QKeySequence("Ctrl+C"), self.gui)
        self.shortcut_exit = QtGui.QShortcut(Qt.QKeySequence("Ctrl+D"), self.gui)
        self.connect(self.shortcut_exit, QtCore.SIGNAL("activated()"), self.gui.close)

        # Update with 2 Hz
        self.update_timer.start(500)

    def toads_callback(self,socket_id,data):
        toads = pickle.loads(data)
        print("TOADs received:", toads)
        for tag_id in toads:
            for rx_id in toads[tag_id]:
                self.toads_history[tag_id][rx_id].appendleft(toads[tag_id][rx_id])
                if self.toads_history_len < len(self.toads_history[tag_id][rx_id]):
                    self.toads_history[tag_id][rx_id].pop()
        # Only update if the tab is selected
        if self.tab_toads == self.tabWidget.currentIndex():
            self.update_toad_plot()

    def locations_callback(self,socket_id,data):
        locations = pickle.loads(data)
        print("Positions received:", locations)
        for tag_id in locations:
            self.locations_history[tag_id].appendleft(locations[tag_id])
            if self.locations_history_len < len(self.locations_history[tag_id]):
                self.locations_history[tag_id].pop()
        # Only update if the tab is selected
        if self.tab_map == self.tabWidget.currentIndex():
            self.update_locations_plot()

    def update_toad_plot(self):
        for tag_id in self.toads_history:
            for rx_id in self.toads_history[tag_id]:
                if 0 < len(self.toads_history[tag_id]):
                    x = list(range(0,len(self.toads_history[tag_id][rx_id]),1))
                    y = list(self.toads_history[tag_id][rx_id])
                    self.toad_curves[tag_id][rx_id].setData(x, y)
                    self.gui.qwtPlotTOADs.replot()

    def update_locations_plot(self):
        for tag_id in self.locations_history:
            if 1 <= len(self.locations_history[tag_id]):
                x = self.locations_history[tag_id][0][0]
                y = self.locations_history[tag_id][0][1]
                if tag_id in self.tag_markers:
                    self.tag_markers[tag_id].remove()
                self.tag_markers[tag_id] = plt.text(x, y, str(tag_id), size=8, ha="center", va="center",
                    bbox=dict(boxstyle="round", ec=(0.25, 0.5, 1.0), fc=(0.4, 0.8, 1.0),) )
                self.canvas.draw()

    def set_bbox_map(self):
        self.bbox_map = [float(self.gui.lineEditLeft.text()),float(self.gui.lineEditBottom.text()),
            float(self.gui.lineEditRight.text()),float(self.gui.lineEditTop.text())]
        if self.bbox_map[0] > self.bbox_map[2] or self.bbox_map[1] > self.bbox_map[3]:
            print("Error: invalid bounding box")
        self.set_map()

    def set_gui_bbox_map(self):
        self.gui.lineEditLeft.setText(str(self.bbox_map[0]))
        self.gui.lineEditLeft.setCursorPosition(0)
        self.gui.lineEditBottom.setText(str(self.bbox_map[1]))
        self.gui.lineEditBottom.setCursorPosition(0)
        self.gui.lineEditRight.setText(str(self.bbox_map[2]))
        self.gui.lineEditRight.setCursorPosition(0)
        self.gui.lineEditTop.setText(str(self.bbox_map[3]))
        self.gui.lineEditTop.setCursorPosition(0)

    def set_map(self):
        # Get figure size in pixels for zoom level calculation
        figure_size = self.figure_map.get_size_inches()*self.figure_map.dpi
        # If necessary create maps directory
        if not os.path.exists("./maps"):
            os.mkdir("./maps")
        # Search for existing map with this bounding box in cache
        if not any(i.find("map_"+"+".join(str(j).replace(".",",") for j in self.bbox_map)+".png")!= -1 for i in os.listdir("./maps/") ):
            print("Download online map with bounding box", self.bbox_map)
            lat_deg = self.bbox_map[1]
            lon_deg = self.bbox_map[0]
            delta_lat = self.bbox_map[3]-self.bbox_map[1]
            delta_lon = self.bbox_map[2]-self.bbox_map[0]
            try:
                img = osm_tile_download.get_image_cluster(lat_deg, lon_deg, delta_lat, delta_lon, figure_size[0], figure_size[1])
                if None == img:
                    raise ValueError
                # Add bbox_map as meta data
                meta = PngImagePlugin.PngInfo()
                meta.add_text("bbox_map", str(self.bbox_map))
                img.save("./maps/map_"+"+".join(str(i).replace(".",",") for i in self.bbox_map)+".png", "png", pnginfo=meta)
            except:
                print("Error: Map download failed.")
                return

        else:
            # If available, open offline map instead
            print("Using cached offline map with bounding box", self.bbox_map)
            try:
                img = Image.open("./maps/map_"+"+".join(str(i).replace(".",",") for i in self.bbox_map)+".png")
            except:
                print("Error: couldn't load map file.")
                return
            bbox_map_meta = eval(img.info["bbox_map"])
            if list(bbox_map_meta) != self.bbox_map:
                print("Error: bounding boxes in meta data and filename do not agree")

        # Get reference UTM grid
        lon_0, lat_0 = map_helpers.get_utm_lon_lat_0(self.bbox_map[0], self.bbox_map[1])

        # Set basemap
        self.basemap = Basemap(llcrnrlon=self.bbox_map[0], llcrnrlat=self.bbox_map[1],
                        urcrnrlon=self.bbox_map[2], urcrnrlat=self.bbox_map[3],
                        projection='tmerc', ax=self.map_axes, lon_0=lon_0, lat_0=lat_0)

        # Add map image to basemap
        self.basemap.imshow(img, interpolation='lanczos', origin='upper', zorder=0)

        # Improve the GUI look
        self.figure_map.tight_layout(pad=0)

        # Activate zooming and panning with the mouse
        self.zp = gui_helpers.canvas_zoom_pan()
        self.zp.zoom_factory(self.map_axes, base_scale = 1.5)
        self.zp.pan_factory(self.map_axes)

        # Draw the map
        self.canvas.draw()
        
        # Plot sensors on the map
        self.plot_sensor_locations()

    def plot_sensor_locations(self):
        # Basemap needs to exist for plotting
        if hasattr(self, "basemap"):
            # Plot receivers (sensors)
            for rx_id in self.geo_coordinates_rx:
                lon = self.geo_coordinates_rx[rx_id][0]
                lat = self.geo_coordinates_rx[rx_id][1]
                x, y = self.basemap(lon, lat)
                print("Sensor %i located at x=%.2f, y=%.2f" % (rx_id,x,y))
                self.map_axes.scatter(x, y, linewidths=2,  marker="^", c="green", s=200, alpha=0.9, zorder=1)
                plt.text(x, y, str(rx_id), color="black", fontsize=8,
                    horizontalalignment="center", verticalalignment="top",)
            # Plot beacon
            lon = self.geo_coordinates_beacon[0]
            lat = self.geo_coordinates_beacon[1]
            x_b, y_b = self.basemap(lon, lat)
            self.map_axes.scatter(x_b, y_b, linewidths=2,  marker='^', c="orange", s=200, alpha=0.9, zorder=1)
            plt.text(x_b, y_b, "b", color="black", fontsize=8,
                horizontalalignment="center", verticalalignment="top",)
            self.canvas.draw()


###################################################################################################

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--config-file", type=str, default="",
                        help="Path to config file. Default: toa_config.ini")
    args = parser.parse_args()
    # Set default path for config file
    if 0 == len(args.config_file):
        args.config_file = os.path.dirname(sys.argv[0]) + "/toa_config.ini"
    return args

if __name__ == "__main__":
    args = parse_args()
    print("Using config file:", args.config_file)
    cfg = config_file_parser.parse_config_file(args.config_file)
    qapp = Qt.QApplication(sys.argv)
    qapp.main_window = gui(cfg,args)
    qapp.main_window.show()
    qapp.exec_()
