#!/usr/bin/env python

##  @package plot_tool
#   This tool is offers a ROS service that draws 2D parametric plots
#   The tool currently supports geometry_msgs::Pose and nav_msgs::Path
#   The plotting is done using pyqtgraph v0.9.8

import logging
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import signal
import attr

import cvra_rpc.message
import cvra_rpc.service_call
import threading
import socketserver

MASTER_BOARD_STREAM_ADDR = ('0.0.0.0', 20042)

# Plotting options
DEFAULT_PATH_SERIES = 0
DEFAULT_PATH_APPEND = True
DEFAULT_PATH_SYMBOL = '-'
DEFAULT_PATH_SYMBOL_SIZE = 4

DEFAULT_POSE_SERIES = 1
DEFAULT_POSE_APPEND = False
DEFAULT_POSE_SYMBOL = 's'
DEFAULT_POSE_SYMBOL_SIZE = 20

DEFAULT_TABLE_SERIES = 2
DEFAULT_TABLE_SYMBOL_SIZE = 4

@attr.s
class Point(object):
    x = attr.ib(default=attr.Factory(float))
    y = attr.ib(default=attr.Factory(float))
    z = attr.ib(default=attr.Factory(float))

@attr.s
class Quaternion(object):
    x = attr.ib(default=attr.Factory(float))
    y = attr.ib(default=attr.Factory(float))
    z = attr.ib(default=attr.Factory(float))
    w = attr.ib(default=attr.Factory(float))

@attr.s
class Pose(object):
    position = attr.ib(default=attr.Factory(Point))
    orientation = attr.ib(default=attr.Factory(Quaternion))

@attr.s
class PoseStamped(object):
    time = attr.ib(default=attr.Factory(float))
    pose = attr.ib(default=attr.Factory(Pose))

@attr.s
class Path(object):
    poses = attr.ib(default=attr.Factory(list))


#   This is a global graph object, we need it global so your service handlers can access the graph
graph_obj = None

##  Class CustomGraphicsWindow
#   We use this class to override the closeEvent function of the base pg.GraphicsWindow
#   This is used to reopen the window, and redraw all the content, whenever the graph window is closed.
class CustomGraphicsWindow(pg.GraphicsWindow):
    ##  Function closeEvent
    #   This overrides the closeEvent function in the base class. This function is invoked automatically by QTGui when the window closes.
    #   @param ev This is the event object (i.e. window close).
    def closeEvent(self,ev):
        # recreate the graph window
        graph_obj.win = CustomGraphicsWindow()
        graph_obj.win.setWindowTitle('Plot Tool')
        graph_obj.graph = graph_obj.win.addPlot()
        graph_obj.graph.showGrid(x=True, y=True)
        # iterate through the current plots, and readd them to the graph GUI
        for s in graph_obj.plot_tracker:
            for p in s:
                graph_obj.graph.addItem(p)

##  Class Graph_Drawer
#   This class manages all the pyqtgraph plotting and rendering.
#   If we wish to use another backend for drawing the graphs, this class should be swapped out.
class Graph_Drawer:
    # We currently support 7 different data series each with its own color and plot storage
    # We can easily add more if needed, however we may have to define the colors as (R,G,B)
    colors=['g','r','b','c','m','y','w']
    plot_tracker=[[] for i in range(7)]
    # This is the list that tracks pending items that are waiting to be drawn
    plot_queue=[]
    ##  Function run_loop
    #   This is the main loop of this ROS Node. It continuously processes incoming QT events (draw requests).
    #   It replaces the standard ROS spin function. We can only do this in Python, since rospy's spin is just a sleep.
    def run_loop(self):
        # Spawn the graph window
        self.win = CustomGraphicsWindow()
        self.win.setWindowTitle('Plot Tool')
        self.graph = self.win.addPlot()
        self.graph.showGrid(x=True, y=True)
        # This needs to be called to process QT events (render the window)
        QtGui.QApplication.instance().processEvents()
        # Main run loop while ROS is still going
        while True:
            # Check pending queue for new plot items
            while len(self.plot_queue) > 0:
                p = self.plot_queue.pop(0)
                self.plot(p.x_set, p.y_set, p.series, p.append, p.symbol, p.symbol_size)
                # This renders each plot item as soon as it is added
                # This causes a slow down if items are plotted at a high frequency (50Hz)
                # However, this approach does not miss any plot items
                # If we don't care about missing plot items, we can remove this
            #Process QT events so graph can still be manipulated when there are no plot requests
            QtGui.QApplication.instance().processEvents()
    ##  Function plot
    #   Takes Plot_Info items from the pending queue and creates a plot object that can be added to the graph
    #   @param x_set the list of x coordinates to be plotted
    #   @param y_set the list of y coordinates to be plotted
    #   @param series the series that the data belongs to
    #   @param append True: plot will be added to existing plots in the series. False: plot will replace all previous plots in the series.
    #   @param symbol Marker types: '-' for line plot, 'o' for circle, 's' for square, 't' for triangle, 'd' for diamond, '+' for cross
    #   @param symbol_size how big the markers will be (does not affect line plots)
    def plot(self,x_set,y_set,series,append,symbol,symbol_size):
        # Default settings will be applied if lacking information
        if series==None:
            logging.warn("plot_tool: series not declared.. setting series to 0")
            series = 0
        elif series < 0 or series > 6:
            logging.warn("plot_tool: series must be between 0 and 6.. setting series to 0")
            series = 0
        if append==None:
            logging.warn("plot_tool: append not declared.. setting append to true")
            append=True
        if len(x_set) > 0 and len(y_set)>0:
            if symbol==None:
                logging.warn("plot_tool: symbol not declared.. setting symbol to -")
                symbol='-'
            else:
                symbol=str(symbol)
                if symbol!='o' and symbol!='s' and symbol!='t' and symbol!='d' and symbol!='+' and symbol!='-':
                    logging.warn("plot_tool: symbol is not valid, should be one of { -,o,s,t,d,+ }.. setting symbol to -")
                    symbol='-'
            if symbol_size==None:
                logging.warn("plot_tool: symbol_size not declared.. setting symbol_size to 10")
                symbol_size=10
        # set plot color based on data series
        c = self.colors[series]
        # clear existing plots if append is False
        if append==False:
            plot_list = self.plot_tracker[series]
            for plot in plot_list:
                plot.clear()
            self.plot_tracker[series]=[]
        # note that we do not always need data, one way to clear plots is to send a request with append = False and no data
        if x_set!=None and y_set!=None:
            # line plots with 1 data point is automatically converted to a scatter plot
            if (len(x_set) == 1 or len(y_set) == 1) and symbol=='-':
                logging.warn("plot_tool: can't plot line with single point.. changing symbol to o")
                symbol='o'
            if symbol=='-':
                plot = self.graph.plot(x_set,y_set, pen=c)
            else:
                plot = self.graph.plot(x_set,y_set, pen=None, symbolPen=c, symbolBrush=c, symbolSize=symbol_size, symbol=symbol)
            # save the plot, so we can access it for redrawing or deleting later
            self.plot_tracker[series].append(plot)

##  Class Plot_Info
#   This class is just used to organize the plot request parameters into a tidy object
class Plot_Info:
    def __init__(self, x_set, y_set, series, append, symbol, symbol_size):
        self.x_set = x_set
        self.y_set = y_set
        self.series = series
        self.append = append
        self.symbol = symbol
        self.symbol_size = symbol_size

##  Function plot_path
#   This is the service handler for plotting nav_msgs::Path messages.
#   @param req this is the standard service request object in ROS
def plot_path(data):
    # Extract the plot data out of the nav_msgs::Path message
    # and organize it into a list of x coordinates and a list of y coordinates
    if data == None:
        x_set = None
        y_set = None
    else:
        pose_list = data.poses
        N=len(pose_list)
        i=0
        x_set=[]
        y_set=[]
        while i<N:
            if (pose_list[i].pose.position.x != None and pose_list[i].pose.position.y != None):
                x_set.append(pose_list[i].pose.position.x)
                y_set.append(pose_list[i].pose.position.y)
            i=i+1
    # Create a Plot_Info object with request parameters and add it to the pending queue (for drawing)
    graph_obj.plot_queue.append(Plot_Info(x_set,y_set,DEFAULT_PATH_SERIES,DEFAULT_PATH_APPEND,DEFAULT_PATH_SYMBOL,DEFAULT_PATH_SYMBOL_SIZE))

##  Function plot_pose
#   This is the service handler for plotting geometry_msgs::Pose messages.
#   @param req this is the standard service request object in ROS
def plot_pose(data):
    # Extract the plot data out of the geometry_msgs::Pose message.
    # and organize it into a list of x coordinates and a list of y coordinates
    if data == None:
        x_set = None
        y_set = None
    else:
        if (data.position.x != None and data.position.y != None):
            x_set=[data.position.x]
            y_set=[data.position.y]
    # Create a Plot_Info object with request parameters and add it to the pending queue (for drawing)
    graph_obj.plot_queue.append(Plot_Info(x_set,y_set,DEFAULT_POSE_SERIES,DEFAULT_POSE_APPEND,DEFAULT_POSE_SYMBOL,DEFAULT_POSE_SYMBOL_SIZE))

class DatagramRcv(QtCore.QThread):
    def __init__(self, remote):
        self.remote = remote
        super(DatagramRcv, self).__init__()

    def msg_cb(self, msg, args):
        # print(msg, args)
        if msg == 'position':
            pose = Pose(Point(args[0]/1000, args[1]/1000, 0), Quaternion(1, 0, 0, 0))
            plot_pose(pose)
        if msg == 'path':
            print('todo: path')

    def run(self):
        RequestHandler = cvra_rpc.message.create_request_handler({}, lambda todo, msg, args: self.msg_cb(msg, args))
        msg_server = socketserver.UDPServer(self.remote, RequestHandler)
        print('start CVRA rpc server')
        msg_server.serve_forever()


##  Function node_setup
#   This function sets up the ROS Node and registers the ROS services it provides
def main():
    rcv_thread = DatagramRcv(MASTER_BOARD_STREAM_ADDR)
    rcv_thread.start()

    # Create a global Graph_Drawer instance for everything to use/draw on
    # This is necessary because QTCore and QTGui does not like to be controlled from multiple threads
    # However, rospy spawns different threads to handle service requests and callbacks
    # To avoid rendering issues, we are using the queue system in Graph_Drawer for plot requests
    global graph_obj
    graph_obj = Graph_Drawer()

    signal.signal(signal.SIGINT, signal.SIG_DFL)  # to kill on ctl-C

    path = Path([
                    PoseStamped(1, Pose(Point(1, 1, 0), Quaternion(1, 0, 0, 0))),
                    PoseStamped(1, Pose(Point(2, 1, 0), Quaternion(1, 0, 0, 0))),
                    PoseStamped(1, Pose(Point(2, 2, 0), Quaternion(1, 0, 0, 0))),
                    PoseStamped(1, Pose(Point(3, 2, 0), Quaternion(1, 0, 0, 0))),
                ])
    pose = Pose(Point(1.5, 1, 0), Quaternion(1, 0, 0, 0))

    plot_path(path)
    plot_pose(pose)
    # draw table
    graph_obj.plot_queue.append(Plot_Info([0,3,3,0,0],[0,0,2,2,0],DEFAULT_TABLE_SERIES,False,'-',DEFAULT_TABLE_SYMBOL_SIZE))

    # Start the ROS Node and register the services offered
    # Kick off the main run loop in the Graph_Drawer instance
    graph_obj.run_loop()

#   Point of entry into the program
if __name__ == '__main__':
    main()
