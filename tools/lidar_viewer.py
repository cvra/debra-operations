import zmqmsgbus


from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

import numpy as np


# PyQtGraph stuff
app = QtGui.QApplication([])
pg.setConfigOptions(antialias=True)
plot = pg.plot(title='Lidar Polar Plot')
plot.resize(600,600)
plot.setAspectLocked()

def PlotPolar():
    # Add polar grid lines
    plot.addLine(x=0, pen=0.2)
    plot.addLine(y=0, pen=0.2)

    radius = np.arange(0.2, 3, 0.2);
    for r in radius:
        circle = pg.QtGui.QGraphicsEllipseItem(-r, -r, r*2, r*2)
        circle.setPen(pg.mkPen(color=(30, 30, 30)))
        plot.addItem(circle)

    radius = np.arange(1, 3.1, 1);
    for r in radius:
        circle = pg.QtGui.QGraphicsEllipseItem(-r, -r, r*2, r*2)
        circle.setPen(pg.mkPen(color=(60, 60, 60)))
        plot.addItem(circle)
        plot.setXRange(-3, 3)
        plot.setYRange(-3, 3)



# zmqmsgbus subscribe
bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                    pub_addr='ipc://ipc/sink')

node = zmqmsgbus.Node(bus)


last_scan = node.recv('/lidar/scan')

def update_scan(topic, message):
    # print(message)
    global last_scan
    last_scan = message

node.register_message_handler('/lidar/scan', update_scan)

print('receiving')
while 1:
    # datagram = node.recv('/lidar/scan')
    datagram = last_scan
    radius = datagram['Data']

    theta = np.linspace(-0.7853981634, 3.926990817, len(radius)) # -45° - 225°

    # Transform to cartesian and plot
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)

    cloudPts = np.array([x, y]).T;

    plot.clear()
    PlotPolar()

    # draw raw measure
    linePen = pg.mkPen(color=(200, 200, 200, 200), width= 2, style=QtCore.Qt.DotLine)
    plot.plot(x, y, pen=linePen,  symbol='o', symbolPen=None, symbolSize=7, symbolBrush=(255, 234, 0, 160))

    app.processEvents()
