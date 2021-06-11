from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
# from random import randint
import numpy as np
import serial


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        # Required initialisation of parent class
        super(MainWindow, self).__init__(*args, **kwargs)

        self.pressureGraphWidget = pg.PlotWidget(title="Pressure")
        self.vertAccelGraphWidget = pg.PlotWidget(title="Vertical Acceleration")
        self.pressureGraphWidget.showGrid(x=True,y=True)
        self.vertAccelGraphWidget.showGrid(x=True,y=True)

        self.group = QtWidgets.QGroupBox()
        group_layout = QtWidgets.QHBoxLayout()

        self.buttons = QtWidgets.QGroupBox()
        button_layout = QtWidgets.QVBoxLayout()

        button1 = QtWidgets.QPushButton('OPEN SERIAL')
        button2 = QtWidgets.QPushButton('START')
        button3 = QtWidgets.QPushButton('STOP')

        button1.clicked.connect(self.button1_clicked)
        button2.clicked.connect(self.button2_clicked)
        button3.clicked.connect(self.button3_clicked)

        button_layout.addWidget(button1)
        button_layout.addWidget(button2)
        button_layout.addWidget(button3)

        self.buttons.setLayout(button_layout)

        group_layout.addWidget(self.pressureGraphWidget)
        group_layout.addWidget(self.vertAccelGraphWidget)
        group_layout.addWidget(self.buttons)
        self.group.setLayout(group_layout)

        # Add widget to hold the graph
        self.setCentralWidget(self.group)

        # Create arrays to hold graph data
        self.start = 0
        self.end = 2*np.pi
        self.num_points = 1000
        # 100 time points
        self.x = np.linspace(self.start, self.end, self.num_points)

        self.amplitude = 1
        self.y = self.amplitude*np.sin(self.x)  # 100 data points

        self.pressureGraphWidget.setBackground('w')
        self.vertAccelGraphWidget.setBackground('w')

        # Define line colour and style
        pen = pg.mkPen(color=(255, 0, 0))
        self.pressure_data = self.pressureGraphWidget.plot(self.x, self.y, pen=pen)
        self.vert_accel_data = self.vertAccelGraphWidget.plot(self.x, self.y, pen=pen)


        # Create a timer and link the function to the time
        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        #baro pressure
        #vert accel
        #save all incoming to csv

        # Update x
        self.start += 0.1
        self.end += 0.1
        self.x = np.linspace(self.start, self.end, self.num_points)

        # Recalc y
        self.y = self.amplitude*np.sin(self.x)

        self.pressure_data.setData(self.x, self.y)  # Update the data.

    def button1_clicked(self):
        #open serial link
        self.ser = serial.Serial('/dev/ttyS1', 57600, timeout=1)     
        
    def button2_clicked(self):
        #start reading
        self.stop = False
        while not self.stop:
            ser.readline()

    def button3_clicked(self):
        #stop reading
        self.stop = True


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())
