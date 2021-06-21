from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
# from random import randint
import numpy as np
import serial
import queue

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

        self.baroQ = queue.SimpleQueue()
        self.vertAccelQ = queue.SimpleQueue()

        self.pressureX = [0]
        self.pressureY = [0]
        self.vertAccelX = [0]
        self.vertAccelY = [0]

        self.pressureGraphWidget.setBackground('w')
        self.vertAccelGraphWidget.setBackground('w')

        # Define line colour and style
        pen = pg.mkPen(color=(255, 0, 0))
        self.pressure_data = self.pressureGraphWidget.plot(self.pressureX, self.pressureY, pen=pen)
        self.vert_accel_data = self.vertAccelGraphWidget.plot(self.vertAccelX, self.vertAccelY, pen=pen)

        # Create a timer and link the function to the time
        self.pressureTimer = QtCore.QTimer()
        self.pressureTimer.setInterval(50)
        self.pressureTimer.timeout.connect(self.update_pressure_plot_data)
        self.pressureTimer.start()

        self.vertAccelTimer = QtCore.QTimer()
        self.vertAccelTimer.setInterval(50)
        self.vertAccelTimer.timeout.connect(self.update_vert_accel_plot_data)
        self.vertAccelTimer.start()

        self.readTimer = QtCore.QTimer()
        self.readTimer.setInterval(10)
        self.readTimer.timeout.connect(self.read_data)

    def update_pressure_plot_data(self):
        #baro pressure
        #vert accel
        #save all incoming to csv
        while not self.baroQ.empty():
            try:
                baroReading = self.baroQ.get_nowait()
            except Empty as err:
                break
            baroReading = baroReading.split(",")
            self.pressureX.append(baroReading[0])
            self.pressureY.append(baroReading[4])

        self.pressure_data.setData(self.pressureX, self.pressureY)  # Update the data.

    def update_vert_accel_plot_data(self):
        while not self.vertAccelQ.empty():
            try:
                vertAccelReading = self.vertAccelQ.get_nowait()
            except Empty as err:
                break
            vertAccelReading = vertAccelReading.split(",")
            self.vertAccelX.append(vertAccelReading[0])
            self.vertAccelY.append(vertAccelReading[4])

        self.vert_accel_data.setData(self.vertAccelX, self.vertAccelY)  # Update the data.

    def read_data(self):
        dataLine = ser.readline()
        if "Baro" in dataLine:
            self.baroQ.put(dataLine)
        elif "IMU" in dataLine:
            self.accelQ.put(dataLine)

    def button1_clicked(self):
        #open serial link
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)

    def button2_clicked(self):
        #start reading
        self.readTimer.start()

    def button3_clicked(self):
        #stop reading
        self.readTimer.stop()


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())
