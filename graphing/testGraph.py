from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
# from random import randint
import numpy as np


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        # Required initialisation of parent class
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()

        self.group = QtWidgets.QGroupBox()
        group_layout = QtWidgets.QHBoxLayout()

        self.buttons = QtWidgets.QGroupBox()
        button_layout = QtWidgets.QVBoxLayout()

        button1 = QtWidgets.QPushButton('+ amplitude')
        button2 = QtWidgets.QPushButton('- amplitude')

        button1.clicked.connect(self.button1_clicked)
        button2.clicked.connect(self.button2_clicked)

        button_layout.addWidget(button1)
        button_layout.addWidget(button2)

        self.buttons.setLayout(button_layout)

        group_layout.addWidget(self.graphWidget)
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

        self.graphWidget.setBackground('w')

        # Define line colour and style
        pen = pg.mkPen(color=(255, 0, 0))
        self.data_line = self.graphWidget.plot(self.x, self.y, pen=pen)

        # Create a timer and link the function to the time
        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):

        # Update x
        self.start += 0.1
        self.end += 0.1
        self.x = np.linspace(self.start, self.end, self.num_points)

        # Recalc y
        self.y = self.amplitude*np.sin(self.x)

        self.data_line.setData(self.x, self.y)  # Update the data.

    def button1_clicked(self):
        self.amplitude += 0.5

    def button2_clicked(self):
        self.amplitude -= 0.5


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())
