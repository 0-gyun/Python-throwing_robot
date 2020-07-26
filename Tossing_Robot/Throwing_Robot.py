import sys
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtSerialPort import *
from PyQt5.QtMultimedia import *
from PyQt5.QtGui import *
from PyQt5.QtMultimediaWidgets import *
from serial import Serial
from pyqt_led import Led
import random
import math
import os
import time
from keras.models import load_model
from keras.layers import Dense, Dropout, BatchNormalization
from keras import backend as K
from sklearn.preprocessing import MinMaxScaler
from keras.models import Sequential
from keras.layers import Dense
import pandas as pd

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *
DATA = np.loadtxt("dyna.csv", delimiter=",")
POSITION = 30
SPEED = 32
# Protocol version
PROTOCOL_VERSION = 1.0
# Default setting
BAUDRATE = 1000000
DEVICENAME = "COM38"
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

packetHandler.write2ByteTxRx(portHandler, 0, SPEED, 50)
packetHandler.write2ByteTxRx(portHandler, 0, POSITION, 512)
packetHandler.write2ByteTxRx(portHandler, 1, SPEED, 50)
packetHandler.write2ByteTxRx(portHandler, 1, POSITION, 512)

angle_max = 60
RPM_max = 60
RPM_min = 20
x_max = 102
x_min = -213
y_max = 487
y_min = 314

global res, num, B, Goal_X, Goal_Y

__platform__ = sys.platform

class SerialReadThread(QThread):
    received_data = pyqtSignal(QByteArray, name="receivedData")

    def __init__(self, serial):
        QThread.__init__(self)
        self.cond = QWaitCondition()
        self._status = False
        self.mutex = QMutex()
        self.serial = serial

    def __del__(self):
        self.wait()


def get_port_path():
    return {"linux": '/dev/ttyS', "win32": 'COM'}[__platform__]


class MyApp(QWidget):
    def __init__(self,  parent=None):
        super().__init__(parent)

        self.resize(1280, 720)
        self.th = Worker(parent=self)
        ##self.showMaximized()
        #self.center()
        self.show()
        self.init_UI()

    def init_UI(self):
        self.Sensor = Serial("COM27", 115200, timeout=0.05)
        self.setWindowTitle('Tossing Robot Learning Program')
        self.setStyleSheet("background-color: black;")
        self.LED = [Led(self, on_color=Led.green, shape=Led.circle, build='debug') for i in range(9)]
        self.Buttons = [QPushButton() for i in range(17)]
        self.LABELS = [QLabel() for i in range(17)]
        self.BOXES = [QLineEdit() for i in range(17)]
        self.ProgressBars = [QProgressBar() for i in range(17)]
        self.flags = np.zeros(20) ### Homing,Origin, Learing, Pause, goalPost, getPolicies,throwing
        self.OK_sign = ["Lifter Up ok\r", "Lifter Down ok\r", "Press Up ok\r", "Press Down ok\r", "Needle Up ok\r",
                        "Needle Down ok\r", "Slider Front ok\r", "Slider Middle ok\r", "Slider Rear ok\r",
                        "Hanging Position ok\r", "Lifter Up ok\r", "Transfer Position ok\r",
                        "Needle Up Unhanging Position ok\r", "Lifter Down ok\r", "Unloading Position ok\r", "Reset\r",
                        "!!EMG!!\r"]

        "Port Group"
        self.port_group = QGroupBox(self.tr(""))
        self.port_group.setStyleSheet("background-color:rgb(0,0,0);color: white;border: 2px solid rgb(60,60,"
                                     "60);border-radius: 1px;")
        self.cb_port = QComboBox()
        self.cb_port.setStyleSheet("background-color:rgb(50,50,50);color: white")
        self.cb_port.setFont(QFont("BankGothic Md BT", 15, QFont.Bold))
        self.serial = QSerialPort()
        self.serial_info = QSerialPortInfo()
        self.serial_read_thread = SerialReadThread(self.serial)
        self.serial_read_thread.received_data.connect(lambda v: self.received_data.emit(v))
        self.serial_read_thread.start()

        self.serial_port_Label = QLabel('Port')
        self.serial_port_Label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.serial_port_Label.setStyleSheet("background-color:rgb(50,50,50);color: white")
        self.serial_port_Label.setFont(QFont("BankGothic Md BT", 15, QFont.Bold))
        self.Serial_window = QTextBrowser(self)
        self.Serial_window.setStyleSheet("background-color:rgb(50,50,50);color:white;border: 2px solid rgb(60,60,"
                                     "60);border-radius: 1px;max-height:70px")
        self.Serial_window.setFont(QFont("BankGothic Md BT", 10, QFont.Bold))
        self.serial_layout = QGridLayout()
        self.serial_layout.addWidget(self.serial_port_Label, 0, 0)
        self.serial_layout.addWidget(self.cb_port, 0, 1)
        self.serial_layout.addWidget(self.Serial_window, 1, 0, 1, 2)
        self._fill_serial_info()
        self.port_group.setLayout(self.serial_layout)
        self.PORT = self.cb_port.currentText()
        self.cb_port.currentTextChanged.connect(self.serial_changing)
        self.tossing_bot = Serial(self.PORT, 115200, timeout=0.05)
        self.counter = 0
        self.timer = QTimer(self)
        self.timer.start()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.serial_read)


        self.setting = "QPushButton { background-color: qradialgradient(cx:0.5,cy:0.5," \
                       "fx:0.5, fy:0.5,radius: 0.9, stop: 0 rgb(220,255,0),stop: 0.5 rgb(150,255,0),stop: 1 rgb(0,150,0));" \
                       "border-radius: 10px;padding: 6px;border:4px outset rgb(100,100,100);padding:10px"\
                       "} QPushButton:pressed { " \
                       "background-color: rgb(80,80,80); border-radius: 10px;border: 4px inset rgb(50,50,50)} "
        self.setting_b = "QPushButton { background-color: qradialgradient(cx:0.5,cy:0.5," \
                       "fx:0.5, fy:0.5,radius: 0.9, stop: 0 rgb(220,255,0),stop: 0.5 rgb(150,255,0),stop: 1 rgb(0,150,0));" \
                       "border-radius: 10px;padding: 6px;border:4px outset rgb(100,100,100);padding:10" \
                       "} QPushButton:pressed { " \
                       "background-color: rgb(80,80,80); border-radius: 10px;border: 4px inset rgb(50,50,50)} "
        self.setting_un = "QPushButton { background-color: rgb(100,100,100); color: white;border-radius: 10px;border: " \
                          "4px outset rgb(50,50,50);padding:14px;} "
        self.m_setting = "QPushButton { background-color: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, " \
                         "stop: 0 rgb(150,150,150),stop: 0.25 rgb(180,180,180),stop: 0.4 rgb(220,220,220),stop: 0.55 " \
                         "rgb(180,180,180), stop: 1 rgb(150,150,150));border-radius: 10px;padding: 20px;border: " \
                         "4px outset rgb(100,100,100)}QPushButton:pressed { background-color: rgb(100,100," \
                         "100); border-radius: 10px;border: 4px inset rgb(50,50,50)} "
        self.box_setting_1 = "QTextBrowser { background-color: black;border-radius: 10px;padding: 20px;border: " \
                         "4px outset rgb(100,100,100)}QPushButton:pressed { background-color: rgb(100,100," \
                         "100); border-radius: 10px;border: 4px inset rgb(50,50,50)} "

        "Title Group"
        self.title_group = QGroupBox('')
        self.title_group.setStyleSheet(
            "QGroupBox{background-color:rgb(0,0,0);border: 2px solid rgb(50,50,50);border-radius: 1px;}")
        self.title_layout = QGridLayout()
        self.title = QLabel('Tossing Robot Learning Program')
        self.title.setStyleSheet(
            "QLabel{background-color:rgb(0,0,0);color: rgb(0,250,0); border: 4px solid rgb(0,0,0);}")
        self.title.setFont(QFont("BankGothic Md BT", 25, QFont.Bold))
        self.title.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.sub_title = QLabel('CAD CAM LAB')
        self.sub_title.setStyleSheet(
            "QLabel{background-color:rgb(0,0,0);color: rgb(0,250,0); border: 4px solid rgb(0,0,0);}")
        self.sub_title.setFont(QFont("BankGothic Md BT", 10, QFont.Bold))
        self.sub_title.setAlignment(Qt.AlignRight | Qt.AlignTop)
        self.title_layout.addWidget(self.title, 0, 0, 1, 6)
        self.title_layout.addWidget(self.sub_title, 0, 5, 1, 1)
        self.title_group.setLayout(self.title_layout)

        "Input Group"
        self.input_group = QGroupBox('')
        self.input_group.setStyleSheet(
            "QGroupBox{background-color:rgb(0,0,0);border: 2px solid rgb(50,50,50);border-radius: 1px;}")
        self.input_layout = QGridLayout()
        self.LABELS[1] = QLabel('Position')
        self.LABELS[2] = QLabel('Velocity')
        self.LABELS[3] = QLabel('Goal   X')
        self.LABELS[4] = QLabel('Y')
        self.LABELS[5] = QLabel('Result')
        for i in range(9):
            self.BOXES[i].setFont(QFont("BankGothic Md BT", 25, QFont.Bold))
            self.BOXES[i].setStyleSheet(
            "QLineEdit{background-color:rgb(0,0,0);color: rgb(0,250,0); border: 2px solid rgb(50,50,50);}")
            self.BOXES[i].setAlignment(Qt.AlignCenter | Qt.AlignCenter)

        for i in range(5):
            self.LABELS[i+1].setFont(QFont("BankGothic Md BT", 25, QFont.Bold))
            self.LABELS[i + 1].setStyleSheet(
            "QLabel{background-color:rgb(0,0,0);color: rgb(0,250,0); border: 4px solid rgb(0,0,0);}")
            self.LABELS[i+1].setAlignment(Qt.AlignCenter | Qt.AlignCenter)
        self.LABELS[4].setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.input_group.setAlignment(Qt.AlignRight | Qt.AlignTop)
        #self.position.setFont(QFont("BankGothic Md BT", 25, QFont.Bold))
        self.input_layout.addWidget(self.LABELS[1], 0, 0, 1, 2)          #Position
        self.input_layout.addWidget(self.BOXES[0], 0, 2)                 #Pos 1
        self.input_layout.addWidget(self.BOXES[1], 0, 3)                 #Pos 2
        self.input_layout.addWidget(self.BOXES[2], 0, 4)                 #Pos 3
        self.input_layout.addWidget(self.LABELS[2], 1, 0, 1, 2)          #Velocity
        self.input_layout.addWidget(self.BOXES[3], 1, 2)                 #Vel 1
        self.input_layout.addWidget(self.BOXES[4], 1, 3)                 #Vel 2
        self.input_layout.addWidget(self.BOXES[5], 1, 4)                 #Vel 3
        self.input_layout.addWidget(self.LABELS[3], 3, 0, 1, 2)          #Goal   x
        self.input_layout.addWidget(self.BOXES[6], 3, 2)                 #x_val
        self.input_layout.addWidget(self.LABELS[4], 3, 3)                #y
        self.input_layout.addWidget(self.BOXES[7], 3, 4)                 #y_val
        self.input_layout.addWidget(self.LABELS[5], 4, 0, 1, 3)          #Result
        self.input_layout.addWidget(self.BOXES[8], 4, 3, 1, 2)           #Result_Text
        self.input_group.setLayout(self.input_layout)

        "Learning Group"
        self.learning_group = QGroupBox('')
        self.learning_group.setStyleSheet(
            "QGroupBox{background-color:rgb(0,0,0);border: 2px solid rgb(50,50,50);border-radius: 1px;}")
        self.learning_layout = QGridLayout()
        self.LABELS[6] = QLabel('Learning Progress')
        self.LABELS[7] = QLabel('Learing')
        self.LABELS[8] = QLabel('Searching')

        for i in range(2):
            self.BOXES[i + 10].setFont(QFont("BankGothic Md BT", 25, QFont.Bold))
            self.BOXES[i + 10].setStyleSheet(
            "QLineEdit{background-color:rgb(0,0,0);color: rgb(0,250,0); border: 2px solid rgb(50,50,50);}")
            self.BOXES[i + 10].setAlignment(Qt.AlignCenter | Qt.AlignCenter)


        for i in range(3):
            self.LABELS[i + 6].setFont(QFont("BankGothic Md BT", 25, QFont.Bold))
            self.LABELS[i + 6].setAlignment(Qt.AlignCenter | Qt.AlignCenter)
            self.LABELS[i + 6].setStyleSheet(
            "QLabel{background-color:rgb(0,0,0);color: rgb(0,250,0); border: 4px solid rgb(0,0,0);}")

        self.learning_layout.addWidget(self.LABELS[6], 0, 0, 1, 2)
        self.learning_layout.addWidget(self.LABELS[7], 1, 0)
        self.learning_layout.addWidget(self.BOXES[10], 1, 1)
        self.learning_layout.addWidget(self.LABELS[8], 2, 0)
        self.learning_layout.addWidget(self.BOXES[11], 2, 1)
        self.learning_group.setLayout(self.learning_layout)

        "Button Group"
        self.Button_group = QGroupBox('')
        self.Button_group.setStyleSheet(
            "QGroupBox{background-color:rgb(0,0,0);border: 2px solid rgb(50,50,50);border-radius: 1px;}")
        self.Button_layout = QGridLayout()
        self.Buttons[0].setText('Homing')
        self.Buttons[0].clicked.connect(self.homing)
        self.Buttons[1].setText('Learning')
        self.Buttons[1].clicked.connect(self.learning)
        self.Buttons[2].setText('Running')
        self.SQ = 0
        self.Buttons[2].clicked.connect(self.running)
        self.Buttons[3].setText('Pause')
        self.Buttons[3].clicked.connect(self.pause)
        self.Buttons[4].setText('Grasp')
        self.Buttons[4].clicked.connect(self.grasp)
        self.Buttons[5].setText('Release')
        self.Buttons[5].clicked.connect(self.release)

        for i in range(7):
            self.Buttons[i].setFont(QFont("BankGothic Md BT", 25, QFont.Bold))
            self.Buttons[i].setStyleSheet(self.setting)
        self.Buttons[0].setStyleSheet(self.setting_b)
        self.Buttons[3].setStyleSheet(self.setting_b)
            #self.Buttons[i].setAlignment(Qt.AlignCenter | Qt.AlignCenter)

        self.Button_layout.addWidget(self.Buttons[0], 0, 1)
        self.Button_layout.addWidget(self.Buttons[2], 0, 0,)
        self.Button_layout.addWidget(self.Buttons[1], 1, 0)
        self.Button_layout.addWidget(self.Buttons[3], 1, 1)
        self.Button_layout.addWidget(self.Buttons[4], 2, 0)
        self.Button_layout.addWidget(self.Buttons[5], 2, 1)
        self.Button_group.setLayout(self.Button_layout)

        "Set Layout"
        self.layout = QGridLayout()
        self.layout.addWidget(self.title_group, 0, 0, 1, 2)
        self.layout.addWidget(self.port_group, 1, 0)
        self.layout.addWidget(self.Button_group, 2, 0)
        self.layout.addWidget(self.input_group, 1, 1,2,1)
        self.layout.addWidget(self.learning_group, 3, 1)
        self.setLayout(self.layout)



    def serial_changing(self, value):
        self.tossing_bot = Serial(value, 115200, timeout=0.05)
    def _fill_serial_info(self):
        self.cb_port.insertItems(0, self._get_available_port())
    def _get_available_port(self):
        available_port = list()
        port_path = get_port_path()
        for number in range(70):
            port_name = port_path + str(number)
            if not self._open(port_name):
                continue
            available_port.append(port_name)
            self.serial.close()
        return available_port
    def _open(self, port_name):
        info = QSerialPortInfo(port_name)
        self.serial.setPort(info)
        return self.serial.open(QIODevice.ReadWrite)
    def alert(self, s):
        err = QErrorMessage(self)
        err.showMessage(s)
    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
    def serial_read(self):
        self.s = self.tossing_bot.readline()
        self.Serial_str = str(self.s, 'ascii', 'replace')
        self.str = self.Sensor.readline()
        self.sensor_str = str(self.str, 'ascii', 'replace')

        if self.flags[0] and (self.Serial_str == "Homing\r\n"):
           self.Serial_window.insertPlainText("homing is done\n")
           self.Serial_window.verticalScrollBar().setValue(self.Serial_window.verticalScrollBar().maximum())
           self.flags[0] = 0

        if self.flags[1] and (self.Serial_str == "Origin\r\n"):
            self.flags[1] = 0
            self.Serial_window.insertPlainText("origin\n")
            self.Serial_window.verticalScrollBar().setValue(self.Serial_window.verticalScrollBar().maximum())

        if self.flags[4] and (self.Serial_str == "throwing\r\n"):
            self.Serial_window.insertPlainText("throwing\n")
            self.Serial_window.verticalScrollBar().setValue(self.Serial_window.verticalScrollBar().maximum())
            self.flags[4] = 0

        if self.flags[5] and (self.sensor_str == "Goal\r\n"):
            self.flags[5] = 0

        if self.counter > 0:
            self.counter -= 1

        else:
            if self.SQ == 1 and self.flags[2] and not(self.flags[0]):
                self.BOXES[11].setText("Loading...")
                self.Serial_window.insertPlainText("searching\n")
                self.Serial_window.verticalScrollBar().setValue(self.Serial_window.verticalScrollBar().maximum())
                self.SQ = 2
                self.counter = 2
            elif self.SQ == 2 and self.flags[2]:
                self.th.searching()
                self.BOXES[11].setText("Done")
                self.Serial_window.insertPlainText("Goal Sensor on\n")
                self.Serial_window.verticalScrollBar().setValue(self.Serial_window.verticalScrollBar().maximum())
                self.sensor_on()
                self.SQ = 3
                self.counter = 2
            elif self.SQ == 3 and self.flags[2]:
                self.Serial_window.insertPlainText("Goal Sensor on\n")
                self.Serial_window.verticalScrollBar().setValue(self.Serial_window.verticalScrollBar().maximum())
                self.getPolicies()
                self.SQ = 4
                self.counter = 2
            elif self.SQ == 4 and self.flags[2]:
                self.throwing()
                self.SQ = 5
                self.counter = 40
            elif self.SQ == 5 and self.flags[2] and not(self.flags[4]):
                if self.flags[5]:
                    self.BOXES[8].setText("Fail")
                else:
                    self.BOXES[8].setText("Success")
                self.Serial_window.insertPlainText("Sequence done\n")
                self.Serial_window.verticalScrollBar().setValue(self.Serial_window.verticalScrollBar().maximum())
                self.Sensor.write(b"1")
                self.counter = 1
                self.save_data()
                self.SQ = 6

            if self.SQ == 50 and self.flags[10]:
                self.BOXES[10].setText("Loading...")
                self.counter = 2
                self.SQ = 51
            elif self.SQ == 51 and self.flags[10]:
                self.th.machine_learning()
                self.BOXES[10].setText("Done")
                self.SQ = 52
                self.flags[10] = 0

    def homing(self):
        self.tossing_bot.write(b"h")
        self.flags[0] = 1

    def learning(self):
        self.flags[10] = 1
        self.SQ == 50
        self.counter = 5

    def running(self):
        self.BOXES[9].setText("")
        self.BOXES[10].setText("")
        self.BOXES[11].setText("")
        self.flags[2] = 1
        self.SQ = 1
        self.homing()
        self.goalPost()
        self.counter = 5

    def pause(self):
        self.flags[2] = 0

    def goalPost(self):
        #time.sleep(1)
        (M, N) = DATA.shape
        k = random.randrange(0, M - 1)
        packetHandler.write2ByteTxRx(portHandler, 0, POSITION, int((DATA[k, 0] / 300 + 1 / 5) * 1024))
        packetHandler.write2ByteTxRx(portHandler, 1, POSITION, int((DATA[k, 1] / 300 + 1 / 5) * 1024))
        L0 = 320
        L1 = 250
        L2 = 250
        m = L0 + L1 * (math.cos(DATA[k, 1] / 180 * math.pi) - math.cos(DATA[k, 0] / 180 * math.pi))
        n = L1 * abs(math.sin(DATA[k, 1] / 180 * math.pi) - math.sin(DATA[k, 0] / 180 * math.pi))
        p1 = math.acos(math.sqrt(m ** 2 + n ** 2) / (2 * L2))
        p2 = math.atan(n / m)
        th1 = 2 * math.pi - p1 + p2
        th2 = math.pi - p1 + p2
        global Goal_X, Goal_Y
        Goal_X = - L0 / 2 + L1 * math.cos(DATA[k, 0] * math.pi / 180) + L2 * math.cos(th1)
        Goal_Y = L1 * math.sin(DATA[k, 1] * math.pi / 180) + L2 * math.sin(th2)
        self.BOXES[6].setText(str(round(Goal_X+1500, 2)))
        self.BOXES[7].setText(str(round(Goal_Y+400, 2)))

    def getPolicies(self):
        self.P1 = B[num, 0] * angle_max
        self.P2 = B[num, 1] * angle_max
        self.P3 = B[num, 2] * angle_max
        self.W1 = B[num, 3] * (RPM_max - RPM_min) + RPM_min
        self.W2 = B[num, 4] * (RPM_max - RPM_min) + RPM_min
        self.W3 = B[num, 5] * (RPM_max - RPM_min) + RPM_min
        self.BOXES[0].setText(str(round(self.P1, 2)))
        self.BOXES[1].setText(str(round(self.P2, 2)))
        self.BOXES[2].setText(str(round(self.P3, 2)))
        self.BOXES[3].setText(str(round(self.W1, 2)))
        self.BOXES[4].setText(str(round(self.W2, 2)))
        self.BOXES[5].setText(str(round(self.W3, 2)))
        data = pd.DataFrame(np.array([[self.P1, self.P2, self.P3, self.W1, self.W2, self.W3, np,max(res)[0]]]))
        data.to_csv('policies_0724_5.csv', mode='a', header=None, index=False)
        self.position_comm = "p" + str(round(self.P1,3)) + "," + str(round(self.P2,3)) + "," + str(round(self.P3,3))
        print(self.position_comm)
        self.tossing_bot.write(str.encode(self.position_comm))
        time.sleep(1.5)
        self.velocity_comm = "v" + str(round(self.W1,3)) + "," + str(round(self.W2,3)) + "," + str(round(self.W3,3))
        print(self.velocity_comm)
        self.tossing_bot.write(str.encode(self.velocity_comm))

    def save_data(self):
        if self.flags[5]:
            self.res = 0
        else:
            self.res = 1
        ata = pd.DataFrame(np.array([[self.P1, self.P2, self.P3, self.W1, self.W2, self.W3, Goal_X, Goal_Y, self.res]]))
        ata.to_csv('input_0724_5.csv', mode='a', header=None, index=False)

    def throwing(self):
        print("throw")
        self.tossing_bot.write(b"o")
        self.flags[4] = 1

    def sensor_on(self):
        self.Sensor.write(b"0")
        self.flags[5] = 1

    def grasp(self):
        self.tossing_bot.write(b"n")

    def release(self):
        self.tossing_bot.write(b"m")

class Worker(QThread):
    def __init__(self, sec=0, parent=None):
        super().__init__()
        self.main = parent

    def machine_learning(self):
        model = Sequential()
        model.add(Dense(256, input_dim=8, init='normal', activation='relu'))
        model.add(BatchNormalization())
        model.add(Dense(256, activation='relu'))
        model.add(BatchNormalization())
        model.add(Dropout(0.4))
        model.add(Dense(128, activation='relu'))
        model.add(BatchNormalization())
        model.add(Dropout(0.4))
        model.add(Dense(128, activation='relu'))
        model.add(BatchNormalization())
        model.add(Dense(1, activation='sigmoid'))
        model.compile(loss='binary_crossentropy', optimizer='adam')
        DATA = np.loadtxt("input_0724_5.csv", delimiter=",")

        X = DATA[:, :8]
        X[:, 0:3] = X[:, 0:3] / angle_max
        X[:, 3:6] = (X[:, 3:6] - RPM_min) / (RPM_max - RPM_min)
        X[:, 6] = (X[:, 6] - x_min) / (x_max - x_min)
        X[:, 7] = (X[:, 7] - y_min) / (y_max - y_min)
        Y = DATA[:, 8]

        model.fit(X, Y, nb_epoch=3000, batch_size=20, verbose=2)
        model.save('learning_model.h5')

    def searching(self):
        global res, num, B
        Coor = np.array([(Goal_X - x_min) / (x_max - x_min), (Goal_Y - y_min) / (y_max - y_min)])
        search_point = 100000
        np.tile(Coor, (search_point, 1))
        A = np.random.rand(search_point, 6)
        B = np.random.rand(search_point, 8)
        B[:, :6] = A
        B[:, 6:] = Coor
        model = load_model('learning_model.h5')
        res = model.predict(B)
        sigma = 3
        A1 = np.random.normal(B[num, 0], sigma, search_point)
        A2 = np.random.normal(B[num, 1], sigma, search_point)
        A3 = np.random.normal(B[num, 2], sigma, search_point)
        A4 = np.random.normal(B[num, 3], sigma, search_point)
        A5 = np.random.normal(B[num, 4], sigma, search_point)
        A6 = np.random.normal(B[num, 5], sigma, search_point)
        B[:, 0] = A1
        B[:, 1] = A2
        B[:, 2] = A3
        B[:, 3] = A4
        B[:, 4] = A5
        B[:, 5] = A6
        B[:, 6:] = Coor
        res = model.predict(B)
        num = np.argmax(res)
        print(np.max(res))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())
