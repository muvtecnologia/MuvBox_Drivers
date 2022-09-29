####  Muv Box Controller v 0.7 alpha ####

## Uses muvbox.py version 0.9 or greater
## Plot improved

# How to improve matplotlib plot performance:
# https://bastibe.de/2013-05-30-speeding-up-matplotlib.html
# https://stackoverflow.com/questions/57891219/how-to-make-a-fast-matplotlib-live-plot-in-a-pyqt5-gui


# TODO: atualizar cores da tabela (legenda)
# TODO: indicar visualmente quando uma MuvBox está conectada
# TODO: trazer as informações da tabela para tela principal
# TODO: retirar os limites superiores dos SpinBoxes de frequência de corte e tamanho da janela

from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QAction, QTableWidget, QTableWidgetItem, QVBoxLayout, QMenu, QAction, QInputDialog, QComboBox, QStackedWidget, QMessageBox
from PyQt5.QtGui import QCursor, QBrush, QColor, QPixmap, QIcon
from muvbox import *
from pylab import *
import datetime
import sys, csv
import time
from threading import Thread, current_thread
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5 import NavigationToolbar2QT as NavigationToolbar
from matplotlib import animation
import matplotlib.image as mpimg
from matplotlib import pyplot as plt
from matplotlib.widgets import SpanSelector
import matplotlib as mpl
import os
import platform
from scipy import ndimage, signal
import mpl_toolkits.mplot3d as plt3d
import numpy as np
import math
from PIL import Image
import subprocess
import ctypes  # An included library with Python install.
import itertools
from ahrs import Quaternion
import ahrs

from utilities import *

TABLE_FILE = "./res/ip.txt"
LOGO_FILE = "./res/muv.svg"
USER_OS = platform.system()

# ctypes - MessageBox
MB_OK = 0
MB_OKCANCEL = 1
MB_YESNOCANCEL = 3
MB_YESNO = 4
IDOK = 1
IDCANCEL = 2
IDABORT = 3
IDYES = 6
IDNO = 7

def Mbox(title, text, style):
        msgBox = QMessageBox()
        msgBox.setText(text)
        msgBox.setWindowTitle(title)
        print(style)
        if style == 'warning':        
            msgBox.setIcon(QMessageBox.Warning)
            msgBox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            returnValue = msgBox.exec()
            if returnValue == QMessageBox.Ok:
                return True
            else:
                return False
        if style == 'information':
            msgBox.setIcon(QMessageBox.Information)
            msgBox.setStandardButtons(QMessageBox.Ok)
            msgBox.exec()


# TODO: inserir o comando finalize quando parar o gráfico.

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        #Load the UI Page
        uic.loadUi('main.ui', self)
        self.button = self.findChild(QtWidgets.QPushButton, 'connectButton')
        self.pages = self.findChild(QtWidgets.QStackedWidget, 'stackedWidget')
        self.start_all_button  = self.findChild(QtWidgets.QPushButton, 'pushButton')
        self.connect_all_button = self.findChild(QtWidgets.QPushButton, 'pushButton_2')
        self.disconnect_all_button = self.findChild(QtWidgets.QPushButton, 'pushButton_8')
        self.save_table_button = self.findChild(QtWidgets.QPushButton, 'pushButton_3')
        self.config_button = self.findChild(QtWidgets.QPushButton, 'pushButton_4')
        self.update_graph_button = self.findChild(QtWidgets.QPushButton, 'pushButton_5')
        self.save_data_button = self.findChild(QtWidgets.QPushButton, 'pushButton_6')
        self.sincronizar_button = self.findChild(QtWidgets.QPushButton, 'pushButton_7')
        self.stop_all_button = self.findChild(QtWidgets.QPushButton, 'pushButton_9')
        self.back_button = self.findChild(QtWidgets.QPushButton, 'pushButton_10')
        self.first_config_button = self.findChild(QtWidgets.QPushButton, 'pushButton_11')
        self.layout2 = self.findChild(QtWidgets.QVBoxLayout, 'verticalLayout_2')
        self.logbox = self.findChild(QtWidgets.QPlainTextEdit, 'plainTextEdit')
        self.parar = False  # para a leitura de dados dos sensores se True
        self.time_window = self.findChild(QtWidgets.QSpinBox, 'spinBox_4')  # Janela de tempo do gráfico em tempo real, em segundos
        self.radio_button_acc = self.findChild(QtWidgets.QRadioButton, 'radioButton')
        self.radio_button_gyr = self.findChild(QtWidgets.QRadioButton, 'radioButton_2')
        self.radio_button_angles = self.findChild(QtWidgets.QRadioButton, 'radioButton_3')
        self.table = self.findChild(QtWidgets.QTableWidget, 'tableWidget')
        self.start_value = self.findChild(QtWidgets.QLineEdit, 'lineEdit')
        self.stop_value = self.findChild(QtWidgets.QLineEdit, 'lineEdit_2')
        self.main_frame = self.findChild(QtWidgets.QFrame, 'frame')
        self.logo_muv = self.findChild(QtWidgets.QLabel, 'label_8')
        self.cutoff = self.findChild(QtWidgets.QSpinBox, 'spinBox_5')
        self.log_data = 0
        ## Config. Inicial var
        self.scanButton = self.findChild(QtWidgets.QPushButton, 'pushButton_12')
        self.saveButton = self.findChild(QtWidgets.QPushButton, 'pushButton_15')
        self.cancelButton = self.findChild(QtWidgets.QPushButton, 'pushButton_16')
        self.configButton = self.findChild(QtWidgets.QPushButton, 'pushButton_13')
        self.backButton = self.findChild(QtWidgets.QPushButton, 'pushButton_14')
        self.listview = self.findChild(QtWidgets.QListWidget, 'listWidget')
        self.networkComboBox = self.findChild(QtWidgets.QComboBox, 'comboBox')
        self.hostnameField = self.findChild(QtWidgets.QLineEdit, 'lineEdit_3')
        self.ssidField = self.findChild(QtWidgets.QLineEdit, 'lineEdit_4')
        self.passwordField = self.findChild(QtWidgets.QLineEdit, 'lineEdit_5')
        self.currentNetworkLabel = self.findChild(QtWidgets.QLabel, 'label_9')
        self.infoLabel = self.findChild(QtWidgets.QLabel, 'label_14')
        self.infoScanLabel = self.findChild(QtWidgets.QLabel, 'label_15')
        self.closeButton = self.findChild(QtWidgets.QPushButton, 'pushButton_18')
        
        self.current_network = ''

        # Log datetime
        
        self.start_log()

        ## Buttons icon set
        # PAGE 1
        self.connect_all_button.setIcon(QIcon('./icons/connect.svg'))
        self.disconnect_all_button.setIcon(QIcon('./icons/disconnect.svg'))
        self.start_all_button.setIcon(QIcon('./icons/play.svg'))
        self.stop_all_button.setIcon(QIcon('./icons/pause.svg'))
        self.sincronizar_button.setIcon(QIcon('./icons/sync.svg'))
        self.update_graph_button.setIcon(QIcon('./icons/updategraph.svg'))
        self.save_data_button.setIcon(QIcon('./icons/save.svg'))
        self.config_button.setIcon(QIcon('./icons/settings.svg'))
        self.first_config_button.setIcon(QIcon('./icons/first_config.svg'))
        # PAGE 2
        self.save_table_button.setIcon(QIcon('./icons/save.svg'))
        self.back_button.setIcon(QIcon('./icons/back.svg'))

        ## First config
        self.scanButton.setIcon(QIcon('./icons/scan.svg'))
        self.saveButton.setIcon(QIcon('./icons/save.svg'))
        self.cancelButton.setIcon(QIcon('./icons/cancel.svg'))
        self.configButton.setIcon(QIcon('./icons/settings.svg'))
        self.backButton.setIcon(QIcon('./icons/back.svg'))
        self.closeButton.setIcon(QIcon('./icons/cancel.svg'))

        # Table
        self.load_table()
       
        # Figure sensor_data
        self.fig2, self.ax2 = plt.subplots(3, sharex=True)
        for x in range(0,3,1):
            self.ax2[x].spines['top'].set_color('#EBEBEB')
            self.ax2[x].spines['bottom'].set_color('black')
            self.ax2[x].spines['left'].set_color('black')
            self.ax2[x].spines['right'].set_color('#EBEBEB')
            self.ax2[x].xaxis.label.set_color('black') 
            self.ax2[x].yaxis.label.set_color('black') 
            self.ax2[x].tick_params(axis='x', colors='black')
            self.ax2[x].tick_params(axis='y', colors='black')
            self.ax2[x].title.set_color('black') 
        self.fig2.patch.set_facecolor('white')
        self.canvas2 = FigureCanvas(self.fig2)
        self.canvas2.draw()                

        # # Cria as linhas
        self.line0 = []
        self.line1 = []
        self.line2 = []

        # self.line0, = self.ax2[0].plot([0],[0]) 
        # self.line1, = self.ax2[1].plot([0],[0]) 
        # self.line2, = self.ax2[2].plot([0],[0]) 
        # self.canvas2.draw()

        # self.ax2[0].draw_artist(self.line0)
        # self.ax2[1].draw_artist(self.line1)
        # self.ax2[2].draw_artist(self.line2)
        
        self.layout2.addWidget(self.canvas2)
        self.canvas2.draw()
        
        
        # TODO - ajustar Navigation Toolbar
        self.mpl_toolbar = NavigationToolbar(self.canvas2, self.main_frame)
        self.mpl_toolbar.setStyleSheet("color:black")
        vbox = QVBoxLayout()
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        
        # Timer de atualização do gráfico
        self.timer2 = QtCore.QTimer()
        self.timer2.setInterval(200)
        self.timer2.timeout.connect(self.update_sensor_data)

        self.M = []  # Vetor de muvboxes

        self.threads = []     
        logo = QPixmap(LOGO_FILE)
        self.logo_muv.setPixmap(logo.scaled(200, 200, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

        
    def stop_reading(self):
        self.timer2.stop()
        for m in range(len(self.M)):
            self.M[m].stop()        
        self.update_sensor_data(0)  # Atualiza o gráfico com todos os pontos lidos
    
    def disconnect(self):
        self.stop_reading() # Para antes de dar disconnect
        for m in range(len(self.M)):
            self.M[m].disconnect()
        
            
    def start(self):              
        for m in range(len(self.M)):
            # TODO: arrumar as cores dos gráficos
            # Cores das linhas dos gráficos
            mpl.style.use('default')
            color_c = 'C' + str(m+1)
            legend = self.M[m].name
            r, g, b =  mpl.colors.to_rgb(color_c)
            r = r*255
            g = g*255
            b = b*255
            #self.M[m].color = [r, g, b]    
            self.M[m].color = color_c
            
            # Cria as linhas dos gráficos
            line0, = self.ax2[0].plot([0],[0]) 
            line1, = self.ax2[1].plot([0],[0]) 
            line2, = self.ax2[2].plot([0],[0]) 
            line0.set_color(self.M[m].color)
            line1.set_color(self.M[m].color)
            line2.set_color(self.M[m].color)
            self.line0.append(line0)
            self.line1.append(line1)
            self.line2.append(line2)
            self.ax2[0].draw_artist(self.line0[m])
            self.ax2[1].draw_artist(self.line1[m])
            self.ax2[2].draw_artist(self.line2[m])

            self.M[m].ACCSCALE = self.table.cellWidget(7, m).currentIndex()
            print(self.M[m].ACCSCALE)
            self.M[m].GYROSCALE = self.table.cellWidget(6,m).currentIndex()
            self.M[m].ACQ_FREQ = int(self.table.item(8,m).text())
            self.M[m].start()
        self.timer2.start()
        self.canvas2.draw()

    def connect_n(self):
        # Connect all MuvBoxes from table
        # Connect to multiple MuvBoxes
        if USER_OS == 'Linux': #Check for Linux and clear mDns
            os.system('avahi-browse -a -r -t -v')
            time.sleep(1)
        for m in range(len(self.M)):
            self.M[m].stop_reading = True
        time.sleep(1)
        self.M.clear()  # Limpa vetor de muvboxes

        print(self.table.columnCount())
        self.stringlist0 = [[] for i in range(self.table.columnCount())]

        for column in range(self.table.columnCount()):
            m = column
            self.M.append(MuvBox())   # cria a muvbox
            #TODO: Check here
            self.M[m].name = self.table.item(0,m).text()
            self.M[m].mac = self.table.item(3,m).text()
            self.M[m].version = self.table.item(1,m).text()
            self.M[m].muvbox_number = m
            self.M[m].location = self.table.item(2,m).text()
            # self.M[m].calculate_quaternion = False
            self.M[m].marg = False      
            self.M[m].logbox = self.stringlist0[m]        
            self.M[m].connect()           

        
        self.parar = False
        
        self.timer3 = QtCore.QTimer()
        self.timer3.setInterval(1000) # 1 segundo
        self.timer3.timeout.connect(self.update_logbox)
        self.timer3.start()    

    def connect_single(self, m):
        # Connect a single MuvBox
        if m>=0 and m<len(self.M):
            self.M[m].ip = self.table.item(4,m).text()
            self.M[m].name = self.table.item(0,m).text()
            self.M[m].mac = self.table.item(3,m).text()
            self.M[m].version = self.table.item(1,m).text()
            self.M[m].muvbox_number = m
            self.M[m].location = self.table.item(2,m).text()
            self.M[m].connect()

    def start_log(self):
        self.log_date = str(datetime.datetime.now())
        self.log_date = self.log_date.replace('-','')
        self.log_date = self.log_date.replace(' ','_')
        self.log_date = self.log_date.replace(':','h',1)        
        self.log_date = self.log_date.replace(':','m',1)        
        self.log_date = self.log_date.replace('.','s')
        self.log_date = self.log_date[ 0 : self.log_date.find('s') ]
        self.log_filename = './log/' + self.log_date + '_log.txt'
        if os.path.exists('./log') != True:
            os.mkdir('./log')
        with open(self.log_filename, 'a') as file_object:
            file_object.write('Log started at '+str(datetime.datetime.now())+'\n\n')
        self.logbox.setReadOnly(True)

    def update_logbox(self):
        try:
            for m in range(len(self.stringlist0)):
                for n in range(len(self.stringlist0[m])):
                    self.log_write('['+str(datetime.datetime.now())+']:  '+self.stringlist0[m][n])
                self.stringlist0[m].clear()   
        except:
            print('Erro atualizando logboxes')

    def log_write(self, string):
        with open(self.log_filename, 'a') as file_object:
            file_object.write('\n'+string)
        self.logbox.appendPlainText(string)
               

        
    def sincronizar(self):
        # Reseta o valor de rtc0 e limpa os vetores de dados, reiniciando o gráfico
        #print('Starting synchronization...')    
        #self.log_write('Starting synchronization...')  
        self.timer2.stop() # Interrompe a atualização do gráfico
        time.sleep(0.3)
        for m in range(len(self.M)):
            self.M[m].ajustar_rtc0 = True   # Nâo precisa: quando a MuvBox reseta já ajusta o rtc0
        time.sleep(0.3)
        self.timer2.start()  # Retoma a atualização do gráfico
        #print('Synchronization done.')     
        #self.log_write('Synchronization done.')   

    def update_sensor_data(self, opt:int=1):
        # TODO: ajustar limite superior dos boxes da frequência de corte e time window
        # Plota os dados das MuvBoxes no gráfico
        # opt = 1: plota os últimos n pontos dados por self.time_window(). Usado para gráfico em tempo real
        # opt = 0: plota todos os pontos dos sensores. Usado para gráfico offline

        nro_active_muvboxes = len(self.M)
        
        self.max_time = 0  # tempo do último ponto a aparecer no gráfico (usado para xlim)

        for m in range(nro_active_muvboxes):
            if (self.M[m].visible and self.M[m].sensors.size>30):
                if self.radio_button_acc.isChecked() or self.radio_button_gyr.isChecked():
                    self.M[m].calculate_quaternion = False
                    last_index = self.M[m].sensors.size - 1
                    rtc = self.M[m].sensors.data[:last_index,0]  # Vetor de tempo
                else:
                    self.M[m].calculate_quaternion = True
                    last_index = self.M[m].angles.size - 1
                    rtc = self.M[m].angles.data[:last_index,0]  # Vetor de tempo
                
                if (len(rtc)>30):
                    deltat = (rtc[-1] - rtc[-20])
                    if (deltat>0):
                        acquisition_rate = (20/deltat)
                    else:
                        acquisition_rate = 0
                else:
                    acquisition_rate = 0
                self.M[m].acq_rate = acquisition_rate
                
                # Calcula o número de pontos dos vetores a serem impressos no gráfico
                nro_pontos = 0
                try:
                    if (len(rtc)>0):
                        x_axis_min = rtc[-1] - self.time_window.value()
                    else:
                        x_axis_min = 0
                    
                    if (opt==1):
                        nro_pontos = sum(rtc>x_axis_min)
                    else:
                        nro_pontos = len(rtc)
                except:
                    nro_pontos = 1

                first_index = len(rtc) - nro_pontos
                
                if self.radio_button_acc.isChecked():
                    try:
                        # TODO: testar para várias muvboxes ao mesmo tempo
                        rtc = self.M[m].sensors.data[first_index:last_index,0]
                        rtc = rtc - self.M[m].rtc0      
                        acc_x = self.M[m].sensors.data[first_index:last_index,1]
                        acc_y = self.M[m].sensors.data[first_index:last_index,2]
                        acc_z = self.M[m].sensors.data[first_index:last_index,3]

                        if (self.M[m].acq_rate > 2*float(self.cutoff.value())):
                            acc_x = lowpass_iir_filter(acc_x, self.M[m].acq_rate, float(self.cutoff.value()))
                            acc_y = lowpass_iir_filter(acc_y, self.M[m].acq_rate, float(self.cutoff.value()))
                            acc_z = lowpass_iir_filter(acc_z, self.M[m].acq_rate, float(self.cutoff.value()))
                        
                        self.line0[m].set_ydata(acc_x)
                        self.line0[m].set_xdata(rtc)
                        self.line1[m].set_ydata(acc_y)
                        self.line1[m].set_xdata(rtc)
                        self.line2[m].set_ydata(acc_z)
                        self.line2[m].set_xdata(rtc)
                        self.ax2[0].draw_artist(self.line0[m])
                        self.ax2[1].draw_artist(self.line1[m])
                        self.ax2[2].draw_artist(self.line2[m])
                                                
                        self.fig2.update()
                        self.fig2.flush_events()

                    except:
                        pass

                if self.radio_button_gyr.isChecked():
                    try:
                        rtc = self.M[m].sensors.data[first_index:last_index,0]
                        rtc = rtc - self.M[m].rtc0   
                        gyr_x = self.M[m].sensors.data[first_index:last_index,4]
                        gyr_y = self.M[m].sensors.data[first_index:last_index,5]
                        gyr_z = self.M[m].sensors.data[first_index:last_index,6]

                        if (self.M[m].acq_rate > 2*float(self.cutoff.value())):
                            gyr_x = lowpass_iir_filter(gyr_x, self.M[m].acq_rate, float(self.cutoff.value()))
                            gyr_y = lowpass_iir_filter(gyr_y, self.M[m].acq_rate, float(self.cutoff.value()))
                            gyr_z = lowpass_iir_filter(gyr_z, self.M[m].acq_rate, float(self.cutoff.value()))
                        
                        self.line0[m].set_ydata(gyr_x)
                        self.line0[m].set_xdata(rtc)
                        self.line1[m].set_ydata(gyr_y)
                        self.line1[m].set_xdata(rtc)
                        self.line2[m].set_ydata(gyr_z)
                        self.line2[m].set_xdata(rtc)
                        self.ax2[0].draw_artist(self.line0[m])
                        self.ax2[1].draw_artist(self.line1[m])
                        self.ax2[2].draw_artist(self.line2[m])
                                                
                        self.fig2.update()
                        self.fig2.flush_events()
                    except:
                        pass
    
                if self.radio_button_angles.isChecked():
                    try:
                        rtc = self.M[m].angles.data[first_index:last_index,0]
                        rtc = rtc - self.M[m].rtc0 
                        yaw = self.M[m].angles.data[first_index:last_index,1]
                        pitch = self.M[m].angles.data[first_index:last_index,2]
                        roll = self.M[m].angles.data[first_index:last_index,3]
                        
                        if (self.M[m].acq_rate > 2*float(self.cutoff.value())):
                            yaw = lowpass_iir_filter(yaw, self.M[m].acq_rate, float(self.cutoff.value()))
                            pitch = lowpass_iir_filter(pitch, self.M[m].acq_rate, float(self.cutoff.value()))
                            roll = lowpass_iir_filter(roll, self.M[m].acq_rate, float(self.cutoff.value()))

                        self.line0[m].set_ydata(yaw)
                        self.line0[m].set_xdata(rtc)
                        self.line1[m].set_ydata(pitch)
                        self.line1[m].set_xdata(rtc)
                        self.line2[m].set_ydata(roll)
                        self.line2[m].set_xdata(rtc)
                        self.ax2[0].draw_artist(self.line0[m])
                        self.ax2[1].draw_artist(self.line1[m])
                        self.ax2[2].draw_artist(self.line2[m])
                                                
                        self.fig2.update()
                        self.fig2.flush_events()
                      
                    except:
                        pass

                    
                if (len(rtc)>0) and (rtc[-1] > self.max_time):
                    self.max_time = rtc[-1]

                    
        # Ajusta xlim
        self.ax2[0].set_xlim(self.max_time-self.time_window.value(), self.max_time)
        self.ax2[1].set_xlim(self.max_time-self.time_window.value(), self.max_time)
        self.ax2[2].set_xlim(self.max_time-self.time_window.value(), self.max_time)

        if self.radio_button_acc.isChecked():
            max_g = 2
            for m in range(len(self.M)):
                if self.M[m]._GSCALE > max_g:
                    max_g = self.M[m]._GSCALE
            
            self.ax2[0].set_ylim(-max_g, max_g)
            self.ax2[1].set_ylim(-max_g, max_g)
            self.ax2[2].set_ylim(-max_g, max_g)
            
            self.ax2[0].set_xlabel('Time [s]')
            self.ax2[0].set_ylabel('Acc(x) [g]')
            self.ax2[1].set_xlabel('Time [s]')
            self.ax2[1].set_ylabel('Acc(y) [g]')
            self.ax2[2].set_xlabel('Time [s]')
            self.ax2[2].set_ylabel('Acc(z) [g]')

        if self.radio_button_gyr.isChecked():
            self.ax2[0].set_ylim(-400, 400)
            self.ax2[1].set_ylim(-400, 400)
            self.ax2[2].set_ylim(-400, 400)
            self.ax2[0].set_xlabel('Time [s]')
            self.ax2[0].set_ylabel('Gyro(x) [°/s]')
            self.ax2[1].set_xlabel('Time [s]')
            self.ax2[1].set_ylabel('Gyro(y) [°/s]')
            self.ax2[2].set_xlabel('Time [s]')
            self.ax2[2].set_ylabel('Gyro(z) [°/s]')

        if self.radio_button_angles.isChecked():
            self.ax2[0].set_ylim(-200, 200)
            self.ax2[1].set_ylim(-200, 200)
            self.ax2[2].set_ylim(-200, 200)
            self.ax2[0].set_xlabel('Time [s]')
            self.ax2[0].set_ylabel('Yaw [°]')
            self.ax2[1].set_xlabel('Time [s]')
            self.ax2[1].set_ylabel('Pitch [°]')
            self.ax2[2].set_xlabel('Time [s]')
            self.ax2[2].set_ylabel('Roll [°]')

        self.canvas2.draw()
        
        self.update_table_status() 
        

    def include_table(self, event):
        self.new_col_num = self.table.columnCount() + 1
        self.table.setColumnCount(self.new_col_num)       
        item = QTableWidgetItem('500')  
        self.table.setItem(8, self.new_col_num-1, item) 
        self.gyro_options = ["250","500","1000","2000"]
        self.acc_options = ["2", "4", "8", "16"]    
        self.gyro_combo = QComboBox()
        self.acc_combo = QComboBox()
        for x in range(0,4,1):
            self.gyro_combo.addItem(self.gyro_options[x])
            self.acc_combo.addItem(self.acc_options[x])
        self.table.setCellWidget(6, self.new_col_num - 1, self.gyro_combo)    
        self.table.setCellWidget(7, self.new_col_num - 1, self.acc_combo)

    def delete_table(self, event):
        if (Mbox('Warning', 'Are you sure you want to erase instance #' + str(self.table.currentColumn() + 1) + '?', 'warning') == True):
            self.table.removeColumn(self.table.currentColumn())
            del self.M[self.table.currentColumn()]

    def connect_table(self, event):
        # Connect selected MuvBox
        self.connect_single(self.table.currentColumn())

    def save_table(self):
        with open(TABLE_FILE, 'w', newline='', encoding='iso-8859-1') as stream:
            writer = csv.writer(stream)
            for column in range(self.table.columnCount()):
                columndata = []
                for row in range(self.table.rowCount()):    
                    if row == 6 or row == 7:
                       print(column)
                       widget = self.table.cellWidget(row, column)
                       columndata.append(str(widget.currentIndex()))                      
                    else:                          
                        item = self.table.item(row, column) 
                        if item is not None:
                            columndata.append(item.text())
                        else:
                            columndata.append('')
                writer.writerow(columndata)
        Mbox('Save file', 'Table saved!', 'information')


    def load_table(self):
        if os.path.isfile('./res/ip.txt') != True:
            with open('./res/ip.txt', 'a') as file_object:
                file_object.write('')
        with open(TABLE_FILE, 'r', newline='' ,encoding='iso-8859-1') as stream:
            self.table.setRowCount(14)
            self.table.setColumnCount(0)
            self.table.resizeRowsToContents()
            self.table.resizeColumnsToContents()
            self.table.setVerticalHeaderLabels(["Name", "Version", "Location", "MAC", "IP", "Status", "Gyro. Scale (º/s)", "Acc. Scale (G)", "Nom. acq. rate (pps)", "Inst. acq. rate (pps)", "Bat. level (%)", "Firmware", "Free heap (bytes)","Sensor task"])
            self.table.verticalHeader().setStyleSheet('background-color:white')   
            self.table.horizontalHeader().setStyleSheet('background-color:white') 
            for columndata in csv.reader(stream):
                column = self.table.columnCount()
                self.table.insertColumn(column)
                for row, data in enumerate(columndata):
                    if row == 6:
                        self.gyro_options = ["250","500","1000","2000"]
                        self.gyro_combo = QComboBox()
                        for x in range(0,4,1):
                            self.gyro_combo.addItem(self.gyro_options[x])
                        self.table.setCellWidget(row,column,self.gyro_combo)
                        if data == '':
                            self.table.cellWidget(row,column).setCurrentIndex(0)
                        else:
                            self.table.cellWidget(row,column).setCurrentIndex(int(data))
                    if row == 7:
                        acc_options = ["2", "4", "8", "16"]
                        self.acc_combo = QComboBox()  
                        for x in range(0,4,1):
                            self.acc_combo.addItem(acc_options[x])
                        self.table.setCellWidget(row,column,self.acc_combo)
                        if data == '':
                            self.table.cellWidget(row,column).setCurrentIndex(0)
                        else:
                            self.table.cellWidget(row,column).setCurrentIndex(int(data))
                    else:
                        item = QTableWidgetItem(data)
                        self.table.setItem(row, column, item)
            # Apaga o ip, status, acq. rate e bat level (colunas 4, 5, 6 e 7)
            for column in range(self.table.columnCount()):
                item = QTableWidgetItem('')  
                self.table.setItem(4, column, item)
                item2 = QTableWidgetItem('')  
                self.table.setItem(5, column, item2)
                item3 = QTableWidgetItem('')  
                self.table.setItem(9, column, item3)
                item4 = QTableWidgetItem('')  
                self.table.setItem(10, column, item4)
                if self.table.item(8, column).text == '':
                    self.table.item(8,column).setText('500')     
            


    def contextMenuEvent(self, event):
        self.menu = QMenu(self)
        # Connect
        connectAction = QAction('Connect', self)
        connectAction.triggered.connect(lambda: self.connect_table(event))
        self.menu.addAction(connectAction)
        # Visible
        visibleAction = QAction('Visible/Invisible', self)
        visibleAction.triggered.connect(lambda: self.visibleSlot(event))
        self.menu.addAction(visibleAction)
        # Delete
        deleteAction = QAction('Delete', self)
        deleteAction.triggered.connect(lambda: self.delete_table(event))
        self.menu.addAction(deleteAction)
        # New line
        includeAction = QAction('Include new column', self)
        includeAction.triggered.connect(lambda: self.include_table(event))
        self.menu.addAction(includeAction)
        # add other required actions
        
        self.menu.popup(QCursor.pos())
     
    def enableSlot(self, event):
        row = self.table.rowAt(event.pos().y())
        col = self.table.columnAt(event.pos().x())
        # get the selected cell
        cell = self.table.item(row, col)
    
    def visibleSlot(self, event):
        #row = self.table.rowAt(event.pos().y())
        column = self.table.currentColumn()
        self.M[column].visible = not self.M[column].visible
        if (not self.M[column].visible):
            self.M[column].color = [192, 192, 192]  # grey

    def change_page(self, index):
        self.pages.setCurrentIndex(int(index))    

    def first_config(self):
        self.networkComboBox.clear()
        self.listview.clear()
        self.ssidField.setText('')
        self.passwordField.setText('')
        self.hostnameField.setText('')
        self.currentNetworkLabel.setText('Rede atual: ')
        self.infoScanLabel.setText('')
        self.infoLabel.setText('')
        self.change_page(2)

    def colorSlot(self, event):
        # todo - implementar troca de cores
        row = self.table.rowAt(event.pos().y())
        col = self.table.columnAt(event.pos().x())
        # get the selected cell
        cell = self.table.item(row, col)

    def renameSlot(self, event):
        # get the selected row and column
        row = self.tableWidget.rowAt(event.pos().y())
        col = self.tableWidget.columnAt(event.pos().x())
        # get the selected cell
        cell = self.tableWidget.item(row, col)
        # get the text inside selected cell (if any)
        cellText = cell.text()
        # get the widget inside selected cell (if any)
        widget = self.tableWidget.cellWidget(row, col)

    def update_table_status(self):
        if (len(self.M) >= self.table.columnCount()):
            for column in range(self.table.columnCount()):
                item = QTableWidgetItem(self.M[column].status) 
                self.table.setItem(5, column, item)  # status
                formatted_acqusition_rate = "{:.0f}".format(self.M[column].acq_rate)
                item = QTableWidgetItem(formatted_acqusition_rate) 
                self.table.setItem(9, column, item)  # acq. rate
                if self.M[column].sensors.size > 0:
                    b = self.M[column].sensors.data[self.M[column].sensors.size-1,7]  # último ponto lido da bateria
                    if b>150:
                        b_level = 'Charging'
                    elif b>100:
                        b_level = '100 %'
                    elif b<0:
                        b_level = '0 %'
                    else:
                        b_level = str(int(b)) + ' %'

                    item = QTableWidgetItem(b_level) 
                    self.table.setItem(10, column, item)  # battery level
                # update row color
                # TODO - atualizar cores nas tabelas
                # for row in range(self.table.rowCount()):
                #     item = self.table.item(row, column)
                #     rgb = self.M[column].color
                #     color_t = QColor.fromRgb(int(rgb[0]), int(rgb[1]), int(rgb[2]))
                #     if item is not None:
                #         item.setForeground(color_t)
                        

    def save_data(self):
        # Salva os dados do gráfico na pasta ./Data
        comment, ok = QInputDialog.getText(self, 'Save file', 'Comment: ')
        if ok:
            start = float(self.start_value.text())
            stop = float(self.stop_value.text())
            SaveRoutine().start(self.M, comment, start, stop)


    def onselect(self, xmin, xmax):
        # Função executada quando uma área do gráfico é selecionada. 
        # Atualiza os limites de tempo para salvamento dos dados
        self.start_value.setText("{:.2f}".format(xmin))
        self.stop_value.setText("{:.2f}".format(xmax))
      
    
    def table_double_click(self, row, column):
        #print(str(row), str(column))
        #self.log_write(str(row)+','+str(column))
        return

    def output_terminal_written(self, text):
        self.logbox.appendPlainText(text)



    ## Config Inicial.

    def scan(self):
        # Limpa combo1 e listview
        self.networkComboBox.clear()
        self.listview.clear()
        # combo1 armazena as redes disponíveis e listview as redes das MuvBoxes ativas
        
        # Identifica rede wifi atual
        if USER_OS == 'Windows':
            terminalOutput = subprocess.check_output('netsh wlan show interfaces')
            terminalOutput = terminalOutput.decode('utf8', errors="ignore")
            terminalOutput = terminalOutput.split()
            for i in range(len(terminalOutput)):
                if terminalOutput[i] == 'SSID':
                    self.current_network = terminalOutput[i+2]
            print(self.current_network)
            self.currentNetworkLabel.setText('Rede atual: ' + self.current_network)
        # Lista todas as redes wifi ao alcance
            apList = subprocess.check_output(['netsh','wlan','show','network'])
            print(apList)
            apList = apList.decode('utf8', errors="ignore")
            apList = apList.replace("\r","")
            apList = apList.split()
            availableSSIDs = []
            for i in range(len(apList)):
                if apList[i] == 'SSID':
                    availableSSIDs.append(i)
        # Identifica se a rede é da MuvBox ou outra
            for i in availableSSIDs:
                network = apList[i+3]
                if network[0:6] != 'MUVBOX':    # TROCAR AQUI PARA IDENTIFICAR REDE DA MUVBOX
                    self.networkComboBox.addItem(network)
                else:
                    self.listview.addItem(network)


        if USER_OS == 'Linux':
            os.system('nmcli device wifi rescan')
            self.infoScanLabel.setText('Escaneando redes...')
            self.infoScanLabel.repaint()
            time.sleep(10)
            self.infoScanLabel.setText('Escaneamento completo!')
            self.infoScanLabel.repaint()
            terminalOutput = str(subprocess.check_output(['nmcli','device','wifi','list']))
            terminalOutput = ','.join(terminalOutput.split())
            terminalOutput = terminalOutput.split(',')
            isSSID = False
            currentNetwork = False
            listOfSSID = []
            auxList = []
            for x in range(len(terminalOutput)):
                if isSSID == True:
                    if terminalOutput[x] == 'Ad-Hoc' or terminalOutput[x] == 'Infra':    
                        isSSID = False
                        toString = ' '
                        auxList = toString.join(auxList)
                        if auxList[0:6] != 'MUVBOX':    # TROCAR AQUI PARA IDENTIFICAR REDE DA MUVBOX                            
                            self.networkComboBox.addItem(auxList)
                        else:
                            self.listview.addItem(auxList)
                        listOfSSID.append(auxList)    
                        if currentNetwork == True:                            
                            self.currentNetworkLabel.setText('Rede atual: ' + auxList)
                            self.current_network = auxList  
                            currentNetwork = False                  
                        auxList = []               
                    else:
                        if isBSSID == False:
                            auxList.append(terminalOutput[x])
                        if isBSSID == True:
                            isBSSID = False
                if terminalOutput[x] == 'BSSID':
                    haveBSSID = True
                if terminalOutput[x] == "\\n" or terminalOutput[x] == '\\n*':
                    if terminalOutput[x] == '\\n*':
                        currentNetwork = True
                    isSSID = True
                    if haveBSSID == True:
                        isBSSID = True

        self.networkComboBox.addItem('Other')  # Outra rede
        

        

    def configureNewMuvBox(self):
        try:
            if self.listview.currentRow() >=0:
                muvboxHostname = self.listview.currentItem().text()
                print(muvboxHostname)
                self.muvboxNetwork = muvboxHostname  
                
                self.hostnameField.setText(muvboxHostname)
                # Alterna para a próxima aba
                self.change_page(3)
        except OSError as msg:
                print(msg)
    
    def save(self):
        self.infoLabel.setText('Processando...')
        self.infoLabel.repaint()
        
        # Alterna para a rede da MuvBox
        try:
            # Cria o profile
            USER_OS = platform.system()
            if USER_OS == 'Windows':
                profileCommand = 'netsh wlan add profile filename=Wi-Fi-MUVBOX-profile.xml'
                print(profileCommand)
                info = subprocess.check_output(profileCommand)
                print(info)
                time.sleep(1)
                profileCommand = 'netsh wlan set profileparameter name=MUVBOX-profile SSIDname=' + self.muvboxNetwork
                print(profileCommand)
                info = subprocess.check_output(profileCommand)
                print(info)
                time.sleep(1)
            
                info = subprocess.check_output('netsh wlan connect name=MUVBOX-profile')
                print(info)
                time.sleep(1)

            if USER_OS == 'Linux':
                info = str(subprocess.check_output(['nmcli','radio','wifi']))
                print(info)
                if info == 'disabled':
                    info = str(subprocess.check_output(['nmcli','radio','wifi','on']))
                    print(info)
                    time.sleep(2.5)
                    info = str(subprocess.check_output(['nmcli','dev','wifi','connect',self.muvboxNetwork,'password','muvtecnologia']))
                    print(info)
                    time.sleep(5)
                else:                    
                    info = str(subprocess.check_output(['nmcli','dev','wifi','connect',self.muvboxNetwork,'password','muvtecnologia']))
                    print(info)
                    time.sleep(5)             

        except Exception as e:
            print(e)
            self.infoLabel.setText('Erro conectando a MuvBox! Tente novamente.')
            print('Error netsh(Windows) or nmcli(Linux)')
        time.sleep(2)

        self.TIMEOUT = 5 # 5 segundos
        self.ip = '192.168.4.1'  # Será sempre este IP
        self.port = 8001
        self._dest = (self.ip, self.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
        # Connect
        try:
            self.sock.settimeout(self.TIMEOUT)  # enable timeout
            self.sock.connect(self._dest)
            #self.sock.settimeout(None)  # disable timeout
        except:
            print('Timeout!')
            self.infoLabel.setText('Erro conectando a MuvBox! Tente novamente.')
        
        # Send command and commit
        try:
            self.sock.settimeout(self.TIMEOUT)  # enable timeout
            hostname = self.hostnameField.text()
            network = self.ssidField.text()
            password = self.passwordField.text()
            
            muvboxCommand = '{\"command\": \"set_access_point\",\"hostname\": \"' + hostname + '\",\"ssid\": \"' + network + '\",\"password\": \"' + password + '\"}'
            print(muvboxCommand)
            self.sock.sendto(muvboxCommand.encode(), self._dest)
            time.sleep(2)
            self.sock.sendto('{\"command\": \"commit\"}'.encode(), self._dest)
            time.sleep(2)
            self.infoLabel.setText('MuvBox configurada com sucesso! Clique no botão fechar no canto superior direito da tela.')
        except:
            print('Timeout!')
            self.infoLabel.setText('Erro enviando as configurações.')

        
        
        # Fecha o socket
        self.sock.close()
        try:
            # Retorna para rede original
            if USER_OS == 'Windows':
                info = subprocess.check_output('netsh wlan connect name=' + self.current_network)
                print(info)
            if USER_OS == 'Linux':
                info = subprocess.check_output(['nmcli','dev','wifi','connect',self.current_network])
                print(info)
        except:
            print('Erro retornando para rede original!')
            self.infoLabel.setText('MuvBox configurada com sucesso! Clique no botão fechar no canto superior direito da tela.')

    def changeHostnameField(self):
        if self.networkComboBox.currentText() == 'Other':
            self.ssidField.setText('')
        else:
            self.ssidField.setText(self.networkComboBox.currentText())



def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle(QtWidgets.QStyleFactory.create('Fusion'))
    main = MainWindow()
    
    main.connect_all_button.clicked.connect(main.connect_n)
    main.disconnect_all_button.clicked.connect(main.disconnect)
    main.start_all_button.clicked.connect(main.start)
    main.stop_all_button.clicked.connect(main.stop_reading)
    main.config_button.clicked.connect(lambda: main.change_page(1))
    main.back_button.clicked.connect(lambda: main.change_page(0))
    main.first_config_button.clicked.connect(main.first_config)


    main.scanButton.clicked.connect(main.scan)
    main.configButton.clicked.connect(main.configureNewMuvBox)
    main.saveButton.clicked.connect(main.save)
    main.backButton.clicked.connect(lambda: main.change_page(2))
    main.networkComboBox.currentIndexChanged.connect(main.changeHostnameField)  
    main.cancelButton.clicked.connect(lambda: main.change_page(0))
    main.closeButton.clicked.connect(lambda: main.change_page(0))
    
    
    main.save_table_button.clicked.connect(main.save_table)
    main.update_graph_button.clicked.connect(main.stop_reading)
    main.save_data_button.clicked.connect(main.save_data)
    main.sincronizar_button.clicked.connect(main.sincronizar)
    
    span0 = SpanSelector(main.ax2[0], main.onselect, 'horizontal', useblit=True, span_stays=True,   
                    rectprops=dict(alpha=0.5, facecolor='green'))
    
    span1 = SpanSelector(main.ax2[1], main.onselect, 'horizontal', useblit=True, span_stays=True,   
                    rectprops=dict(alpha=0.5, facecolor='green'))
    
    span2 = SpanSelector(main.ax2[2], main.onselect, 'horizontal', useblit=True, span_stays=True,   
                    rectprops=dict(alpha=0.5, facecolor='green'))
    
    
    main.table.customContextMenuRequested.connect(main.contextMenuEvent)
    main.table.cellDoubleClicked.connect(main.table_double_click)
    
    main.show()

    sys.exit(app.exec_())

if __name__ == '__main__':         
    main()
    