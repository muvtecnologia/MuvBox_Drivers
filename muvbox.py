""" 
    ** MuvBox v. 0.9 **

    This is the interface module for MuvBox IMU.

    MuvBox module contains a class for connecting and reading data from a MuvBox. 
    Six degrees-of-freedom are present: 3D accelerometer and 3D gyroscope.
    
    For instantiating a MuvBox, type M = MuvBox(). Use M.connect() for wifi connect and 
    data reading. Hostname (M.name) or IP (M.ip) must be set before connecting. Default port is 8001.

    MuvBox version is collected direcly from the MuvBox. It is not necessary to inform from the application.
    Current driver supports version 'Super Alpha 1.257' only.

    The function connect() establishes a connection with a MuvBox. To start a new thread and continuosly acquire sensor data 
    execute the command start(). Command stop() finishes data acquisition, but stays connected. To broke connection execute
    the command disconnect().
    If a network error occur, thread stops automatically and MuvBox is disconnected.

    Sensors scales can be set by user. Example:
    M.ACCSCALE = 0
    M.GYROSCALE = 1
    If not set, default values will be used.
    
    Features:
     - Exclusive for firmware version FM10V000.xxx: MuvBox M1, 6-DOF, vector
     - Include commands
     - Include battery reading
     - Thread is deamonic (it automatically ends when program exit)
     - Redesigned state machine 
     - Use numpy array
     - 

    Example:

    from muvbox import *
    M = MuvBox()
    M.version = 'Super Alpha 1.257'
    M.name = 'MuvBox14'
    M.port = 8001
    M.connect()
    M.start()
    M.stop()
    M.disconnect()

    Data is stored in 'sensors.data' matriz and can be accessed while MuvBox is running
    or after stop reading. Each column corresponds to a sensor and lines to sampled data.
    For example:

    time = M.sensors.data[:,0]
    accx = M.sensors.data[:,1]
    accy = M.sensors.data[:,2]
    accz = M.sensors.data[:,3]
    gyrx = M.sensors.data[:,4]
    gyry = M.sensors.data[:,5]
    gyrz = M.sensors.data[:,6]
    bat = M.sensors.data[:,7]

    acc is in g (~9.81 m/s^2) and gyr in °/s. Time is in seconds

    Battery level information is stored in column 'bat', in %.

    Euler angles are calculated if flag 'calculate_quaternion' is set. 
    Values are stored in 'angles.data' matrix. Quaternions are stored in 'Q.data' matrix.
    Example:

    yaw = M.angles.data[:,1]
    pitch = M.angles.data[:,2]
    roll = M.angles.data[:,3]
    Q = M.Q.data[:,1:5]

    
"""
import numpy as np
import pandas as pd

import struct
#import time
import datetime

import socket
from socket import AF_INET, SOCK_DGRAM

from threading import Thread, current_thread

from ahrs.filters import Madgwick
from ahrs import Quaternion
import ahrs

import json

class MuvBox():

    PC_DRIVER_VERSION = '0.9'   # Version of Python MuvBox driver

    def __init__(self, m:int=0, ip:str='192.168.0.1', version:str='FM10V000.950', port:int=8001):
        self.muvbox_number = m  # MuvBox number in the application
        self.ip = ip            # IP address
        self.port = port        # Port number
        self._dest = ''         # Internal tuple for wifi connection 
        self.mac = ''           # MuvBox mac address
        self.firmware_version_full = version # MuvBox firmware version informed by application
        self.firmware_version = int(self.firmware_version_full[5:8])
        
        self.logbox = None

        # Data packet
        self.WINDOWS_SIZE = 24*20  # read 20 samples at once
        self.STEP = 24             # number of bits per packet
        self.PROTOCOL = 'TCP'      # Only TCP implemented
        self.TIMEOUT = 5           # Maximum wifi waiting time, in seconds

        # Sensors scales
        self.ACQ_FREQ = 1000   # nominal acquisition rate (samples/s)
        self.ACCSCALE = 1      # scale range (0 to 3)
        self.GYROSCALE = 0     # scale range (0 to 3)
        self._GSCALE = 2  # Max G (calculated from self.GYROSCALE)
        self._DEGSCALE = 500   #  (calculated from self.ACCSCALE)
        self.TO_DPS = (2**(16-1))/self._DEGSCALE # Gyroscope scale
        self.TO_G = (2**(16-1))/self._GSCALE   # Accelerometer scale
        self.TIMESCALE = 1/1000000  # 1 us
        self.AXIS = 3
        self.ACC_WORD_SIZE = 16 # Number of bits of accelerometer register
        self.GYR_WORD_SIZE = 16 # Number of bits of gyroscope register
        self.GRAVITY = 9.807 # m/s^2

        # Battery limits
        self.BAT_VMAX = 4      # Volts 
        self.BAT_VMIN = 3.6    # Volts

        self.free_heap = 0     # free MuvBox memory
        self.sensor_task = ''  # status of MuvBox sensor chip

        self.mag_present = False  # Indicates the presence of magnetometer
        
        self.location = ''       # Location of MuvBox in the application (free text)
        self.name = ''           # MuvBox hostname
        
        self.status = 'Offline'  # Current MuvBox status
        self.acq_rate = 0        # Current MuvBox instant acquisition rate
        
        self.color = [0,0,0]     # black
        
        self.rtc0 = 0.0          # Initial time

        self.sock = None         # WIFI Socket

        ## Data vectors
        # Units:
        # time: s (seconds)
        # acc: g (gravity)
        # gyr: °/s (degrees per second)
        # bat: % (percentage)
        # yaw, pitch, roll: ° (degrees)
        self.sensors = MuvBox_DataFrame(8)  # time, accx, accy, accz, gyrx, gyry, gyrz, bat
        self.angles = MuvBox_DataFrame(4)   # time, yaw, pitch, row
        self.Q = MuvBox_DataFrame(5)        # time, a, b, c, d   (Quaternion = a + b*i + c*j + d*k)

        self.madgwick = ahrs.filters.Madgwick()   
        self.calculate_quaternion = False
        self.marg = False

        self.t = Thread() # thread for reading data
        
        # Command flags
        self.stop_reading = False
        self.ajustar_rtc0 = False
        self.visible = True
        self.reading_values = False  # Indica que os dados estão sendo lidos do socket
        self.state = 0

    def print_log(self, message):
        if self.logbox != None:
            self.logbox.append(message)
        else:
            print(message)

    def connect(self):
        
        if self.state == 0:
            self.stop_reading = False
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Start _do_connect')
            self.status = 'Connecting'
            got_ip = False
            try:
                self.ip = socket.gethostbyname(self.name)
                got_ip = True
            except:
                self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Could not find hostname ' + self.name)
            if not got_ip:
                try:
                    self.ip = socket.gethostbyname(self.name+'.local')
                    got_ip = True
                except:
                    self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Could not find hostname ' + self.name + '.local')
            if got_ip:
                try:
                    self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + ' IP = ' + self.ip)
                    self._dest = (self.ip, self.port)
                    self.acq_rate = 0
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
                    self.sock.settimeout(self.TIMEOUT)  # enable timeout
                    self.sock.connect(self._dest)
                    self.sock.settimeout(None)  # disable timeout
                    self.status = 'Online'
                    self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'SUCCESSFULL CONNECTED TO ' + self.ip)
                    self.command__system_info()  # Read MuvBox info, including firmware version
                    self.setup()   # Configure environment according to firmware version
                    self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'End do_connect')
                    self.state = 2
                except socket.timeout:
                    self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + '[MuvBox Error] Timeout: cannot connect to ' + self.ip)
                    self.status = 'Error'
                except OSError as msg:
                    self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + '[MuvBox Error]: cannot connect to ' + self.ip + ' - Message: ' + str(msg))
                    self.status = 'Error'
                except:
                    print('Erro desconhecido')
        

    def stop(self):
        if self.state == 4 or self.state == 3:
            self.status = 'Stopping'
            self.stop_reading = True
            cont = 0
            while self.reading_values: # Fica preso aqui enquanto socket estiver sendo lido
                print('Reading socket')
                cont = cont + 1
                if cont == 99999999:
                    print("No response from socket.")
                    break
                
            self.state = 5
            print(str(self.muvbox_number) + ': state 5')
            print('iniciando stop_reading')
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Start stop_reading')
            self.status = 'Online'
            self.command__stop_transmission()
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'End stop_reading')
            print('finalizando stop_reading') 
            self.sensors.finalize()
            self.Q.finalize()
            self.angles.finalize()
            print(len(self.sensors.data),' dados lidos.')
            self.state = 2  # Retorna ao estado 2


    def start(self):
        if self.state==2:
            print(str(self.muvbox_number) + 'state 3')
            self.stop_reading = False
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Start start_reading')
            try:
                self.command__start_sensor()
                self.convert_scale()
                self.clear()  # clear data
                self.state = 3
                print(str(self.muvbox_number) + 'state 3')
                if not self.t.is_alive():
                    self.stop_reading = False
                    self.ajustar_rtc0 = True
                    self.t = Thread(target=self.thread_reading, daemon=True)
                    self.t.start()
                    self.status = 'Running'
                else:
                    self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Already connected - ' + self.ip + ' ')
            except:
                print('Erro start')
                self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Error starting - ' + self.ip + ' ')
                self.state = 2
                print('state 2')
            

    def thread_reading(self):
        # State 4
        self.state = 4
        print(str(self.muvbox_number) + 'state 4')
        try:
            while not self.stop_reading:
                self.sock.settimeout(self.TIMEOUT)  # enable timeout
                self.reading_values = True # Flag que indica que os dados estão send lidos do socket
                self.read_values()
                self.reading_values = False
                self.sock.settimeout(None)  # disable timeout
                self.updateQuaternion()
                if (self.ajustar_rtc0 == True):
                    if self.sensors.size>0:
                        self.rtc0 = self.sensors.data[self.sensors.size-1, 0]
                        self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' +  " - rtc0: " + str(self.rtc0))
                        self.clear()  # Limpa os deques
                        self.ajustar_rtc0 = False
            print('End thread_reading')
        except socket.timeout:
            self.reading_values = False
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + '[MuvBox Error] Timeout: cannot connect to ' + self.ip)
            self.status = 'Error'
        except OSError as msg:
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + '[MuvBox Error]: cannot connect to ' + self.ip + ' - Message: ' + str(msg))
            self.status = 'Error'
        except:
            print('Erro desconhecido')

    def disconnect(self):
        self.sock.close()
        self.status = 'Offline'
        self.acq_rate = 0
        self.stop_reading = False
        self.ajustar_rtc0 = False
        self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Connection closed ' + self.ip)
        self.state = 0


    def sync(self):
        self.ajustar_rtc0 = True
   

    def setup(self):
        # Configure environment according to firmware version
        # firmware_version must be known
        if self.firmware_version==0:
            # Configure MuvBox
            self.mag_present = False
            self.marg = False
            self.STEP = 24                     # number of bits per packet
            self.WINDOWS_SIZE = self.STEP*150  # MuvBox sends 150 samples per packet. Read 300 samples at once.
            self.PROTOCOL = 'TCP'
            self.convert_scale()               # calculate sensor scales
            self.TIMESCALE = 1/1000000         # 1 us
            self.AXIS = 3
            self.TIMEOUT = 5                   # Maximu wifi waiting time, in secondes
            self.BAT_VMAX = 4                  # Voltage for 100% battery
            self.BAT_VMIN = 3.6                # Voltage for 0% battery
            self.clear()                       # Reset all vectors
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Setup done for MuvBox version ' + self.firmware_version_full)
        else:
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Firmware version ' + self.firmware_version_full + ' unknown. Setup not done.')

    def read_values(self):
        data = self.sock.recv(self.WINDOWS_SIZE)  # TCP
        
        while (len(data) != self.WINDOWS_SIZE):
            data += self.sock.recv(self.WINDOWS_SIZE-len(data))  # TCP

        if (data[0] == 00 and data[-1]==255):
            self.appendFromWindow(data)
        else:
            self.print_log(data)
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Packets lost - synchronization error')
        

    # Função secundária
    def shortToLong(self, shortInt):
        longInt = 0
        for i in range(len(shortInt)):
            mult = 256 ** i
            longInt += shortInt[i] * mult
        return longInt

    def appendSensors(self, timeRtc,  accLocal,  gyroLocal, batteryLocal):
        
        batteryLocal = batteryLocal*1E-3  # transform to Volts
        batteryLocalpercent = int(100*((batteryLocal-self.BAT_VMIN)/(self.BAT_VMAX-self.BAT_VMIN)))

        self.sensors.append([timeRtc*self.TIMESCALE,
                        accLocal[0] / self.TO_G, 
                        accLocal[1] / self.TO_G, 
                        accLocal[2] / self.TO_G,
                        gyroLocal[0] / self.TO_DPS,
                        gyroLocal[1] / self.TO_DPS,
                        gyroLocal[2] / self.TO_DPS,
                        batteryLocalpercent])


        
    def appendFromSliced(self, v):
        
        if self.firmware_version == 0:
            # In this firmware version packet has 24 bytes (24 positions in v vector), organized as follows:
            # 0 -> start byte (must be 0)
            # 1 to 8 -> time
            # 9 to 10 -> ax
            # 11 to 12 -> ay
            # 13 to 14 -> az
            # 15 to 16 -> gx
            # 17 to 18 -> gy
            # 19 to 20 -> gz
            # 21 to 22 -> battery level
            # 23 -> end byte (must be 255)
            
            if (v[0] == 00 and v[-1]==255) and len(v)==24: # Verify packet integrity
                time = self.shortToLong(v[1:9])
                accLocal = []
                gyroLocal = []
                for i in range(self.AXIS):
                    accLocal.append(struct.unpack(
                        "<h", v[9 + 2 * i: 9 + 2 * i + 2])[0])
                    gyroLocal.append(struct.unpack(
                        "<h", v[15 + 2 * i: 15 + 2 * i + 2])[0])
                batteryLocal =   struct.unpack("<h", v[21:23])[0]  
                self.appendSensors(time, accLocal, gyroLocal, batteryLocal)
            else:
                self.print_log(v)
                self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Error - packet corrupted.')
        else: 
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Firmware version ' + self.firmware_version_full + ' unknown. Data not read.')

    def appendFromWindow(self, v):
        for i in range(self.WINDOWS_SIZE//self.STEP):
            #print(i)
            self.appendFromSliced(v[self.STEP*i: self.STEP*(i+1)])
            

    def updateQuaternion(self):
        # Cálculo do Quaternion

        if self.calculate_quaternion:
            
            if self.sensors.size > 0:
                # Valor inicial do quaternion e dos ângulos
                if self.Q.size == 0:
                    self.Q.append([self.sensors.data[0,0], 1, 0, 0, 0])
                    self.angles.append([self.sensors.data[0,0], 0, 0, 0])

                diff_size = self.sensors.size - self.Q.size
                
                ## Cada leitura do socket recebe n valores de rtc
                for i in range(diff_size, 0, -1):
                    acc = self.sensors.data[self.sensors.size - i,1:4]
                    acc = acc*self.GRAVITY
                    gyr = self.sensors.data[self.sensors.size - i,4:7]
                    gyr = gyr*np.pi/180

                    # TODO: implementar filtro marg com mag - ajustar tamanho do vetor mag
                    t_atual = self.sensors.data[self.sensors.size - i, 0]
                    t_anterior = self.sensors.data[self.sensors.size - i - 1, 0]
                    self.madgwick.Dt =  t_atual - t_anterior 
                    last_Q = self.Q.data[self.Q.size-1, 1:5]
                    if self.marg:  # default is False
                        Q = self.madgwick.updateMARG(Quaternion(last_Q), gyr=gyr, acc=acc, mag=gyr) # TODO: ajustar vetor magnetometro
                    else:
                        Q = self.madgwick.updateIMU(Quaternion(last_Q), gyr=gyr, acc=acc)
                    self.Q.append([t_atual, Q.w, Q.x, Q.y, Q.z])
                    angles = Quaternion(Quaternion(Q).conj).to_angles()*180/np.pi
                    self.angles.append([t_atual, angles[2], angles[1], angles[0]])
                    
    
    def clear(self):
        # Clear and reconstructs all vectors
        self.sensors.clear()
        self.angles.clear()
        self.Q.clear()


    def convert_scale(self):
        # Calculate self.GSCALE and self.DEGSCALE from self.ACCSCALE and self.GYROSCALE
        if self.firmware_version == 0:
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Start scale procedure.')
            self.ACC_WORD_SIZE = 16 # Number of bits of accelerometer register
            self.GYR_WORD_SIZE = 16 # Number of bits of gyroscope register
            
            if self.ACCSCALE==0:   # 2g scale
                self._GSCALE = 2
            elif self.ACCSCALE==1: # 4g scale
                self._GSCALE = 4   
            elif self.ACCSCALE==2: # 8g scale
                self._GSCALE = 8   
            elif self.ACCSCALE==3: # 16g scale
                self._GSCALE = 16  
            else:
                self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Warning: ACC Scale out of range. Scale not set.')
        
            if self.GYROSCALE==0:      # 250°/s scale
                self._DEGSCALE = 250
            elif self.GYROSCALE==1:    # 500°/s scale
                self._DEGSCALE = 500
            elif self.GYROSCALE==2:    # 1000°/s scale
                self._DEGSCALE = 1000
            elif self.GYROSCALE==3:    # 2000°/s scale
                self._DEGSCALE = 2000
            else:
                self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Warning: GYR Scale out of range. Scale not set.')
            
            self.TO_DPS = (2**(self.GYR_WORD_SIZE-1))/self._DEGSCALE # 65.536
            self.TO_G = (2**(self.ACC_WORD_SIZE-1))/self._GSCALE  
            self.print_log ('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Scale procedure done. Using DEGSCALE=' + str(self._DEGSCALE) + ' GSCALE=' + str(self._GSCALE))
        else:
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Firmware version ' + self.firmware_version_full + ' unknown. Scale not set.')
        

    def command__system_info(self):
        data = None
        try:
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Start command__system_info')
            self.sock.settimeout(self.TIMEOUT)  # enable timeout
            self.sock.sendto('{\"command\": \"system_info\"}'.encode(), self._dest)
            data = self.sock.recv(1024)
            self.sock.settimeout(None)  # disable timeout
            d2 = data.decode("utf8")
            parsed_data=json.loads(d2) # decode json data
            self.free_heap = parsed_data.get('free_heap')
            self.mac = parsed_data.get('mac')
            self.firmware_version_full = parsed_data.get('firmware')
            self.firmware_version = int(self.firmware_version_full[5:8])
            self.sensor_task = parsed_data.get('sensor_task')
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'free heap: ' + str(self.free_heap) + ' mac: ' + str(self.mac) + ' firmware: ' + str(self.firmware_version) + ' sensor task: ' + str(self.sensor_task))
            self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'command__system_info successful')
        except:
            self.print_log(data)
            self.print_log('Error command system_info')
        
    def command__start_sensor(self):
        self.print_log ('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Start command__start_sensor')
        message = '{\"command\": \"start_sensor\",\"freq\": ' + str(int(self.ACQ_FREQ)) + ',\"GYRO\": ' + str(self.GYROSCALE) + ',\"ACCEL\": ' + str(self.ACCSCALE) + '}'
        #print(message)
        self.sock.settimeout(self.TIMEOUT)  # enable timeout
        self.sock.sendto(message.encode(), self._dest)
        self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'start_sensor command successful')
        self.sock.settimeout(None)  # disable timeout
        
        
    def command__stop_transmission(self):
        self.print_log ('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Start command__stop_transmission')
        self.sock.settimeout(self.TIMEOUT)  # enable timeout
        self.sock.sendto('{\"command\": \"stop_transmission\"}'.encode(), self._dest)
        self.print_log('* ' + self.name +  ' #' + str(self.muvbox_number) + ': ' + 'Stop_transmission command successful')
        self.sock.settimeout(None)  # disable timeout
        

class MuvBox_DataFrame:

    def __init__(self, c=8):
        self.cols = c
        self.capacity = 4096
        self.data = np.empty((self.capacity, self.cols))
        self.size = int(0)

    def append(self, x):
        if self.size == len(self.data):
            self.capacity *= 2
            newdata = np.empty((self.capacity,self.cols))
            newdata[:self.size] = self.data
            self.data = newdata
            print('Resizing memory for', len(self.data),' samples')

        for i in range(len(x)):
            self.data[self.size,i] = x[i]
        self.size += 1

    def finalize(self):
        data = self.data[:self.size]
        self.data = np.reshape(data, newshape=(self.size, self.cols))

    def clear(self):
        self.capacity = 4096
        self.data = np.empty((self.capacity, self.cols))
        self.size = 0
        


class SaveRoutine:

    __instance = None

    def __new__(cls):
        if SaveRoutine.__instance is None:
            SaveRoutine.__instance = object.__new__(cls)
            SaveRoutine.__instance.PATH = "./data/"
            SaveRoutine.__instance.__saving = False
            SaveRoutine.__instance.__start = False
            SaveRoutine.__instance.__current_file = ""
            SaveRoutine.__instance.__comment = []
            SaveRoutine.__instance.__name = []
            SaveRoutine.__instance.__datetime2 = ""
            SaveRoutine.__instance.__start_point = 0
            SaveRoutine.__instance.__stop_point = 0
            SaveRoutine.__instance.__rtc0 = []
            SaveRoutine.__instance.M = []
        return SaveRoutine.__instance

    def __append_points(self, m):
        with open(self.__current_file, "a") as f:
            # Discover first and last elements to be saved:
            print('m = ' + str(m))
            time2 = self.M[m].sensors.data[:,0]
            print('rtc0 = ' + str(self.M[m].rtc0))
            #for i in range(len(time2)):
            #    time2[i] = time2[i] - self.M[m].rtc0
            time2 = time2 - self.M[m].rtc0
            if (self.__start_point>=0):
                try:
                    first = next(x[0] for x in enumerate(time2) if x[1] > self.__start_point)
                except:
                    first = 0
            else:
                first = 0
            
            if (self.__stop_point<=time2[-1]):
                try:
                    last =  next(x[0] for x in enumerate(time2) if x[1] > self.__stop_point)
                except:
                    last = 0
            else:
                last = len(time2) - 1

            s = ""

            for i in range(first, last, 1):
                s += str(time2[i])
                for j in range(6):
                    s += ";{}".format(self.M[m].sensors.data[i,j+1])
                #for j in range(3):
                #    s += ";{}".format(self.M[m].gyroDeg[j][i])
                #for j in range(3):
                    #s += ";{}".format(self.M[m].magNorm[j][-1]) - TODO: incluir magnetometro
                #    s += ";{}".format(0)
                s += "\n"

            f.write(s)
            print('Data saved to ' + self.__current_file)
            
    def __start_file(self, m):
        name = self.M[m].name
        self.__current_file = SaveRoutine().PATH + self.__datetime2 + "_" + self.__comment + "_" + name + ".csv"
        with open(self.__current_file, "w") as f:
            s = name + ";" + self.M[m].location + ";" + self.__comment; self.__datetime2
            s += "\n"
            s += "time"
            axis = ["x", "y", "z"]
            types = ["acc_", "gyr_"]
            for i in types:
                for j in axis:
                    s += ";{}{}".format(i, j)
            s += "\n"
            f.write(s)
    
    def start(self, M=[], comment="", start_point=0, stop_point=0):
        self.M = M
        self.__comment = comment
        self.__start_point = start_point
        self.__stop_point = stop_point
        self.__datetime2 = str(datetime.datetime.now()).replace(" ", "_").replace(":","-")
        for m in range(len(self.M)):
            if (M[m].sensors.size > 0):
                self.__start_file(m)
                self.__append_points(m)
        
    def stop(self):
        self.__start = False
        self.__saving = False



