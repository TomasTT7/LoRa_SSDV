"""
LoRa Gateway for parsing, decoding and uploading High Altitude Balloon telemetry and SSDV images.

PRO MINI -> GATEWAY protocol
    p:    packet
    t:    telemetry
    j:    ssdv packet
    m:    LoRa mode
    i:    explicit/implicit
    b:    bandwidth
    e:    error coding
    s:    spreading factor
    l:    low data rate
    f:    frequency
    r:    current rssi
    n:    packet SNR
    c:    packet RSSI
    q:    frequency error
    w:    payload length
    o:    RX on/off
    z:    CRC error

GATEWAY -> PRO MINI protocol
    m   LoRa mode
    f   frequency
    l   payload length
    o   RX on/off
"""

from Tkinter import *
from PIL import ImageTk, Image
import ttk
import sys
import glob
import serial
import datetime
import subprocess
import math
import os
import tkFont
import threading
import couchdbkit
import couchdbkit.exceptions
import json
import time
import strict_rfc3339
import base64
import hashlib
import restkit
import restkit.errors
import Queue
import requests


LoRa_mode = {'Mode 0':'0', 'Mode 1':'1', 'Mode 2':'2', 'Mode 3':'3', 'Mode 4':'4', 'Mode 5':'5', 'Mode 6':'6', 'Mode 7':'7', 'Mode 8':'8', 'Mode 9':'9'}
LoRa_header_mode = {'Mode 0':'explicit', 'Mode 1':'implicit', 'Mode 2':'explicit', 'Mode 3':'explicit', 'Mode 4':'implicit', 'Mode 5':'explicit', 'Mode 6':'implicit', 'Mode 7':'explicit', 'Mode 8':'implicit', 'Mode 9':'implicit'}
LoRa_bandwidth = {'Mode 0':'20.8kHz', 'Mode 1':'20.8kHz', 'Mode 2':'62.5kHz', 'Mode 3':'250kHz', 'Mode 4':'250kHz', 'Mode 5':'41.7kHz', 'Mode 6':'41.7kHz', 'Mode 7':'20.8kHz', 'Mode 8':'62.5kHz', 'Mode 9':'500kHz'}
LoRa_error_coding = {'Mode 0':'4/8', 'Mode 1':'4/5', 'Mode 2':'4/8', 'Mode 3':'4/6', 'Mode 4':'4/5', 'Mode 5':'4/8', 'Mode 6':'4/5', 'Mode 7':'4/5', 'Mode 8':'4/5', 'Mode 9':'4/5'}
LoRa_spreading_factor = {'Mode 0':'11', 'Mode 1':'6', 'Mode 2':'8', 'Mode 3':'7', 'Mode 4':'6', 'Mode 5':'11', 'Mode 6':'6', 'Mode 7':'7', 'Mode 8':'6', 'Mode 9':'6'}
LoRa_low_data_rate = {'Mode 0':'1', 'Mode 1':'0', 'Mode 2':'0', 'Mode 3':'0', 'Mode 4':'0', 'Mode 5':'0', 'Mode 6':'0', 'Mode 7':'0', 'Mode 8':'0', 'Mode 9':'0'}


"""
    Function taken from:
    https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
"""
def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


class Application(Frame):


    def __init__(self, master, serialPort):
        """ Initialize the Frame """
        Frame.__init__(self, master)

        # serial port
        self.serialPort = serialPort
        self.activePort = ''

        # ssdv
        self.imgList = []   # lists the received images e.g. ['2018-02-09_17-27-19_TT7L_0001', '2018-02-09_17-27-51_TT7L_0002']
        self.pktList = []   # lists the received number of packets for each image e.g. [4, 3]
        self.typeList = []  # lists the image types (FEC, NO-FEC) of each received image e.g. ['g', 'g']

        # receiver/listener
        self.rxLat_val = 0.0
        self.rxLon_val = 0.0
        self.rxAlt_val = 0
        self.rxCall_val = ""
        self.rxLat_ok = 0
        self.rxLon_ok = 0
        self.rxAlt_ok = 0
        self.rxCall_ok = 0
        self.rxChangedInfo = 0
        self.rxChangedTelem = 0

        # info
        self.losLast = 0.0
        self.distanceLast = 0.0
        self.azimuthLast = 0.0
        self.elevationLast = 0.0

        # serial port parser
        self.lastPktType = 'p'
        self.lastPkt = ''
        self.lastPktTime = datetime.datetime(1,1,1)

        # ssdv window
        self.ssdv_window = Toplevel(self)
        self.ssdv_window.protocol('WM_DELETE_WINDOW', self.remove_image_window)
        self.ssdv_window.title("SSDV Image")
        self.ssdv_window.geometry("320x240")
        self.ssdv_window.withdraw()
        self.ssdv_canvas = Canvas(self.ssdv_window, width=320, height=240)

        # uploader
        self._queue = Queue.Queue()
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.thread.start()
        
        self._lock = threading.RLock()
        self._latest = {}
        self._max_merge_attempts = 20
        
        server = couchdbkit.Server("http://habitat.habhub.org")
        self.db = server['habitat']

        self.ssdvServer = "http://ssdv.habhub.org/api/v0/packets"
        
        #
        self.RXonoff = '0'
        self.grid()
        self.create_widgets()
        self.status("Gateway started.")


    def create_widgets(self):
        """ Create several widgets """
        self.port_label = Label(self, text="Port:")
        self.port_label.grid(row=0, column=0, sticky=W)
        
        self.port_box_value = StringVar()
        self.port_box = ttk.Combobox(self, textvariable=self.port_box_value, width=7)
        self.port_box['values'] = (serial_ports())
        self.port_box.current(0)
        self.port_box.bind("<<ComboboxSelected>>", self.update_buttons)
        self.port_box.grid(row=0, column=1, sticky=W+E, padx=15)

        self.port_button = Button(self, text="Refresh", command=self.refresh_ports)
        self.port_button.grid(row=0, column=3, sticky=W+E)

        self.port_button2 = Button(self, text="Connect", command=self.connect_port)
        self.port_button2.grid(row=0, column=2, sticky=W+E)

        self.mode_label = Label(self, text="Mode:")
        self.mode_label.grid(row=1, column=0, sticky=W)

        self.mode_box_value = StringVar()
        self.mode_box = ttk.Combobox(self, textvariable=self.mode_box_value, state=DISABLED, width=7)
        self.mode_box['values'] = ('Mode 0', 'Mode 1', 'Mode 2', 'Mode 3', 'Mode 4', 'Mode 5', 'Mode 6', 'Mode 7', 'Mode 8', 'Mode 9')
        self.mode_box.current(0)
        self.mode_box.grid(row=1, column=1, sticky=W+E, padx=15)

        self.mode_button = Button(self, text="Select", command=self.change_mode, state=DISABLED)
        self.mode_button.grid(row=1, column=2, sticky=W+E, columnspan=1)
        
        self.payload_label = Label(self, text="Payload Length:")
        self.payload_label.grid(row=3, column=0, sticky=W)
        
        self.payload_in = Entry(self, state=DISABLED, width=14)
        self.payload_in.grid(row=3, column=1, sticky=W+E, padx=15)
        
        self.payload_button = Button(self, text="Select", command=self.change_length, state=DISABLED)
        self.payload_button.grid(row=3, column=2, sticky=W+E, columnspan=1)

        self.freq_label = Label(self, text="Frequency (MHz):")
        self.freq_label.grid(row=2, column=0, sticky=W)
        
        self.freq_in = Entry(self, state=DISABLED, width=10)
        self.freq_in.grid(row=2, column=1, sticky=W+E, padx=15)
        
        self.freq_button = Button(self, text="Select", command=self.change_freq, state=DISABLED)
        self.freq_button.grid(row=2, column=2, sticky=W+E, columnspan=1)

        self.upload_buttonVar = IntVar()
        self.upload_button = Checkbutton(self, state=ACTIVE, text="Upload Data to Habitat", variable=self.upload_buttonVar, command=self.update_listener)
        self.upload_button.grid(row=4, column=0, sticky=W, columnspan=2, padx=20)

        self.save_buttonVar = IntVar(value=1)
        self.save_button = Checkbutton(self, state=ACTIVE, text="Save Data to File", variable=self.save_buttonVar)
        self.save_button.grid(row=5, column=0, sticky=W, columnspan=2, padx=20)

        self.img_buttonVar = IntVar()
        self.img_button = Checkbutton(self, state=ACTIVE, text="Show Latest Image", variable=self.img_buttonVar, command=self.show_image)
        self.img_button.grid(row=6, column=0, sticky=W, columnspan=2, padx=20)

        self.onoff_button = Button(self, text="ON/OFF", command=self.change_onoff, state=DISABLED)
        self.onoff_button.grid(row=5, column=2, sticky=W+E, columnspan=2)

        self.onoff_label = Label(self, text="RX OFF", state=DISABLED, font='Tahoma 8 bold')
        self.onoff_label.grid(row=6, column=2, sticky=S, columnspan=2)

        self.actualFreq_label = Label(self, text="LoRa Frequency:")
        self.actualFreq_label.grid(row=0, column=5, sticky=W)
        self.actualFreqVal_label = Label(self, text="", width=12, anchor=E)
        self.actualFreqVal_label.grid(row=0, column=6, sticky=E)

        self.actualMode_label = Label(self, text="LoRa Mode:")
        self.actualMode_label.grid(row=1, column=5, sticky=W)
        self.actualModeVal_label = Label(self, text="")
        self.actualModeVal_label.grid(row=1, column=6, sticky=E)

        self.Header_label = Label(self, text="Header Mode:")
        self.Header_label.grid(row=2, column=5, sticky=W)
        self.HeaderVal_label = Label(self, text="")
        self.HeaderVal_label.grid(row=2, column=6, sticky=E)

        self.Bandwidth_label = Label(self, text="Bandwidth:")
        self.Bandwidth_label.grid(row=3, column=5, sticky=W)
        self.BandwidthVal_label = Label(self, text="")
        self.BandwidthVal_label.grid(row=3, column=6, sticky=E)

        self.Error_label = Label(self, text="Error Coding:")
        self.Error_label.grid(row=4, column=5, sticky=W)
        self.ErrorVal_label = Label(self, text="")
        self.ErrorVal_label.grid(row=4, column=6, sticky=E)

        self.Spreading_label = Label(self, text="Spreading Factor:")
        self.Spreading_label.grid(row=5, column=5, sticky=W)
        self.SpreadingVal_label = Label(self, text="")
        self.SpreadingVal_label.grid(row=5, column=6, sticky=E)

        self.Rate_label = Label(self, text="Low Data Rate:")
        self.Rate_label.grid(row=6, column=5, sticky=W)
        self.RateVal_label = Label(self, text="")
        self.RateVal_label.grid(row=6, column=6, sticky=E)

        self.freqErr_label = Label(self, text="Frequency Error:")
        self.freqErr_label.grid(row=0, column=8, sticky=W)
        self.freqErrVal_label = Label(self, text="", width=10, anchor=E)
        self.freqErrVal_label.grid(row=0, column=9, sticky=E)

        self.snr_label = Label(self, text="Packet SNR:")
        self.snr_label.grid(row=1, column=8, sticky=W)
        self.snrVal_label = Label(self, text="")
        self.snrVal_label.grid(row=1, column=9, sticky=E)

        self.snrest_label = Label(self, text="Estimated SNR:")
        self.snrest_label.grid(row=2, column=8, sticky=W)
        self.snrestVal_label = Label(self, text="")
        self.snrestVal_label.grid(row=2, column=9, sticky=E)

        self.prssi_label = Label(self, text="Packet RSSI:")
        self.prssi_label.grid(row=3, column=8, sticky=W)
        self.prssiVal_label = Label(self, text="")
        self.prssiVal_label.grid(row=3, column=9, sticky=E)
        
        self.crssi_label = Label(self, text="Current RSSI:")
        self.crssi_label.grid(row=4, column=8, sticky=W)
        self.crssiVal_label = Label(self, text="")
        self.crssiVal_label.grid(row=4, column=9, sticky=E)

        self.pyld_label = Label(self, text="Payload Length:")
        self.pyld_label.grid(row=5, column=8, sticky=W)
        self.pyldVal_label = Label(self, text="")
        self.pyldVal_label.grid(row=5, column=9, sticky=E)

        self.crcerr_label = Label(self, text="CRC Error:")
        self.crcerr_label.grid(row=6, column=8, sticky=W)
        self.crcerrVal_label = Label(self, text="0")
        self.crcerrVal_label.grid(row=6, column=9, sticky=E)

        self.txLat_label = Label(self, text="Latitude:")
        self.txLat_label.grid(row=1, column=11, sticky=W)
        self.txLat_label_val = Label(self, text="")
        self.txLat_label_val.grid(row=1, column=12, sticky=E)

        self.txLon_label = Label(self, text="Longitude:")
        self.txLon_label.grid(row=2, column=11, sticky=W)
        self.txLon_label_val = Label(self, text="")
        self.txLon_label_val.grid(row=2, column=12, sticky=E)

        self.txAlt_label = Label(self, text="Altitude:")
        self.txAlt_label.grid(row=3, column=11, sticky=W)
        self.txAlt_label_val = Label(self, text="")
        self.txAlt_label_val.grid(row=3, column=12, sticky=E)

        self.txTime_label = Label(self, text="Time:")
        self.txTime_label.grid(row=0, column=11, sticky=W)
        self.txTime_label_val = Label(self, text="")
        self.txTime_label_val.grid(row=0, column=12, sticky=E)

        self.rxLat_label = Label(self, text="RX Latitude:")
        self.rxLat_label.grid(row=4, column=11, sticky=W)
        self.rxLat_var = StringVar()
        self.rxLat_var.trace(mode="w", callback=self.rxLat_content)
        self.rxLat_in = Entry(self, state=NORMAL, width=10, background="light pink", textvariable=self.rxLat_var)
        self.rxLat_in.grid(row=4, column=12, sticky=E)

        self.rxLon_label = Label(self, text="RX Longitude:")
        self.rxLon_label.grid(row=5, column=11, sticky=W)
        self.rxLon_var = StringVar()
        self.rxLon_var.trace(mode="w", callback=self.rxLon_content)
        self.rxLon_in = Entry(self, state=NORMAL, width=10, background="light pink", textvariable=self.rxLon_var)
        self.rxLon_in.grid(row=5, column=12, sticky=E)

        self.rxAlt_label = Label(self, text="RX Altitude:")
        self.rxAlt_label.grid(row=6, column=11, sticky=W)
        self.rxAlt_var = StringVar()
        self.rxAlt_var.trace(mode="w", callback=self.rxAlt_content)
        self.rxAlt_in = Entry(self, state=NORMAL, width=10, background="light pink", textvariable=self.rxAlt_var)
        self.rxAlt_in.grid(row=6, column=12, sticky=E)

        self.los_label = Label(self, text="Line of Sight:")
        self.los_label.grid(row=0, column=14, sticky=W)
        self.los_label_val = Label(self, text="")
        self.los_label_val.grid(row=0, column=15, sticky=E)
        
        self.distance_label = Label(self, text="Distance:")
        self.distance_label.grid(row=1, column=14, sticky=W)
        self.distance_label_val = Label(self, text="", width=12, anchor=E)
        self.distance_label_val.grid(row=1, column=15, sticky=E)

        self.azimuth_label = Label(self, text="Azimuth:")
        self.azimuth_label.grid(row=2, column=14, sticky=W)
        self.azimuth_label_val = Label(self, text="")
        self.azimuth_label_val.grid(row=2, column=15, sticky=E)

        self.elevation_label = Label(self, text="Elevation:")
        self.elevation_label.grid(row=3, column=14, sticky=W)
        self.elevation_label_val = Label(self, text="")
        self.elevation_label_val.grid(row=3, column=15, sticky=E)

        self.rxCall_label = Label(self, text="RX Callsign:")
        self.rxCall_label.grid(row=4, column=14, sticky=W)
        self.rxCall_var = StringVar()
        self.rxCall_var.trace(mode="w", callback=self.rxCall_content)
        self.rxCall_in = Entry(self, state=NORMAL, width=10, background="light pink", textvariable=self.rxCall_var)
        self.rxCall_in.grid(row=4, column=15, sticky=E)

        self.pktNum_label = Label(self, text="Packets:")
        self.pktNum_label.grid(row=5, column=14, sticky=W)
        self.pktNum_label_val = Label(self, text="0")
        self.pktNum_label_val.grid(row=5, column=15, sticky=E)

        self.pktAge_label = Label(self, text="Last Packet:")
        self.pktAge_label.grid(row=6, column=14, sticky=W)
        self.pktAge_label_val = Label(self, text="-")
        self.pktAge_label_val.grid(row=6, column=15, sticky=E)

        self.packets_text = Text(self, wrap=WORD, height=30)
        self.packets_text.grid(row=7, column=0, columnspan=17, sticky=W+E)

        self.status_label = Label(self, text="Status:")
        self.status_label.grid(row=8, column=0, sticky=NS)
        self.status_label_val = Label(self, text="")
        self.status_label_val.grid(row=8, column=1, sticky=W, columnspan=13)

        self.padding1_label = Label(self, text="", width=7)
        self.padding1_label.grid(row=0, column=4, sticky=W+E)

        self.padding2_label = Label(self, text="",width=10)
        self.padding2_label.grid(row=4, column=2, sticky=W+E)

        self.padding3_label = Label(self, text="",width=10)
        self.padding3_label.grid(row=4, column=3, sticky=W+E)

        self.padding4_label = Label(self, text="", width=7)
        self.padding4_label.grid(row=0, column=7, sticky=W+E)

        self.padding5_label = Label(self, text="", width=7)
        self.padding5_label.grid(row=0, column=10, sticky=W+E)

        self.padding6_label = Label(self, text="", width=7)
        self.padding6_label.grid(row=0, column=13, sticky=W+E)

        self.padding7_label = Label(self, text="", width=4)
        self.padding7_label.grid(row=0, column=16, sticky=W+E)

        
    def refresh_ports(self):
        """ Refreshes the COM port list """
        self.port_box['values'] = (serial_ports())


    def update_buttons(self, event=None):
        """ Update button based on Combobox selection """
        if self.port_box.get() == self.activePort:
            self.port_button2.config(text="Disconnect", command=self.disconnect_port, state=ACTIVE)
            self.port_button.config(state=DISABLED)
        else:
            if self.serialPort.isOpen():
                self.port_button2.config(text="Connect", command=self.connect_port, state=DISABLED)
                self.port_button.config(state=DISABLED)
            else:
                self.port_button2.config(text="Connect", command=self.connect_port, state=ACTIVE)
                self.port_button.config(state=ACTIVE)


    def connect_port(self):
        """ Connect to selected port """
        self.serialPort.port = self.port_box.get()
        self.serialPort.open()

        # added to support CP2102 with DTR instead of RTS
        self.serialPort.setDTR(True)
        time.sleep(0.05)
        self.serialPort.setDTR(False)
        time.sleep(0.05)
        self.serialPort.setDTR(True)
        
        self.activePort = self.port_box.get()
        self.update_buttons()
        self.status("Connected to port " + self.port_box.get() + ".")
        
        self.mode_box.config(state=ACTIVE)
        self.mode_button.config(state=ACTIVE)
        self.freq_in.config(state=NORMAL)
        self.freq_button.config(state=ACTIVE)
        self.onoff_button.config(state=ACTIVE)
        self.onoff_label.config(state=NORMAL)
        self.decode_ssdv()
        self.read_port()


    def read_port(self):
        """ Periodically check for data """
        if self.serialPort.isOpen():
            
            while (self.serialPort.inWaiting() > 0):
                line = self.serialPort.readline()
                
                if len(line) < 3:
                        break
                
                # SSDV packet may contain '\n' as part of the data
                if  line[0] == 'j' and line[1] == ':':

                    if len(line[2:]) == 257:
                           packet = line[2:-1]
                    else:
                           packet = line[2:]
                    
                    for i in range(256 - len(packet)):
                        b = self.serialPort.read()
                        packet = packet + b

                    # packet info
                    pktInfo = ''
                    timestamp = str(datetime.datetime.now())
                    pktType = packet[1]
                    callsign = ''
                    imgID = ord(packet[6])
                    pktID = (ord(packet[7]) << 8) | ord(packet[8])
                    width = ord(packet[9]) * 16
                    height = ord(packet[10]) * 16
                    flags = ord(packet[11])
                    mcuOffset = ord(packet[12])
                    mcuIndex = (ord(packet[13]) << 8) | ord(packet[14])
                    checksum = 0
                    FEC = []

                    if len(timestamp) == 19:
                        timestamp += ".00"
                    
                    if pktType == 'f': # FEC packet type
                        checksum = (ord(packet[220]) << 24) | (ord(packet[221]) << 16) | (ord(packet[222]) << 8) | ord(packet[223])
                        FEC = packet[224:]
                    elif pktType == 'g': # NO-FEC packet type
                        checksum = (ord(packet[252]) << 24) | (ord(packet[253]) << 16) | (ord(packet[254]) << 8) | ord(packet[255])

                    #decode Callsign
                    c = (ord(packet[2]) << 24) | (ord(packet[3]) << 16) | (ord(packet[4]) << 8) | ord(packet[5])

                    while c > 0:
                        d = c % 40
                        
                        if d == 0:
                            callsign = callsign + '-'
                        elif d < 11:
                            callsign = callsign + chr(48 + d - 1)
                        elif d < 14:
                            callsign = callsign + '-'
                        else:
                            callsign = callsign + chr(65 + d - 14)
                        
                        c = c / 40
                                        
                    pktInfo = timestamp[0:22] + ' ' + "SSDV from: " + callsign + ' ' + "ImageID: " + str(imgID) + ' ' + "PacketID: " + str(pktID) + ' ' + \
                              str(width) + 'x' + str(height) + ' ' + "Quality: " + str((((flags >> 3) & 7) ^ 4)) + ' '

                    if pktType == 'f':
                        pktInfo += "Normal"
                    elif pktType == 'g':
                        pktInfo += "No-FEC"

                    if ((flags >> 2) & 1) == 1:
                        pktInfo += " END OF IMAGE"
                    else:
                        pktInfo += ""

                    self.lastPkt = pktInfo
                    self.lastPktType = 'j'
                    self.status("SSDV packet received.")

                    self.update_ssdv_queue(packet)

                    # check if the file already exists
                    matches = []
                    matchesN = []
                    # example: 2017-08-20_16-29-18_TT7L_0001
                    ssdvFile = timestamp[0:10] + '_' + timestamp[11:13] + '-' + timestamp[14:16] + '-' + timestamp[17:19] + '_' + callsign + '_' + str(imgID).zfill(4)
                    
                    # save data to file
                    if self.save_buttonVar.get() == 1:

                        # save raw SSDV packets
                        abs_dir = os.path.dirname(__file__)
                        rel_dir = "raw/" + ssdvFile
                        abs_path = os.path.join(abs_dir, rel_dir)

                        # is it the first received image/packet?
                        if not self.imgList:
                            self.imgList.append(ssdvFile)
                            self.pktList.append(1)
                            self.typeList.append(pktType)

                            # store raw packet
                            f = open(abs_path, 'wb')
                            f.write(packet)
                            f.close()
                            break

                        # does file with this callsign and image ID already exist?
                        # packets may be coming in from different transmitters, need to store the packet in the right file
                        for i in range(len(self.imgList)):
                            
                            if callsign and str(imgID).zfill(4) in self.imgList[i]:
                                matches.append(self.imgList[i])
                                matchesN.append(i) # appends the sequence number within the list of the matched file

                        # if not create the file
                        if not matches:
                            self.imgList.append(ssdvFile)
                            self.pktList.append(1)
                            self.typeList.append(pktType)
                        # else store the packet in the existing file
                        # imgList holds its contents only while the program is running, restarting the application creates new files (e.g. restarting while receiving the same image)
                        else:
                            # if the transmitter was restarted and the gateway wasn't, new images under old image IDs will be received
                            rel_dir = "raw/" + matches[-1]
                            abs_path = os.path.join(abs_dir, rel_dir)
                            
                            f = open(abs_path, 'rb')
                            pkt = f.read()[-256:]
                            f.close()
                            pktIDlast = (ord(pkt[7]) << 8) | ord(pkt[8])

                            # if last saved packet ID is bigger then current, store the packet in a new file
                            if pktID <= pktIDlast:
                                self.imgList.append(ssdvFile)
                                self.pktList.append(1)
                                self.typeList.append(pktType)
                            # else store the packet in the latest file
                            else:
                                ssdvFile = matches[-1]
                                self.pktList[matchesN[-1]] += 1

                        rel_dir = "raw/" + ssdvFile
                        abs_path = os.path.join(abs_dir, rel_dir)
                        
                        # store raw packet
                        f = open(abs_path, 'ab')
                        f.write(packet)
                        f.close()

                    break
                
                line = line.strip()

                # Current RSSI
                if line[0] == 'r' and line[1] == ':':
                    self.crssiVal_label.config(text=line[2:] + " dBm")

                    # update time since last received packet
                    if self.lastPktTime != datetime.datetime(1,1,1):
                        pktAge = datetime.datetime.now() - self.lastPktTime
                        self.pktAge_label_val.config(text=str(pktAge).split('.')[0])
                    
                    break

                # Frequency info
                if line[0] == 'f' and line[1] == ':':
                    self.actualFreqVal_label.config(text=line[2:] + " MHz")
                    self.status("New frequency tunned.")
                    break

                # Telemetry packet
                if  line[0] == 't' and line[1] == ':':
                    self.lastPkt = line[2:]
                    self.lastPktType = 't'
                    self.status("Telemetry packet received.")
                    
                    self.update_telemetry_queue(line[2:] + '\n')

                    # parsing
                    telem = line[2:]
                    fields = telem.split(',')

                    self.txLat_label_val.config(text=fields[3])
                    self.txLon_label_val.config(text=fields[4])
                    self.txAlt_label_val.config(text=str(int(fields[5])))
                    self.txTime_label_val.config(text=fields[2])
                    
                    if self.rxLat_ok == 1 and self.rxLon_ok == 1 and self.rxAlt_ok == 1:
                        self.losLast = self.LineOfSight(self.rxLat_val, self.rxLon_val, self.rxAlt_val, float(fields[3]), float(fields[4]), float(fields[5]))
                        self.los_label_val.config(text="{0:.3f} km".format(self.losLast / 1000.0))
                        
                        self.distanceLast = self.GreatCircleDistance(self.rxLat_val, self.rxLon_val, float(fields[3]), float(fields[4]))
                        self.distance_label_val.config(text="{0:.3f} km".format(self.distanceLast / 1000.0))

                        self.azimuthLast = self.Azimuth(self.rxLat_val, self.rxLon_val, float(fields[3]), float(fields[4]))
                        self.azimuth_label_val.config(text=u"{0:.1f} \u00b0".format(self.azimuthLast))

                        self.elevationLast = self.Elevation(self.distanceLast, float(fields[5]), float(self.rxAlt_val))
                        self.elevation_label_val.config(text=u"{0:.1f} \u00b0".format(self.elevationLast))
                    
                    break

                # Unidentified packet
                if line[0] == 'p' and line[1] == ':':
                    self.lastPkt = line[2:]
                    self.lastPktType = 'p'
                    self.status("Unidentified packet received.")
                    break

                # CRC error packet
                if line[0] == 'z' and line[1] == ':':
                    self.lastPkt = line[2:]
                    self.lastPktType = 'z'

                    self.crcerrVal_label.config(text=str(int(self.crcerrVal_label['text']) + 1))
                    self.status("CRC error in last packet.")
                    break

                # Packet SNR
                if line[0] == 'n' and line[1] == ':':
                    self.snrVal_label.config(text=line[2:] + " dB")
                    break

                # Packet RSSI
                if line[0] == 'c' and line[1] == ':':
                    self.prssiVal_label.config(text=line[2:] + " dBm")

                    # calculate estimated SNR
                    prssiTemp = int(line[2:])
                    crssiTemp = int(self.crssiVal_label['text'].split(' ')[0])
                    snrTemp = int(self.snrVal_label['text'].split(' ')[0])

                    if snrTemp > 5:
                        snrestTemp = prssiTemp - crssiTemp
                        self.snrestVal_label.config(text=str(snrestTemp) + " dB")
                    else:
                        self.snrestVal_label.config(text=self.snrVal_label['text'])
                        
                    break

                # Frequency Error
                if line[0] == 'q' and line[1] == ':':
                    self.freqErrVal_label.config(text=line[2:] + " kHz")

                    # append packet info, print and save packet
                    if self.rxLat_ok == 1 and self.rxLon_ok == 1 and self.rxAlt_ok == 1 and self.lastPktType == 't':
                        pktInfo = "   snr: " + self.snrestVal_label['text'].split(' ')[0] + " rssi: " + self.prssiVal_label['text'].split(' ')[0] + " err: " + line[2:] + \
                                  " los: " + "{0:.3f}".format(self.losLast / 1000.0) + " dst: " + "{0:.3f}".format(self.distanceLast / 1000.0) + \
                                  " azm: " + "{0:.1f}".format(self.azimuthLast) + " elv: " + "{0:.1f}".format(self.elevationLast)
                    else:
                        pktInfo = "   snr: " + self.snrestVal_label['text'].split(' ')[0] + " rssi: " + self.prssiVal_label['text'].split(' ')[0] + " err: " + line[2:]

                    if self.lastPktType == 'z':
                        self.packets_text.insert(0.0, "CRC Error" + pktInfo + '\n')
                    else:
                        self.packets_text.insert(0.0, self.lastPkt + pktInfo + '\n')
                        self.pktNum_label_val.config(text=str(int(self.pktNum_label_val['text']) + 1))
                        self.lastPktTime = datetime.datetime.now()
                    
                    if self.save_buttonVar.get() == 1:
                        if self.lastPktType == 't':
                            f = open("out_telemetry.txt", 'a')
                        elif self.lastPktType == 'j':
                            f = open("out_ssdv.txt", 'a')
                        elif self.lastPktType == 'p':
                            f = open("out_packets.txt", 'a')
                        elif self.lastPktType == 'z':
                            f = open("out_packets.txt", 'a')

                        if self.lastPktType == 'z':
                            f.write(self.lastPkt + "   CRC Error" + pktInfo + '\n')
                        else:
                            f.write(self.lastPkt + pktInfo + '\n')
                        
                        f.close()

                    break

                # LoRa mode info
                if line[0] == 'm' and line[1] == ':':
                    self.actualModeVal_label.config(text=line[2:])
                    self.status("New LoRa mode set.")
                    break

                # Implicit/Explicit header mode info
                if line[0] == 'i' and line[1] == ':':
                    self.HeaderVal_label.config(text=line[2:])
                    
                    if line[2:] == "implicit":
                        self.payload_button.config(state=ACTIVE)
                        self.payload_in.config(state=NORMAL)
                        self.pyld_label.config(state=NORMAL)
                        self.pyldVal_label.config(state=NORMAL)
                    else:
                        self.payload_button.config(state=DISABLED)
                        self.payload_in.config(state=DISABLED)
                        self.pyld_label.config(state=DISABLED)
                        self.pyldVal_label.config(state=DISABLED)

                    break

                # Bandwidth info
                if line[0] == 'b' and line[1] == ':':
                    self.BandwidthVal_label.config(text=line[2:] + " kHz")
                    break

                # Error Coding info
                if line[0] == 'e' and line[1] == ':':
                    self.ErrorVal_label.config(text=line[2:])
                    break

                # Spreading Factor info
                if line[0] == 's' and line[1] == ':':
                    self.SpreadingVal_label.config(text=line[2:])
                    break

                # Low Data Rate info
                if line[0] == 'l' and line[1] == ':':
                    self.RateVal_label.config(text=line[2:])
                    break

                # Payload Length info
                if line[0] == 'w' and line[1] == ':':
                    self.pyldVal_label.config(text=line[2:])
                    self.status("New payload length set.")
                    break

                # LoRa module in RX Continuous or Standby info
                if line[0] == 'o' and line[1] == ':':
                    self.RXonoff = line[2]

                    if self.RXonoff == '0':
                        self.onoff_label.config(text="RX OFF", font='Tahoma 8 bold')
                        self.status("LoRa module in standby.")
                    elif self.RXonoff == '1':
                        self.onoff_label.config(text="RECEIVING", font='Tahoma 8 bold')
                        self.status("LoRa module in receive continuous.")
                    break
            
            self.after(1, self.read_port) # milliseconds


    def disconnect_port(self):
        """ Closes the serial port connection """
        self.serialPort.close()
        self.activePort = 0
        self.update_buttons()

        self.mode_box.config(state=DISABLED)
        self.mode_button.config(state=DISABLED)
        self.freq_in.config(state=DISABLED)
        self.freq_button.config(state=DISABLED)
        self.onoff_button.config(state=DISABLED)
        self.onoff_label.config(state=DISABLED)
        self.status("Port disconnected.")


    def change_freq(self):
        """ Send command to the device """
        content = self.freq_in.get()

        try:
            if float(content) >= 433.05 and float(content) <= 434.79:
                self.serialPort.write('f')
                self.serialPort.write(content.encode())
                self.read_port()
        except:
            pass
        
        self.freq_in.delete(0, 'end')


    def change_mode(self):
        """ Send command to the device """
        self.serialPort.write('m')
        self.serialPort.write(LoRa_mode[self.mode_box.get()])
        self.read_port()


    def change_length(self):
        """ Send command to the device """
        content = self.payload_in.get()

        try:
            if int(content) >= 1 and int(content) <= 255:
                self.serialPort.write('l')
                self.serialPort.write(content.encode())
                self.read_port()
        except:
            pass
        
        self.payload_in.delete(0, 'end')


    def decode_ssdv(self):
        """ in intervals decodes new ssdv packets """
        newPackets = 0
        
        for i in range(len(self.pktList)):
            
            if self.pktList[i] > 0:
                args = ''
                newPackets += self.pktList[i]
                self.pktList[i] = 0
                
                if self.typeList[i] == 'f':
                    args = "SSDV.exe -d \""
                else:
                    args = "SSDV.exe -d -n \""
                
                args += ("raw\\" + self.imgList[i] + "\" \"" + self.imgList[i] + ".jpg\"")
                subprocess.Popen(args, shell=True)
                
                self.status("New SSDV packets decoded.")
        
        self.show_image()
        
        if self.serialPort.isOpen():
            # decide the frequency of decoding based on the rate at which new packets arrive
            if newPackets >= 1:
                self.after(3000, self.decode_ssdv) # milliseconds
            else:
                self.after(6000, self.decode_ssdv) # milliseconds


    def show_image(self):
        """ opens the latest received image in a new window """
        if self.img_buttonVar.get() == 1:
            if self.ssdv_window.state() == "withdrawn":
                self.ssdv_window.deiconify()

            try:
                img1 = ImageTk.PhotoImage(Image.open(self.imgList[-1] + ".jpg"))
                self.ssdv_window.title(self.imgList[-1] + ".jpg")
                self.ssdv_window.geometry(str(img1.width()) + "x" + str(img1.height()))

                self.ssdv_canvas.config(width=img1.width(), height=img1.height())
                self.ssdv_canvas.create_image(img1.width() / 2, img1.height() / 2, image=img1)
                self.ssdv_canvas.img1 = img1
                self.ssdv_canvas.grid()
            except:
                pass
            
        else:
            if self.ssdv_window.state() == "normal":
                self.ssdv_window.withdraw()


    def remove_image_window(self):
        """ manages ssdv window closure """
        self.img_buttonVar.set(value=0)
        self.ssdv_window.withdraw()


    def change_onoff(self):
        """ Turn RXing ON or OFF """
        self.serialPort.write('o')

        if self.RXonoff == '0':
            self.serialPort.write('1')
        elif self.RXonoff == '1':
            self.serialPort.write('0')
        
        self.read_port()


    def GreatCircleDistance(self, lat1, lon1, lat2, lon2):
        """ Calculates the great circle distance between two points """

        R = 6371000 # meters

        lat1 = lat1 / 180 * math.pi
        lon1 = lon1 / 180 * math.pi
        lat2 = lat2 / 180 * math.pi
        lon2 = lon2 / 180 * math.pi

        a = math.sin((lat1 - lat2) / 2) * math.sin((lat1 - lat2) / 2) + \
            math.cos(lat1) * math.cos(lat2) * \
            math.sin((lon1 - lon2) / 2) * math.sin((lon1 - lon2) / 2)

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c


    def Azimuth(self, lat1, lon1, lat2, lon2):
        """ Calculates the Azimuth between two points """

        lat1 = lat1 / 180 * math.pi
        lon1 = lon1 / 180 * math.pi
        lat2 = lat2 / 180 * math.pi
        lon2 = lon2 / 180 * math.pi
        
        theta = math.atan2(math.sin(lon2 - lon1) * math.cos(lat2), math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))

        azimuth = math.fmod(theta / math.pi * 180.0 + 360.0, 360.0)
        
        return azimuth


    def Elevation(self, gcdistance, alt1, alt2):
        """ Calculates the Elevation between two points """
        
        R = 6371000 # meters (Earth's radius)

        a = gcdistance / (2 * math.pi * R) * (2 * math.pi) # rad

        s = math.sin(a) * (R + alt1) # m

        c = math.cos(a) * (R + alt1) - (R + alt2) # m

        l = math.sqrt((s**2 + c**2)) # m
        
        y = math.pi / 2 - math.acos((s**2 - c**2 - l**2) / (-2 * c * l)) # rad
        ydeg = y / math.pi * 180.0
        
        return ydeg


    def LineOfSight(self, lat1, lon1, alt1, lat2, lon2, alt2):
        """ Calculates the Line of Sight distance from coords_1 to coords_2 """

        R = 6371000.0 # meters (Earth's radius)
        
        laShrt = math.sin(math.fabs(lat1 - lat2) / 2.0 / 180.0 * math.pi) * (R + alt1) * 2.0 # m
        laRad = math.sin((90 - lat2) / 180.0 * math.pi) * (R + alt1) # m (radius at latitude)
        loShrt1 = math.sin(math.fabs(lon1 - lon2) / 2.0 / 180.0 * math.pi) * (laRad) * 2.0 # m (shortest longitudinal distance at ground level)
        loCenterAngle = math.asin((loShrt1 / 2.0) / R) * 2.0 # Rad (longitude angle at the center of the Earth)
        loShrt2 = math.sin(loCenterAngle / 2.0) * (R + alt1) * 2.0 # m (shortest longitudinal distance altitude adjusted)
        shrt = math.sqrt(laShrt**2 + loShrt2**2) # m (shortest distance between coords_1 and coords_2 at coords_1 altitude)
        mainCenterAngle = math.asin((shrt / 2.0) / (R + alt1)) * 2.0 # Rad (angle at the center of the Earth between coords_1 and coords_2)
        
        LOS = math.sqrt(shrt**2 + (alt2 - alt1)**2 - 2.0 * shrt * (alt2 - alt1) * math.cos(mainCenterAngle / 2.0 + math.pi / 2.0)) # m [a2 = b2 + c2 - 2bc * cos(alpha)]

        return LOS


    def rxLat_content(self, *args):
        """ Oversees whether Entry was written or not """
        if len(self.rxLat_in.get()) > 0:
            try:
                self.rxLat_val = float(self.rxLat_in.get())
                self.rxLat_in.config(background="White")
                self.rxLat_ok = 1
                self.rxChangedTelem = 1
            except:
                self.rxLat_in.config(background="light pink")
                self.rxLat_ok = 0
        else:
            self.rxLat_in.config(background="light pink")
            self.rxLat_ok = 0


    def rxLon_content(self, *args):
        """ Oversees whether Entry was written or not """
        if len(self.rxLon_in.get()) > 0:
            try:
                self.rxLon_val = float(self.rxLon_in.get())
                self.rxLon_in.config(background="White")
                self.rxLon_ok = 1
                self.rxChangedTelem = 1
            except:
                self.rxLon_in.config(background="light pink")
                self.rxLon_ok = 0
        else:
            self.rxLon_in.config(background="light pink")
            self.rxLon_ok = 0


    def rxAlt_content(self, *args):
        """ Oversees whether Entry was written or not """
        if len(self.rxAlt_in.get()) > 0:
            try:
                self.rxAlt_val = int(self.rxAlt_in.get())
                self.rxAlt_in.config(background="White")
                self.rxAlt_ok = 1
                self.rxChangedTelem = 1
            except:
                self.rxAlt_in.config(background="light pink")
                self.rxAlt_ok = 0
        else:
            self.rxAlt_in.config(background="light pink")
            self.rxAlt_ok = 0


    def rxCall_content(self, *args):
        """ Oversees whether Entry was written or not """
        if len(self.rxCall_in.get()) > 0:
            try:
                self.rxCall_val = str(self.rxCall_in.get())
                self.rxCall_in.config(background="White")
                self.rxCall_ok = 1
                self.rxChangedInfo = 1
            except:
                self.rxCall_in.config(background="light pink")
                self.rxCall_ok = 0
        else:
            self.rxCall_in.config(background="light pink")
            self.rxCall_ok = 0


    def run(self):
        """ Background thread """
        while True:
            item = self._queue.get()

            if item is None:
                break
            
            (func, arg) = item

            if func == "listener_information":
                doc = self.upload_listener_information(self.rxCall_val, self.rxLat_val, self.rxLon_val)
            elif func == "listener_telemetry":
                doc = self.upload_listener_telemetry(self.rxCall_val, self.rxLat_val, self.rxLon_val, self.rxAlt_val)
            elif func == "payload_telemetry":
                doc = self.upload_payload_telemetry(arg, self.rxCall_val)
            elif func == "ssdv_packet":
                doc = self.upload_ssdv_packet(arg, self.rxCall_val)


    def update_listener(self):
        """ Uploads listener's details to Habitat and periodically updates it """
        # update only if "Upload Data to Habitat" is checked
        if self.upload_buttonVar.get() == 1:
            # update only if all fields are filled correctly
            if self.rxLat_ok == 1 and self.rxLon_ok == 1 and self.rxAlt_ok == 1 and self.rxCall_ok == 1:
                # send update only if fields changed
                if self.rxChangedInfo == 1:
                    self._queue.put(("listener_information", ""))
                    self.rxChangedInfo = 0

                if self.rxChangedTelem == 1:
                    self._queue.put(("listener_telemetry", ""))
                    self.rxChangedTelem = 0

                self.status("Uploading to Habitat enabled.")
            else:
                self.status("Fill out the listener information to allow uploading data!")
        else:
            self.status("Uploading to Habitat disabled.")


    def update_telemetry_queue(self, string):
        """ Adds telemetry packet to upload queue """
        # basic telemetry format check
        stringTemp = string.split('\n')

        if self.upload_buttonVar.get() == 1:
        
            if stringTemp[0][0] == '$' and stringTemp[0][1] == '$' and (stringTemp[0][-5] == '*' or stringTemp[0][-3] == '*'):
        
                if self.rxCall_ok == 1 and self.rxLat_ok == 1 and self.rxLon_ok == 1 and self.rxAlt_ok == 1:
                    
                    with self._lock:
                        if self.rxChangedInfo == 1:
                            self._queue.put(("listener_information", ""))
                            self.rxChangedInfo = 0

                        if self.rxChangedTelem == 1:
                            self._queue.put(("listener_telemetry", ""))
                            self.rxChangedTelem = 0

                    self._queue.put(("payload_telemetry", string))
                else:
                    self.status("Fill out the listener information to allow uploading data!")
                    
            else:
                self.status("Telemetry check ERROR! Not uploaded.")


    def update_ssdv_queue(self, pkt):
        """ Adds SSDV packet to upload queue """
        if self.upload_buttonVar.get() == 1:

            if ord(pkt[0]) == 85 and (ord(pkt[1]) == 102 or ord(pkt[1]) == 103) and len(pkt) == 256:

                if self.rxCall_ok == 1 and self.rxLat_ok == 1 and self.rxLon_ok == 1 and self.rxAlt_ok == 1:

                    self._queue.put(("ssdv_packet", pkt))

                else:
                    self.status("Fill out the listener information to allow uploading data!")

            else:
                self.status("SSDV packet check ERROR! Not uploaded.")


    def upload_listener_information(self, callsign, latitude, longitude):
        """
        Creates a new document in Habitat's couchDB database.

        {
          "type": "listener_information", 
          "data": {
            "callsign": "TT7", 
            "radio": "LoRa receiver", 
            "location": "49.4688 18.1508"
          }, 
          "time_created": "2018-03-21T18:02:02+01:00", 
          "time_uploaded": "2018-03-21T18:02:02+01:00"
        }
        """
        data = {}

        data["radio"] = "LoRa receiver"
        data["callsign"] = callsign
        data["location"] = str(latitude) + " " + str(longitude)

        doc = {
                "data": data
        }

        time_uploaded = int(round(time.time()))
        time_created = int(round(time.time()))

        to_rfc3339 = strict_rfc3339.timestamp_to_rfc3339_localoffset
        doc["time_uploaded"] = to_rfc3339(time_uploaded)
        doc["type"] = "listener_information"
        doc["time_created"] = to_rfc3339(time_created)
        
        self.db.save_doc(doc)
        doc_id = doc["_id"]

        with self._lock:
            self._latest["listener_information"] = doc_id
            self.status("Listener information uploaded.")
        
        return doc_id


    def upload_listener_telemetry(self, callsign, latitude, longitude, altitude):
        """
        Creates a new document in Habitat's couchDB database.

        {
          "type": "listener_telemetry", 
          "data": {
            "latitude": 49.4688, 
            "altitude": 403, 
            "callsign": "TT7", 
            "longitude": 18.1508
          }, 
          "time_created": "2018-03-21T18:03:10+01:00", 
          "time_uploaded": "2018-03-21T18:03:10+01:00"
        }
        """
        data = {}

        data["callsign"] = callsign
        data["latitude"] = latitude
        data["longitude"] = longitude
        data["altitude"] = altitude

        doc = {
                "data": data
        }
        
        time_uploaded = int(round(time.time()))
        time_created = int(round(time.time()))

        to_rfc3339 = strict_rfc3339.timestamp_to_rfc3339_localoffset
        doc["time_uploaded"] = to_rfc3339(time_uploaded)
        doc["type"] = "listener_telemetry"
        doc["time_created"] = to_rfc3339(time_created)
        
        self.db.save_doc(doc)
        doc_id = doc["_id"]

        with self._lock:
            self._latest["listener_telemetry"] = doc_id
            self.status("Listener telemetry uploaded.")
        
        return doc_id

 
    def upload_payload_telemetry(self, string, listener):
        """
        Either creates a new document in Habitat's couchDB database for *string* or adds a receiver to *string*'s existing document.
        """
        receiver_info = {}
        
        with self._lock:
            for doc_type in ["listener_telemetry", "listener_information"]:
                if doc_type in self._latest:
                    receiver_info["latest_" + doc_type] = self._latest[doc_type]
        
        for i in xrange(self._max_merge_attempts):
            try:
                time_uploaded = int(round(time.time()))
                time_created = int(round(time.time()))

                to_rfc3339 = strict_rfc3339.timestamp_to_rfc3339_localoffset
                receiver_info["time_uploaded"] = to_rfc3339(time_uploaded)
                receiver_info["time_created"] = to_rfc3339(time_created)
                
                doc_id = hashlib.sha256(base64.b64encode(string)).hexdigest()
                
                doc_ish = {
                    "data": {"_raw": base64.b64encode(string)},
                    "receivers": {listener: receiver_info}
                }
                
                url = "_design/payload_telemetry/_update/add_listener/" + doc_id
                self.db.res.put(url, payload=doc_ish).skip_body()
                
                with self._lock:
                    self.status("Telemetry string uploaded.")
                
            except couchdbkit.exceptions.ResourceConflict:
                continue
            except restkit.errors.Unauthorized:
                raise UnmergeableError
            else:
                return doc_id
        else:
            raise UnmergeableError


    def upload_ssdv_packet(self, pkt, listener):
        """ POSTs an SSDV packet to the SSDV server """

        hexPacket = ''.join('{:02X}'.format(ord(i)) for i in pkt)

        received = "{:%Y-%m-%dT%H:%M:%SZ}".format(datetime.datetime.now())

        ssdvData = {'type':'packet',
                    'packet':hexPacket,
                    'encoding':'hex',
                    'received':received,
                    'receiver':listener,
                    'fixes':0}
        
        headers = {'Host':'ssdv.habhub.org',
           'Content-Type': 'application/json',
           'Content-Length':str(len(json.dumps(ssdvData)))
           }
        
        r = requests.post(url = self.ssdvServer, data=json.dumps(ssdvData), headers=headers)
        with self._lock:
            self.status("SSDV packet uploaded.")
        
        return r


    def status(self, status):
        """ Update the status bar """
        timestamp = "{:%H:%M:%S}".format(datetime.datetime.now())
        self.status_label_val.config(text=timestamp + "   " + status)


    def __del__(self):
        """ Cleanup """
        self.serialPort.close()
    

#create the window
root = Tk()

#modify root window
root.title("LoRa Gateway TT7")
root.geometry("1150x635")

app = Application(root, serial.Serial(baudrate=250000))

#kick off the event loop
root.mainloop()

