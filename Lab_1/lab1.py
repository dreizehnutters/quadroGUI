###############################################################################
###############################################################################
## Lab 1
##
## This file contains the software skeleton for the lab_1 tasks
##
## author: Nicole Todtenberg, Thomas Basmer
## modifyed : Fabian Kopp
## date: 2016-04-22
##
## version: 0.00
##
## changes:
##    - 0.00 2014-08-05
##        - initial version
###############################################################################
###############################################################################

## import the serial class to work on serial ports. This is needed to access
## the 433 MHz modules connected to the ground station via USB
import serial

## import SerialListen class. This class provides a thread listening on an open
## serial port for incoming data
import SerialListen

## this class is generated using the pymavlink generator. It provides, objects,
## methods and aliases to work on mavlink messages. 
import mavlink0

## import modules to work with system time. These modules are needed for time
## stamps
import datetime, time

## import the sys class from the os module. It is used realize system events
## like exiting the program
from os import sys

## the PyQt4 module is needed to connect the python code to a Qt GUI 
from PyQt4 import QtGui, uic, QtCore

## mavlink parser module. it should be used to parse incoming messages. it is
## also used as dummy quadrocopter object for offline test. (Please Ignore this error :-))
import mavlink_parser

##mavuntilites 
import mavutil

import math
from pymavlink.mavextra import earth_gyro
###############################################################################
###############################################################################
## MyWindow class
##
## This class implements the underlying functionality of the GUI
###############################################################################
###############################################################################
class MyWindow(QtGui.QMainWindow):
    
    ###########################################################################
    ## This is the constructor of the MyWindow class. Here the main features
    ## are set up like:
    ##    - including the Qt GUI file
    ##    - open a serial port where the telemetry module is connected to
    ## It is executed when the program is started
    ###########################################################################
    def __init__(self):
        
        super(MyWindow, self).__init__()
        
        ## load the Qt GUI
        ## This GUI was build using Qt Designer software (also available on this 
        ## VM for this lab the following elements of the GUI are needed:
        ##
        ## pB_connect:    its the push button in the upper left of the GUI. It
        ##                connects and disconnects the this software to quadro-
        ##                copters in its area.
        ##
        ## pB_emergency:   This push button (upper right) is used to force all  
        ##                vehicles to land in the range of the wireless tele-
        ##                metry module if the user asserts an emergency situation
        ##
        ## tB_debug:      This text box is the debug window at the bottom of GUI.
        ##                it is used to print information
        ## neighborhood tab ###################################################
        ## tableWidget    This is the table showing the devices in the neighborhood
        ##
        ## All elements could be accessed using the "self.<object_name>". For
        ## available methods of these objects have look at the Qt documentation
        uic.loadUi('QuadrocopterLab1GUI.ui', self)
        
        ## connect the button-clicked-event of the "Connect" button to the
        ## handleConnectButton method. handleConnectButton implements what
        ## should happen if the connect button has been clicked
        self.pB_connect.clicked.connect(self.handleConnectButton) 
        
        self.getGPS.clicked.connect(self.handleGPSrequest)

        self.getIMU.clicked.connect(self.handleIMUrequest)        
        ##handel EmergencyButton
        self.pB_emergency.clicked.connect(self.handleEmergencyButton)
        
        ##handle arm
        self.pB_arm.clicked.connect(self.handleArmButton)
        self.pB_arm.setStyleSheet("color: rgb(200, 0, 0)")
        
        self.pB_startRoute.clicked.connect(self.startMission)
        
        self.pB_flyT.clicked.connect(lambda: self.requestMisson(1))
        
        self.pB_start.clicked.connect(self.startMission)
        
        self.setHight.returnPressed.connect(lambda: self.requestMisson(0))
                
        ##handle Slider changes valueChanged
        self.throttleSlider.sliderReleased.connect(self.handleChanged)
        self.yawSlider.sliderReleased.connect(self.handleChanged)
        self.pitchSlider.sliderReleased.connect(self.handleChanged)
        self.rollSlider.sliderReleased.connect(self.handleChanged)
        
        ##handle IS chnages
        self.compIDBox.valueChanged.connect(self.setID)
        self.sysIDBox.valueChanged.connect(self.setID)   
           
        ##handle ToggleButton
        self.toggleRaw.toggled.connect(self.toggleView)
        
         
        ## status bar widget showing that a serial port has not been opened yet
        ## to communicate with the radio module
        self.dis = QtGui.QLabel("Serial Port Disconnected")
        self.dis.setStyleSheet('QLabel{color:red}')
        
        ## status bar widget showing that a serial port has been opened
        ## to communicate with the radio module
        self.con = QtGui.QLabel("Serial Port Connected")
        self.con.setStyleSheet('QLabel{color:green}')
        
        ## on init no serial port is opened so show the disconnect state in the statusbar
        QtGui.QStatusBar.addWidget(self.statusbar, self.dis)
        self.dis.show()
        
        ## set the column widths for the table widget, so all data can be seen
        self.tableWidget.setColumnWidth(0,150) # time and date column
        self.tableWidget.setColumnWidth(1,100) # system ID column
        self.tableWidget.setColumnWidth(2,150) # component ID column
        self.tableWidget.setColumnWidth(3,100) # message type column
        self.tableWidget.setColumnWidth(4,240) # system status column
             
        ## make the GUI visible
        self.show()

        ## this variable signals if the program has a wireless connection to
        ## other systems (quadrocopter) 
        self.connected  = False
        #isArm ?
        self.armed      = False
        #isLoaded ?
        self.loaded     = False
        ##stream
        self.GPSactive  = False
        self.IMUactive  = False
        ## RAW OR SCALED2
        self.showIMU2   = False
        
        ## 0 takeoff only
        ## 1 thraeder
        ## 2 else 
        self.figure = 0
        
        ##Sensor Label for ACC, GYRO and MAG
        self.xa = 0 
        self.ya = 0
        self.za = 0
        self.xg = 0
        self.yg = 0
        self.zg = 0
        self.xm = 0
        self.ym = 0 
        self.zm = 0
        
        ##Scaled values
        self.xa2 = 0 
        self.ya2 = 0
        self.za2 = 0
        self.xg2 = 0
        self.yg2 = 0
        self.zg2 = 0
        self.xm2 = 0
        self.ym2 = 0 
        self.zm2 = 0
        
        self.lat = 51.765023
        self.lon = 14.327053
        self.hdg = 0
        
        #init motor
        self.throttle = 1000
        self.pitch    = 1500
        self.roll     = 1500
        self.yaw      = 1500
        
        #ID
        self.sysID  = 1
        self.compID = 1
        
        #init table
        self.tableWidget.setRowCount(1)         
        self.tableWidget.setColumnCount(5)
        
        self.tableWidgetRoute.setRowCount(1)
        self.tableWidgetRoute.setColumnCount(4)
        #Zielwinkel = (hdg + (hdg - ziel))%360
          
        ## get from system parameters if current session is a debug session or not
        self.debug = int(sys.argv[1])
        
    ###########################################################################
    ## This method handles received data from the serial link. It is called
    ## by the SerialListen thread
    ##
    ## The event parameter contains the received data
    ###########################################################################
    def onSerialRX(self, event):
        ## if in debug mode, start sending heartbeat messages
        ## before working on received data check if a proper mavlink message
        ## has been received. If an exception is thrown there should be some-
        ## thing wrong with the received data and the rx_message should be none
        try:
            ## send received data to parser and wait for data tupel (text for
            ## debug console, message object and an empty string for later use)
            rx_message = self.mavlink_obj.parse_char(event)
        except:
    
            ## some thing is wrong with the received data, so set the message to
            ## None-type
            rx_message = None
               
            ## print exception message to console
            ##print str(sys.exc_info()[0])
            
        ## if no correct mavlink message has been received print an error message in the
        ## debug console of the GUI
        if rx_message == None:
            ##self.tB_debug.append("OnSerialRX::None type received")
            pass
        
        ## if a correct mavlink message has been received start the proper
        ## handler method   
        else:
            isForEmu = True
            ## if system is in debug mode check if received message is for the emulator
            if self.debug == 1:
                isForEmu = mavlink_parser.sendToEmulator(self.mavlink_obj,event,self.debug)

            ## if received message was not for the emulator or system is not in debug mode
            if isForEmu == False or self.debug == 0 and rx_message != None:
                ## decode the received message and pass over to the correct method for the message type
                (self.msgId, self.sysId, self.compId, _) = mavlink_parser.parse(self.mavlink_obj,event,self.debug)
                #Heartbeat #0
                if (rx_message.get_type() == 'HEARTBEAT'): 
                    ##handle ARM status of the copter
                    self.handelArmState(rx_message)
                    self.handleHeartbeatMsg(self.sysId, self.compId, rx_message)
               
                #scaledIMU2 #116
                elif  (rx_message.get_type() == 'SCALED_IMU2'):
                    self.handleIMU2Msg(rx_message)
                    
                #RAW IMU #27
                elif (rx_message.get_type() == 'RAW_IMU'):
                    self.handleIMUMsg(rx_message)
                    
                #battery_status #147
                elif (rx_message.get_type() == 'BATTERY_STATUS'):
                    self.handleBattery(rx_message)
                    
                #SYS_STATUS #1
                elif (rx_message.get_type() == 'SYS_STATUS'):
                    self.handleBattery(rx_message)
                
                #GPS #33
                elif (rx_message.get_type() == 'GLOBAL_POSITION_INT'):
                    self.handleGPSMsg(rx_message)
                    
                #RC_CHANNELS_RAW  1000% = 0 2000 = 100% #35
                elif (rx_message.get_type() == 'RC_CHANNELS_RAW'):
                    self.handleRCinput(rx_message)
                
                ## MISSION_REQUEST ( #40 )
                elif (rx_message.get_type() == 'MISSION_REQUEST'):
                    self.handleMissonSend(rx_message)

#               #ACK
                elif (rx_message.get_type() == 'COMMAND_ACK'):
                    self.handleCommandACK(rx_message)

                #MISSON_ACK
                elif (rx_message.get_type() == 'MISSION_ACK'):
                    self.handleMissonACK(rx_message)
                #    
                elif (rx_message.get_type() == 'MISSION_ITEM_REACHED'):
                    self.tableWidgetRoute.setItem(rx_message.__getattribute__('seq')-1, 3, QtGui.QTableWidgetItem("YES"))
                    print rx_message.__getattribute__('seq')
                #
                elif (rx_message.get_type() == 'MISSION_CURRENT'):
                    self.tB_debug.append(rx_message.__getattribute__('seq')+" wird gerade gefolgen")
                #
                elif (rx_message.get_type() == 'MISSION_REQUEST_LIST'):
                    pass
                else:
                    print self.msgId

    ########################################################
    ## get mission Status
    ##
    #########################################################
    def handleMissonACK(self,message):
        state = mavlink0.enums['MAV_MISSION_RESULT'][message.__getattribute__('type')].name
        self.tB_debug.append(state)
            
    ###################################################################
    ##  send the mission selected in the gui
    ##
    ###################################################################
    def handleMissonSend(self, message):
        ## takeoff only
        if (self.figure == 0):
            if (message.__getattribute__('seq') == 0):
                #dummy
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   0, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_TAKEOFF, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #0pitch
                                                   0, #p2
                                                   0, #p3
                                                   0, #0yaw
                                                   self.lat, #lat
                                                   self.lon, #long
                                                   0)#float(self.setHight.text())) #alt  
                
            elif (message.__getattribute__('seq') == 1):
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   1, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_TAKEOFF, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #0pitch
                                                   0, #p2
                                                   0, #p3
                                                   0, #0yaw
                                                   self.lat, #lat
                                                   self.lon, #long
                                                   float(self.setHight.text())) #alt 
            elif (message.__getattribute__('seq') == 2):
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   2, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_LOITER_TIME, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   50, #loiter time
                                                   0, #empty
                                                   0, #radius
                                                   0, #?
                                                   self.lat, #lat
                                                   self.lon, #long
                                                   0)#float(self.setHight.text())) #alt 
            elif (message.__getattribute__('seq') == 3):
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   3, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_RETURN_TO_LAUNCH, #command
                                                   0, #current
                                                   0, #autocontinue
                                                   0, #0pitch
                                                   0, #p2
                                                   0, #p3
                                                   0, #0yaw
                                                   0, #lat
                                                   0, #long
                                                   0) #alt  
                self.loaded = True
                self.tB_debug.append("Mission transfered")
                self.pB_start.setEnabled(True)
                
        ## senc the tetrahedron mission
        elif (self.figure == 1):
            #TODO only calc values once
            
            #has = Heading at Start
            self.has = 42 #self.hdg
            
            #1. gerade
            fstLat, fstLon = self.getLatLon(self.lat,self.lon,3,self.has)
            
            #2. gerade sndLat, sndLon = self.getLatLon(fstLat,fstLon,4,((self.has+90)%360))
            sndLat, sndLon = self.getLatLon(self.lat,self.lon,3,((60-self.has)%360))

            #3. gerade
            thdLat, thdLon = (self.lat, self.lon) 
            
            #4. gerade
            fouLat, fouLon = self.getLatLon(self.lat,self.lon,1.5,((30-self.has)%360))
            
            print "1"
            print fstLat
            print fstLon
            print "---"
            print "2"
            print sndLat
            print sndLon
            print "---"
            print "3"
            print thdLat
            print thdLon
            print "---"
            print "4"
            print fouLat
            print fouLon
            print "---"
            
            if (message.__getattribute__('seq') == 0):
                #dummy
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   0, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_TAKEOFF, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #0pitch
                                                   0, #p2
                                                   0, #p3
                                                   0, #0yaw
                                                   0, #lat
                                                   0, #long
                                                   0) #alt  
            elif (message.__getattribute__('seq') == 1):
                self.tableWidgetRoute.setItem(0,0, QtGui.QTableWidgetItem("Takeoff"))
                self.tableWidgetRoute.setItem(0,1, QtGui.QTableWidgetItem(str(self.has)))
                self.tableWidgetRoute.setRowCount(2)
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   1, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_TAKEOFF, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #0pitch
                                                   0, #p2
                                                   0, #p3
                                                   0, #0yaw
                                                   self.lat, #lat
                                                   self.lon, #long
                                                   1.2) #alt
            #1. gerade
            elif (message.__getattribute__('seq') == 2):
                self.tableWidgetRoute.setItem(1,0, QtGui.QTableWidgetItem("1. gerade"))
                self.tableWidgetRoute.setItem(1,1, QtGui.QTableWidgetItem(str(self.has)))
                self.tableWidgetRoute.setRowCount(3)                
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   2, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_WAYPOINT, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #
                                                   0.2, #radius
                                                   0, #?
                                                   0, #?
                                                   fstLat, #lat
                                                   fstLon, #long
                                                   1.2) #alt 
            #2. gerade
            elif (message.__getattribute__('seq') == 3):
                self.tableWidgetRoute.setItem(2,0, QtGui.QTableWidgetItem("2. gerade"))
                self.tableWidgetRoute.setItem(2,1, QtGui.QTableWidgetItem(str((self.has+60)%360)))
                self.tableWidgetRoute.setRowCount(4)                
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   3, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_WAYPOINT, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #
                                                   0.2, #radius
                                                   0, #?
                                                   0, #?
                                                   sndLat, #lat
                                                   sndLon, #long
                                                   1.2) #alt
            #3. gerade
            elif (message.__getattribute__('seq') == 4):
                self.tableWidgetRoute.setItem(3,0, QtGui.QTableWidgetItem("3. gerade"))
                self.tableWidgetRoute.setItem(3,1, QtGui.QTableWidgetItem(str((self.has+120)%360)))                
                self.tableWidgetRoute.setRowCount(5)                
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   4, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_WAYPOINT, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #
                                                   0.2, #radius
                                                   0, #r?
                                                   0, #?
                                                   thdLat, #lat
                                                   thdLon, #long
                                                   1.2) #alt
            
            #4 oben
            elif (message.__getattribute__('seq') == 5):
                self.tableWidgetRoute.setItem(4,0, QtGui.QTableWidgetItem("4. gerade"))
                self.tableWidgetRoute.setItem(4,1, QtGui.QTableWidgetItem(str((self.has+30)%360)))
                self.tableWidgetRoute.setRowCount(6)                
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   5, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_WAYPOINT, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #
                                                   0.2, #radus
                                                   0, #
                                                   0, #?
                                                   fouLat, #lat
                                                   fouLon, #long
                                                   2.2) #alt
            
            elif (message.__getattribute__('seq') == 6):
                self.tableWidgetRoute.setItem(5,0, QtGui.QTableWidgetItem("Landung"))
                self.mavlink_obj.mission_item_send(self.sysID,
                                                   self.compID,
                                                   6, #seq 
                                                   mavlink0.MAV_FRAME_GLOBAL_RELATIVE_ALT, #frame
                                                   mavlink0.MAV_CMD_NAV_LAND, #command
                                                   0, #current
                                                   1, #autocontinue
                                                   0, #
                                                   0, #empty
                                                   0, #radius
                                                   0, #?
                                                   0, #lat
                                                   0, #long
                                                   0) #alt
                self.loaded = True
                self.tB_debug.append("Mission transfered")
                self.pB_startRoute.setEnabled(True)
        else:
            self.tB_debug.append("Error")
            
         
    ######################################################################
    ## get new LAT/LON values for translation of copter in global COS
    ## degree * pi / 180 * 6360Km = 110.947Km where pi = 3.14
    ######################################################################    
    def getLatLon(self, lat, lon, distance, angle):
        
#         newLat = math.asin(math.sin(lat)*math.cos(distance)+math.cos(lat)*math.sin(distance)*math.cos(angle))
# 
#         if (math.cos(lat)==0):
#             newLon = lon
#         else:
#             a = math.sin(angle)*math.sin(distance)/math.cos(lat)
#             print '^^'
#             print a
#             print '+++++'
#             b = math.asin(a)
#             newLon = ((lon-b + math.pi) % (2*math.pi))
#             
# still wrong
        
        
#              newLat=asin(sin(lat)*cos(d)+cos(lat1)*sin(d)*cos(tc))
#      IF (cos(lat)=0)
#         lon=lon1      // endpoint a pole
#      ELSE
#         lon=mod(lon1-asin(sin(tc)*sin(d)/cos(lat))+pi,2*pi)-pi
#      ENDIF
# 
#      lat =asin(sin(lat1)*cos(d)+cos(lat1)*sin(d)*cos(tc))
#      dlon=atan2(sin(tc)*sin(d)*cos(lat1),cos(d)-sin(lat1)*sin(lat))
#      lon=mod( lon1-dlon +pi,2*pi )-pi




#         earthRadius = 6378137 ##6371000
        
#         rad = angle*math.pi/180
#         
#         delta = distance/earthRadius
#         
#         a = math.sin(lat)*math.cos(delta)
#         b = math.cos(lat)*math.sin(delta)*math.cos(rad)
#         newLat = math.asin(a+b)
#         
#         x = math.sin(rad)*math.sin(delta)*math.cos(lat)
#         y = math.cos(delta)-math.sin(lat)*math.sin(newLat)
#         newLon = lon + math.atan2(x,y)
        
#         distanceNorth = math.sin(angle) * distance
#         distanceEast  = math.cos(angle) * distance
#            
#          
#         newLat = lat + (distanceNorth / earthRadius) * 180 / math.pi
#         newLon = lon + (distanceEast /  (earthRadius * math.cos(newLat * 180 / math.pi))) * 180 / math.pi
# 
#         return (newLat, newLon)

#         distanceNorth = math.sin(angle) * distance
#         distanceEast  = math.cos(angle) * distance
#         earthRadius   = 6378137 ##6371000
# 
#         newLat = lat + (distanceNorth / earthRadius) * 180 / math.pi
#         newLon = lon + (distanceEast /  (earthRadius * math.cos(newLat * 180 / math.pi))) * 180 / math.pi
    
        distanceNorth = math.cos(angle) * distance
        distanceEast  = math.sin(angle) * distance
        earthRadius   = 6378137 ##6371000

        newLat = lat + (distanceNorth / earthRadius) * 180 / math.pi
        newLon = lon + (distanceEast /  (earthRadius * math.cos(newLat * 180 / math.pi))) * 180 / math.pi

          
        return (newLat, newLon)

    ##########
    ##
    #####
    def getMeters(self, oLat, oLon, nLat, nLon):
        earthRadius = 6378137
        dLat = (nLat - oLat) * math.pi / 180
        dLon = (nLon - oLon) * math.pi / 180
      
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(oLat * math.pi / 180) * math.cos(self.lat * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = earthRadius * c
    
        return d 
    
    ##################################################
    ## start the selected Mission
    ##
    ##################################################
    def startMission(self):
        if (self.connected and self.armed and self.loaded):
            
#           # override send throttle auf 
            self.mavlink_obj.set_mode_send(self.sysID, mavlink0.MAV_MODE_FLAG_AUTO_ENABLED, 3) #0
            if (self.figure == 1):
                self.tB_debug.append("flying Tetrahedron")   
                self.mavlink_obj.command_long_send(self.sysID,
                                                   self.compID,
                                                   mavlink0.MAV_CMD_MISSION_START, #command
                                                   1, # 0confirmation
                                                   1, # param1 first item
                                                   6, # param2 last item
                                                   0, # param3
                                                   0, # param4
                                                   0, # param5
                                                   0, # param6
                                                   0) # param7
            elif (self.figure == 0):      
                self.tB_debug.append("TAKEOFF")              
                #throttle setztn wenn misson statr geht 
                self.mavlink_obj.command_long_send(self.sysID,
                                                   self.compID,
                                                   mavlink0.MAV_CMD_MISSION_START, #command
                                                   1, # 0confirmation
                                                   1, # param1 first item
                                                   3, # param2 last item
                                                   0, # param3
                                                   0, # param4
                                                   0, # param5
                                                   0, # param6
                                                   0) # param7 
            else:
                print "!error !"
        else:
            self.tB_debug.append("ERROR")
   
   
    ####################################################
    ## request selected mission
    ## 0 takeoff
    ## 1 fly tetrahedron
    ## else nothing
    ####################################################
    def requestMisson(self, mode):
        if (mode == 0 and self.connected):
            self.mavlink_obj.mission_count_send(self.sysID, self.compID, 3)
            self.figure = 0
        elif (mode == 1 and self.connected):
            self.mavlink_obj.mission_count_send(self.sysID, self.compID, 7)
            #set mission to fly a pyramide (tetrahedron)
            self.figure = 1
        else:     
            self.tB_debug.append("No copter connected")
           
            
            
    #####################################################    
    ## handle change in sliders and send update to MAV
    ##
    #####################################################
    def handleChanged(self):
        ## send the values to the copter
        if(self.armed):
            self.mavlink_obj.rc_channels_override_send(self.sysID, 
                                               self.compID,
                                               self.rollSlider.value(), 
                                               self.pitchSlider.value(), 
                                               self.throttleSlider.value(), 
                                               self.yawSlider.value(),
                                               0,0,0,0)
    ###########################################################################
    ## display incoming RC messages
    ##
    ###########################################################################   
    def handleRCinput(self, message):
        if (self.armed):
            ##setting displays
            self.lcdRoll.display((message.__getattribute__('chan1_raw')-1500)/10)
            self.lcdPitch.display((message.__getattribute__('chan2_raw')-1500)/10)
            self.lcdThrottle.display((message.__getattribute__('chan3_raw')-1000)/10)
            self.lcdYaw.display((message.__getattribute__('chan4_raw')-1500)/10)
        else:
            self.lcdYaw.display((self.yawSlider.value()-1500)/10)
            self.lcdRoll.display((self.rollSlider.value()-1500)/10)
            self.lcdPitch.display((self.pitchSlider.value()-1500)/10)
            self.lcdThrottle.display((self.throttleSlider.value()-1000)/10)
        ##show signal strength
        signalStreagth = message.__getattribute__('rssi')
        if (signalStreagth != 255):
            self.rssi.setValue(signalStreagth)
        
    #####################################################
    ## arm/disarm the copter
    ##
    #####################################################
    def handleArmButton(self):
        self.pB_arm.setEnabled(False)
         
        if (self.connected):
            self.mavlink_obj.set_mode_send(self.sysID, mavlink0.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
            
            #request all streams
            self.mavlink_obj.request_data_stream_send(self.sysID, self.compID,
                                                  mavlink0.MAV_DATA_STREAM_ALL, 
                                                  2, #requestrate
                                                  1)
       
            ## ARMING
            if(self.armed == False):
                ## set copters throttle to 0% !!!
                if(self.throttleSlider.value() > 1000):
                    self.mavlink_obj.rc_channels_override_send(self.sysID, 
                                                               self.compID, 
                                                               self.yaw,
                                                               self.pitch,
                                                               self.throttle, 
                                                               self.roll, 
                                                               0, 0, 0, 0)
                    self.throttleSlider.setValue(1000);
                    
                ## send arm command to copter 
                self.mavlink_obj.command_long_send(self.sysID,
                                                   mavlink0.MAV_COMP_ID_SYSTEM_CONTROL,
                                                   mavlink0.MAV_CMD_COMPONENT_ARM_DISARM, 
                                                   0, 1, #arm
                                                   0, 0, 0, 0, 0, 0)
                
            ## DISARMING
            elif(self.armed):
                ## send disarm command to copter
                self.mavlink_obj.command_long_send(self.sysID,
                                                   mavlink0.MAV_COMP_ID_SYSTEM_CONTROL,
                                                   mavlink0.MAV_CMD_COMPONENT_ARM_DISARM, 
                                                   0, 0, #disarm
                                                   0, 0, 0, 0, 0, 0)
                
                
        ## else
        else:
            self.tB_debug.setText('there is no copter connected.')
            self.pB_arm.setEnabled(True)

    ######################################################
    ## handle Command ACK
    ##
    ######################################################        
    def handleCommandACK(self, message):
        ## MAV_CMD_COMPONENT_ARM_DISARM
        if(message.__getattribute__('command') == mavlink0.MAV_CMD_COMPONENT_ARM_DISARM): 
            if(message.__getattribute__('result') == mavlink0.MAV_RESULT_ACCEPTED):
                if (self.armed == False):
                    
                    ##open rc stream
                    self.mavlink_obj.request_data_stream_send(self.sysID,
                                                              self.compID, 
                                                              mavlink0.MAV_DATA_STREAM_RC_CHANNELS, 
                                                              1, #request rate
                                                              1) #start
                    self.tB_debug.append("ARMING the copter...")
                
                else:
                    #close RC stream
                    self.mavlink_obj.request_data_stream_send(self.sysID,
                                                              self.compID,
                                                              mavlink0.MAV_DATA_STREAM_RC_CHANNELS, 
                                                              0,
                                                              0) #stop
                    self.tB_debug.append("DISARMING the copter...")
                

            else:
                self.tB_debug.append("NO ACK!")
                self.tB_debug.append (str(message.__getattribute__('result')))
           
        ## different command MAV_CMD_
        else: 
            print "commandACK:"
            print message.__getattribute__('command')
        
    #####################################################
    ## This method handels IMU messages
    ######################################################
    def handleIMUMsg(self, message):
        
        #retrieve values from payload
        xacc = message.__getattribute__('xacc')
        yacc = message.__getattribute__('yacc')
        zacc = message.__getattribute__('zacc')
        
        xgyro = message.__getattribute__('xgyro')
        ygyro = message.__getattribute__('ygyro')
        zgyro = message.__getattribute__('zgyro')
        
        xmag = message.__getattribute__('xmag')
        ymag = message.__getattribute__('ymag')
        zmag = message.__getattribute__('zmag')
        
        ##setting new values
        self.xa = xacc 
        self.ya = yacc
        self.za = zacc
        self.xg = xgyro
        self.yg = ygyro
        self.zg = zgyro
        self.xm = xmag
        self.ym = ymag
        self.zm = zmag
  
        #updateGUI
        if (self.showIMU2 == False):
            self.updateGUI(xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag)
   
    #####################################################
    ## This method handels scaled_IMU2 messages
    ##
    ##
    ######################################################
    def handleIMU2Msg(self, message):
        
        #retrieve values from payload
        xacc = message.__getattribute__('xacc')
        yacc = message.__getattribute__('yacc')
        zacc = message.__getattribute__('zacc')
        
        xgyro = message.__getattribute__('xgyro')
        ygyro = message.__getattribute__('ygyro')
        zgyro = message.__getattribute__('zgyro')
        
        xmag = message.__getattribute__('xmag')
        ymag = message.__getattribute__('ymag')
        zmag = message.__getattribute__('zmag')
        
        self.xa2 = xacc 
        self.ya2 = yacc
        self.za2 = zacc
        self.xg2 = xgyro
        self.yg2 = ygyro
        self.zg2 = zgyro
        self.xm2 = xmag
        self.ym2 = ymag
        self.zm2 = zmag
  
        #display vlaues
        if (self.showIMU2):
            self.updateGUI(xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag)
    
    #####################################################
    ## This method handels GPS messages
    ##
    ######################################################
    def handleGPSMsg(self, message):
        
        #retrieve values from payload
        lat  = message.__getattribute__('lat')/10000000.00
        lon  = message.__getattribute__('lon')/10000000.00
        alt  = message.__getattribute__('alt')/1000.00
        ralt = message.__getattribute__('relative_alt')/1000.00
        vx   = message.__getattribute__('vx')/100.00
        vy   = message.__getattribute__('vy')/100.00
        vz   = message.__getattribute__('vz')/100.00
        hdg  = message.__getattribute__('hdg')/100.00
        
        self.lat = lat
        self.lon = lon
        self.hdg = hdg
        #display values 
        self.lcdXSpeed.display(vx)
        self.lcdYSpeed.display(vy)
        self.lcdZSpeed.display(vz)
        
        self.lcdAlt.display(alt)
        self.lcdRelAlt.display(ralt)
        self.lcdHeading.display(hdg)
        
        self.labelLat.setText(str(lat))
        self.labelLong.setText(str(lon))
        
        self.dialCompass.setValue(hdg)  
        
    #####################################################
    ## This method handles Battery Status
    ##
    ##
    ######################################################
    def handleBattery(self, message):
        self.batteryStatus.setValue(message.__getattribute__('battery_remaining')) 
        
    #####################################################    
    ## handle GUI when ARMED / DISARMED
    ## 
    ######################################################
    def handelArmState(self, message):
        if mavutil.armed(message) == 1:
            self.armStatus.setText('A')
            self.armStatus.setStyleSheet("color: rgb(200, 0, 0)")
            self.armed = True
            self.pB_arm.setStyleSheet("color: rgb(0, 200, 0)")
            self.pB_arm.setText('Disarm')
            
            ##enable Sliders if copter is ARMED
            self.throttleSlider.setEnabled(True)
            self.yawSlider.setEnabled(True)
            self.pitchSlider.setEnabled(True)
            self.rollSlider.setEnabled(True)
          
        else:
            self.armed = False
            self.armStatus.setText('D')
            self.armStatus.setStyleSheet("color: rgb(0, 200, 0)")
            self.pB_arm.setStyleSheet("color: rgb(200, 0, 0)")
            self.pB_arm.setText('Arm')
            
            ## set to init value 
            self.throttleSlider.setValue(1000)
            self.rollSlider.setValue(1500)
            self.yawSlider.setValue(1500)
            self.pitchSlider.setValue(1500)
       
            ##disable Sliders while DISARMED
            self.throttleSlider.setEnabled(False)
            self.yawSlider.setEnabled(False)
            self.pitchSlider.setEnabled(False)
            self.rollSlider.setEnabled(False)
        ##enable arm button again
        self.pB_arm.setEnabled(True)
    
    
    #########################
    ## Set sys and comp ID
    #####################################     
    def setID(self):
        self.sysID = self.sysIDBox.value()
        self.compID = self.compIDBox.value()
        
    #####################################################
    ## This method handles Heartbeatmessages
    ##
    ##
    ######################################################
    def handleHeartbeatMsg(self, sysId, compId, message):
        #get time
        time = str(datetime.datetime.now().time())
        # get state  from payload and match it     
        state = mavlink0.enums['MAV_STATE'][message.__getattribute__('system_status')].name
        
        #get row count   
        maxRows = self.tableWidget.rowCount()
        
        #update UI
        item = QtGui.QTableWidgetItem()
        
        #row check    
        for i in range(0,maxRows) :
 
            self.mavlink_obj.rc_channels_override_send(self.sysID, 
                                                       self.compID,
                                                       self.rollSlider.value(), 
                                                       self.pitchSlider.value(), 
                                                       self.throttleSlider.value(), 
                                                       self.yawSlider.value(),
                                                       0,0,0,0)
            
            #get sysId and compId strings
            if self.tableWidget.item(i,1) != None:
                item = self.tableWidget.item(i,1)
                columnSysID = QtCore.QString()
                columnSysID = item.text()
                item = self.tableWidget.item(i,2)
                columnCompID = QtCore.QString()
                columnCompID = item.text()
                    
            #update if empty
            if self.tableWidget.item(i,1) == None :
                self.tableWidget.setItem(i,0, QtGui.QTableWidgetItem(time))    
                self.tableWidget.setItem(i,1, QtGui.QTableWidgetItem(str(sysId)))   
                self.tableWidget.setItem(i,2, QtGui.QTableWidgetItem(str(compId)))   
                self.tableWidget.setItem(i,3, QtGui.QTableWidgetItem("HEARTBEAT"))             
                self.tableWidget.setItem(i,4, QtGui.QTableWidgetItem(state))   
                #add new row for new data
                self.tableWidget.setRowCount(maxRows+1)   
                
                    
            #update SysId and CompID entries if occurred previously
            elif columnSysID == str(sysId) and columnCompID == str(compId):
                self.tableWidget.setItem(i,0, QtGui.QTableWidgetItem(time))    
                self.tableWidget.setItem(i,3, QtGui.QTableWidgetItem("HEARTBEAT"))
                self.tableWidget.setItem(i,4, QtGui.QTableWidgetItem(state)) 
                break
       
    #################################################################    
    ## This method handles the Emergency Modus
    ##
    ##################################################################
    def handleEmergencyButton(self):
        #connected ?
        if self.armed == True:
            self.tB_debug.append("Emergency-Modus engaged!")
           
            self.mavlink_obj.command_long_send(self.sysID,
                                               self.compID,
                                               mavlink0.MAV_CMD_NAV_LAND, #command
                                               1, # 0confirmation
                                               1, # param1 first item
                                               3, # param2 last item
                                               0, # param3
                                               0, # param4
                                               0, # param5
                                               0, # param6
                                               0) # param7  
        else:
            self.tB_debug.append("Copter not ARMED")
            
    ##########################################################################        
    ## togleView
    ##########################################################################
    def toggleView(self):
        self.showIMU2 = not (self.showIMU2)
        if (self.showIMU2):
            self.updateGUI(self.xa2, self.ya2, self.za2, self.xg2, self.yg2, self.zg2, self.xm2, self.ym2, self.zm2)
        else:
            self.updateGUI(self.xa, self.ya, self.za, self.xg, self.yg, self.zg, self.xm, self.ym, self.zm)
    
    ############################################################################ 
    ## update GUI
    ############################################################################
    def updateGUI(self, xa, ya, za, xg, yg, zg, xm, ym, zm):
        self.labelAccX.display(xa)
        self.labelAccY.display(ya)
        self.labelAccZ.display(za)
        
        self.labelGyroX.display(xg)
        self.labelGyroY.display(yg)
        self.labelGyroZ.display(zg)
        
        self.labelMagX.display(xm)
        self.labelMagY.display(ym)
        self.labelMagZ.display(zm)
    
    ##################################################################################    
    ##    request GPS stream
    ##################################################################################    
    def handleGPSrequest(self):
        if self.GPSactive == False:
            #request GPS messages  
            self.mavlink_obj.request_data_stream_send(self.sysID,
                                                      self.compID,
                                                      mavlink0.MAV_DATA_STREAM_POSITION, 
                                                      8,1)
            self.GPSactive = True
            
        else:
            self.mavlink_obj.request_data_stream_send(self.sysID,
                                                      self.compID,
                                                      mavlink0.MAV_DATA_STREAM_POSITION,
                                                      8, 0)
            self.IMUactive = False
            
    ###################################################################################    
    ##    request IMU stream
    ###################################################################################    
    def handleIMUrequest(self):
        if self.IMUactive == False:
            # request IMU messages 
            self.mavlink_obj.request_data_stream_send(self.sysID,
                                                      self.compID,
                                                      mavlink0.MAV_DATA_STREAM_RAW_SENSORS,
                                                      8, 1)
            self.IMUactive = True
        else:
            self.mavlink_obj.request_data_stream_send(self.sysID,
                                                      self.compID,
                                                      mavlink0.MAV_DATA_STREAM_RAW_SENSORS, 
                                                      8, 0)
            self.IMUactive = False
                
    
    ###########################################################################
    ## This method handles the events on the connect button in the GUI
    ## 
    ## If it pushed once it opens a serial port and starts a listener thread 
    ## to receive serial data. It also generates a MAVLink object to prepare
    ## and send MAVLink messages.
    ##
    ## If its pushed twice it disconnects the serial communication by closing
    ## serial port
    ###########################################################################    
    def handleConnectButton(self):
         
        ## if no connection to other devices is established yet
        if self.connected == False:
            
            ## this try-exception block catches any exception occurring during opening of
            ## the serial port and prints a message to the debug window
            try:
                ## if the debug parameter given to the GUI on start is zero the software
                ## expected a connected telemetry module and tries to open a virtual serial
                ## COM port 
                if self.debug == 0:
                    ## open a serial port at USB0 with a baudrate of 57 kBaud. The telemtry
                    ## module is expected at this port. If its not there it is necessary to
                    ## adapt the module name here
                    self.quadport = serial.Serial(port = '/dev/ttyUSB0', baudrate = '57600')
                
                ## if the debug parameter is one no telemetry module is expected and the a
                ## serial COM port is opened that only loops the sent data back to the
                ## receiver. That can be used for test purposes   
                else:
                    self.quadport = serial.serial_for_url('loop://')#,timeout = 5)
            
            ## here are the exception listed to be caught. Here is also an implementation of
            ## what should happen if one of these exceptions is thrown 
            except serial.SerialException, e:
                self.tB_debug.append("Error while opening serial port. (port available? port occupied?)"+'\n'+e+'\n'+"-----")
            
            
            ## if a serial port has been opened set system status to serial connect, generate a mavlink object
            ## for communication using the mavlink protocol, open a thread listening on the serial port for
            ## incoming data broadcast a ping request  to get information on the neighborhood.
            if self.quadport.isOpen() == True:
                
                ## set system connection status to connected (true)
                self.connected = True
                
                ## change button text back to "Disconnect"
                self.pB_connect.setText("Disconnect")
            
                ## remove the disconnected widget from the statusbar of the GUI and append the
                ## connected widget and make it visible
                self.statusBar().removeWidget(self.dis)
                QtGui.QStatusBar.addWidget(self.statusbar, self.con)
                self.con.show()
                
                ## bind the serial port to the MAVLINK protocol module so the protocol can
                ## use the serial port for communication and set the system and component ID of the ground station
                self.mavlink_obj = mavlink0.MAVLink(self,file = self.quadport, srcComponent = 255, srcSystem = 255)
            
                ## create a SerialListen thread listening on the opened serial port for
                ## incoming data from the telemetry module
                self.listenThread = SerialListen.SerialListen(self, self.quadport,self.mavlink_obj)
            
                ## connect the new_message signal of the serial listener thread with the 
                ## on_new_message method in this class. So the data is given from the
                ## serial listener thread to the main GUI thread without blocking it.
                ## Else the program throws errors.
                self.connect(self.listenThread, self.listenThread.new_message, self.onSerialRX)
            
                ## start the thread listening on the serial port for incoming data
                self.listenThread.start()
                
                #request gps stream
                self.mavlink_obj.request_data_stream_send(self.sysID,
                                                          self.compID,
                                                          mavlink0.MAV_DATA_STREAM_POSITION, 
                                                          8,1)
                
                ## start emulator if debug mode
                if self.debug == 1:
                    mavlink_parser.startQCEmulator(self.mavlink_obj)
                    
                ## broadcast a ping message to see if anybody is around 
                
                ## get the current system time
                #then = datetime.datetime.now()
            
                ## transform the current system time in a microsecond format
                #usec = time.mktime(then.timetuple()) * 1000000
            
                ## if in debug mode, start sending heartbeat messages
                if self.debug == 1:
                    time.sleep(2)
                    self.mavlink_obj.heartbeat_send(mavlink0.MAV_TYPE_QUADROTOR, mavlink0.MAV_AUTOPILOT_PX4, mavlink0.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, mavlink0.MAV_STATE_STANDBY)                
                
        ## if system is connected and the connect button is pushed again, disconnect the system by terminating 
        ## the listener thread, closing the serial port and setting the system status to disconnected.   
        else:
            ## remove the connected widget from the statusbar of the GUI and append the
            ## disconnected widget and make it visible
            self.statusBar().removeWidget(self.con)
            QtGui.QStatusBar.addWidget(self.statusbar, self.dis)
            self.dis.show()
            
            ## set system connection status to disconnected = False
            self.connected = False
            ## reset data stream
           
            self.mavlink_obj.request_data_stream_send(self.sysID,
                                                      self.compID,
                                                      mavlink0.MAV_DATA_STREAM_ALL, 0,0)
            
            
            self.throttleSlider.setEnabled(True)
            self.yawSlider.setEnabled(True)
            self.pitchSlider.setEnabled(True)
            self.rollSlider.setEnabled(True)
            
            ## change button text back to "Disconnect"
            self.pB_connect.setText("Connect")
           
            
            ## stop the thread
            self.listenThread.terminate()
            
            ## close the serial port
            self.quadport.close()
            
        
###########################################################################
## Open the GUI on start and execute a system exit when its close to destroy
## all elements of the application (threads, ports, ...)
## 
########################################################################### 
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())