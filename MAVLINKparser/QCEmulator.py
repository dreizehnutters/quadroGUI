 ###############################################################################
###############################################################################
## Quadrocopter Emulator
##
## This file contains an emulator for Quadrocopter telemtry communication
##
## author: Nicole Todtenberg, Thomas Basmer
##
## date: 2015-04-29
##
## version: 0.01
##
## changes:
##    - 0.00 2015-04-29
##        - initial version
##    - 0.01 2015-06-12
##        - arm command only accepted if component ID is 250
##        - copter is set with and system id on start up and checks this for incomming messages
##            - this enables the emulation of several copters
##        - class for mission items inserted
##        - start and land emulation integrated
##        - emulates start to an specified height and landing
##    - 0.02 2015-06-30
##        - arm or disarm value (param_1) expected at wrong position and it is a float and has to be converted from 4 byte to float/int
###############################################################################
###############################################################################

## this class is generated using the pymavlink generator. It provides, objects,
## methods and aliases to work on mavlink messages.
import mavlink0

## threads are used to realize periodical sending of messages
import threading
#from PyQt4 import QtCore

## used to get system time
import time

## used to generate random numbers
from random import randint, random

import struct

import MissionItem

class QCEmulator(threading.Thread):
#class QCEmulator(QtCore.QThread): 
   
    ###########################################################################
    ## init method
    ## - inits variables
    ##########################################################################
    def __init__(self,obj,sysID=1):
       
        super(QCEmulator,self).__init__()
       
        ## mavlink object to use mavlink methods and objects
        self.mavlink_obj = obj

        ## counter used to count intervals
        self.count = 0
       
        ## list storing which data stream is enabled
        self.stream   = [0]*255
       
        ## list storing interval each stream sends
        self.streamPer = [0]*255
       
        ## start time of the parser, used to emulate system boot time
        self.startTime = time.time()
       
        ## set the system id of the copter
        self.system_id = sysID
       
        ## set the system id of the copter
        self.component_id = 1
       
        ## set quadrocopter to disarmed mode - RC control is not possible
        self.isArmed = False
       
        self.isStoped = False
       
        ## initialize the mission item value to zero
        self.mission_items = 0
       
        ## init rc channel values
        self.yaw = 1500
        self.throttle = 1000
        self.pitch = 1500
        self.roll = 1500
       
        ## array holding mission items
        self.mission_items_array = [MissionItem.MissionItem() for i in range(255)]
       
        ## the quadrocopter is initialized that it is not on a mission
        self.arm_mission = False
        self.on_mission = False
       
        ## set mode of the quadrocopter
        self.mode = mavlink0.MAV_MODE_MANUAL_DISARMED
       
        ## external event to set next item
        self.next_item = False
       
        ## current mission item (not zero because first item is a dummy item)
        self.current_item = 1
       
        ## set initial position of the quadrocopter in mm
        self.long = 143270530
        self.lat = 517650230
        self.alt = 76000
        self.heading = 36000
       
        ## speed in m/s * 100
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0
       
        ## todo
        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
   
       
    def run(self):
       
        while self.isStoped == False:
           
            time.sleep(1)
           
            if self.count % 11 == 0:
                self.sendHeartbeat()
                self.sendSysStatus()
       
           
            ## if stream 1 is enabled
            if self.stream[1] == 1:
                   
                ## counter has reached next sending time
                if self.count % self.streamPer[1] == 0:
                   
                    ## send scaled imu2 message
                    self.sendScaledImu2()
                   
                    self.sendRawImu()
                   
                    self.sendSystemTime()
       
            ## if stream 6 is enabled
            if self.stream[6] == 1:
                   
                ## counter has reached next sending time
                if self.count % self.streamPer[6] == 0:
                   
                    self.sendGps()
                   
           
            ## if stream 3 is enabled
            if self.stream[3] == 1:
               
                self.sendRCValues()
                                       
            ## period counter
            if self.count < 600:
                self.count = self.count + 1
            else:
                self.count = 0
           
           
            ###################################################################   
            ## mission handling ###############################################
            ###################################################################
           
            ## if quadrocopter is on a mission
            if self.on_mission == True:
               
                ## regulating height (z-axis)
                ## if qc has to climb
                if self.mission_items_array[self.current_item].pos_z > self.pos_z:
                    self.throttle = 1600
                    self.speed_z = 100
                   
                ## if qc must sink   
                elif self.mission_items_array[self.current_item].pos_z < self.pos_z:
                    self.throttle = 1200
                    self.speed_z = -100
                   
                ## if qc has reached its height
                else:
                    self.throttle = 1350
                    self.speed_z = 0
                   
                ## regulating the x coordinate
                ## if qc is left from the x position 
                if self.mission_items_array[self.current_item].pos_x > self.pos_x:
                    self.roll = 1700
                    self.speed_x = 100
                   
                ## if qc is right from the x position   
                elif self.mission_items_array[self.current_item].pos_x < self.pos_x:
                    self.roll = 1300
                    self.speed_x = -100
                   
                ## if qc has x
                else:
                    self.roll = 1500
                    self.speed_x = 0
                   
                ## regulating the y coordinate
                ## if qc is behind the y position 
                if self.mission_items_array[self.current_item].pos_y > self.pos_y:
                    self.pitch = 1300
                    self.speed_y = -100
                   
                ## if qc is in front of the y position   
                elif self.mission_items_array[self.current_item].pos_y < self.pos_y:
                    self.pitch = 1700
                    self.speed_y = 100
                   
                ## if qc has x
                else:
                    self.pitch = 1500
                    self.speed_y = 0
                   
                ## mission item has been completed go to next item
                if (self.pos_x + 10 >= self.mission_items_array[self.current_item].pos_x and self.pos_x - 10 <= self.mission_items_array[self.current_item].pos_x) and(self.pos_y + 10 >= self.mission_items_array[self.current_item].pos_y and self.pos_y - 10 <= self.mission_items_array[self.current_item].pos_y) and(self.pos_z + 500 >= self.mission_items_array[self.current_item].pos_z and self.pos_z - 500 <= self.mission_items_array[self.current_item].pos_z):
                   
                    ## go to next item if auto continue is set
                    if self.mission_items_array[self.current_item].autocontinue == 1:
                        if self.current_item < self.mission_items - 1:
                            self.current_item = self.current_item + 1
                           
                    ## wait for event to go to next item
                    else:
                        if self.next_item == True:
                            self.current_item = self.current_item + 1
                            self.next_item = False
               
                ## until no flight simulation is only active during mission
                ## later it should be available during all flight modes
                self.flight(self.throttle, self.roll, self.pitch)
               
               
    ###############################################################################
    ## flight model of the qc
    ##
    ## it is just a simple flight model -> should be replaced by a better one
    ##    - yaw is not used
    ##    - assumed qc is directed to the nord
    ##    - pitch changes y position (north to south)
    ##    - roll changes x position (east to west)
    ##    - throttle changes z position (climb and sink)
    ###############################################################################
    def flight(self, throttle, roll, pitch):
       
        ## simulate climb and sink rate
        if(throttle > 1350):
            self.pos_z = self.pos_z + 300
        elif (throttle < 1350):
            self.pos_z = self.pos_z - 300
           
           
        ## simulate x direction
        if(roll > 1500):
            self.pos_x = self.pos_x + 5
        elif (roll < 1350):
            self.pos_x = self.pos_x - 5
           
        ## simulate y direction
        if(pitch > 1500):
            self.pos_y = self.pos_y - 5
        elif (pitch < 1350):
            self.pos_y = self.pos_y + 5
   
   
    ###############################################################################
    ## This method sends emulated heartbeat messages
    ###############################################################################           
    def sendHeartbeat(self):
   
        custom_mode = mavlink0.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
       
        if self.isArmed == True:
            custom_mode = custom_mode + mavlink0.MAV_MODE_FLAG_SAFETY_ARMED
   
        self.mavlink_obj.heartbeat_send(mavlink0.MAV_TYPE_QUADROTOR, mavlink0.MAV_AUTOPILOT_PX4, custom_mode, randint(0,8),3)
   
   
    ###############################################################################
    ## This method sends emulated SYS_STATUS messages
    ###############################################################################           
    def sendSysStatus(self):
   
        custom_mode = mavlink0.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
       
        if self.isArmed == True:
            custom_mode = custom_mode + mavlink0.MAV_MODE_FLAG_SAFETY_ARMED
   
        self.mavlink_obj.sys_status_send(6421551, 6421551, 6421551, randint(20,150), randint(14000,16000), randint(400,550), randint(75,99), 0, 0, 0, 0, 0, 0)
        #self.mavlink_obj.sys_status_send(6421551, 6421551, 6421551, 20, 14000, 400, 75, 0, 0, 0, 0, 0, 0)
       
       
    ###############################################################################
    ## This method sends emulated scaled imu2 messages
    ###############################################################################
    def sendScaledImu2(self):
        #self.mavlink_obj.scaled_imu2_send( (time.time()-self.startTime),randint(-32767,32767),randint(-32767,32767),randint(-32767,32767),int(randint(1,360)*0.017*1000),int(randint(1,360)*0.017*1000),int(randint(1,360)*0.017*1000),randint(1,1000),randint(1,1000),randint(1,1000))
        self.mavlink_obj.scaled_imu2_send( (time.time()-self.startTime),((-1)*32767),randint(-32767,32767),randint(-32767,32767),int(randint(1,360)*0.017*1000),int(randint(1,360)*0.017*1000),int(randint(1,360)*0.017*1000),randint(1,1000),randint(1,1000),randint(1,1000))
     
     
    ###############################################################################
    ## This method sends emulated raw imu messages
    ###############################################################################
    def sendRawImu(self):
        self.mavlink_obj.raw_imu_send(time.time()*1000,randint(0,100),randint(0,100),randint(0,100),int(randint(1,360)*0.017*1000),int(randint(1,360)*0.017*1000),int(randint(1,360)*0.017*1000),randint(1,1000),randint(1,1000),randint(1,1000))
     
     
    ###############################################################################
    ## This method sends emulated system time messages
    ###############################################################################
    def sendSystemTime(self):
     
        self.mavlink_obj.system_time_send(time.time()*1000 , time.time() - self.startTime)
   
   
    ###############################################################################
    ## This method sends emulated gps messages
    ###############################################################################
    def sendGps(self):
       
        #alt = randint(10000, 100000)
        #self.mavlink_obj.global_position_int_send(time.time()-self.startTime, 517650230, 143270530, alt, alt-100, randint(0,400), randint(0,400), randint(0,400), randint(1,360))
        self.mavlink_obj.global_position_int_send( (time.time()-self.startTime), self.lat, self.long, self.alt, self.alt-76000, self.speed_x, self.speed_y, self.speed_z, self.heading)
   
   
    ###############################################################################
    ## This method sends emulated rc values messages
    ###############################################################################
    def sendRCValues(self):
       
        self.mavlink_obj.rc_channels_raw_send( (time.time()-self.startTime), 1, self.roll, self.pitch, self.throttle, self.yaw, 0, 0, 0, 0, randint(0,100))
       
       
    ###############################################################################
    ## This method emulates the quadrocopter receive message
    ###############################################################################
    def generate(self, msgType, sysID, comp, payload, mavObj):
       
        ## if addressed copter is this copter
        if self.system_id == sysID:
           
            ## if received message is a parameter request
            if msgType == mavlink0.MAVLINK_MSG_ID_PARAM_REQUEST_READ:         
               
                ## temporary string to store the parameter id
                ## this is needed when the string in the payload is shorter than 16 chars
                ## shorter strings are terminated using a Null byte
                tmp = []
                s = ""
                ## run through the payload section containing the parameter ID string
                ## store every char in the temporary string until a Null byte is read.
                for i in range(4,20):
                    if payload[i] != 0x00:
                        tmp.append(str(unichr(payload[i])))
                    else:
                        break   
               
                ## transform array into string
                s=''.join(tmp)
           
                ## if parameter request is a voltage request
                if str(s) == "FS_BATT_VOLTAGE":
                   
                    # build a parameter response with randomly generated voltage value
                    mavObj.param_value_send(s,3.00 + random(), mavlink0.MAVLINK_TYPE_FLOAT, 331, payload[0])
           
            ## if received request is for a data stream       
            elif msgType == mavlink0.MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
               
                ## check which stream should be started/stopped
           
                ## enable/disable streams and set their sending interval
                self.stream[payload[4]] = payload[5]
                self.streamPer[payload[4]] = payload[0]
           
            ## if received packet is a long command
            elif msgType == mavlink0.MAVLINK_MSG_ID_COMMAND_LONG:
                         
                ## if received command is a arm/disarm command
                ## it is necessary to calculate the modulo value because the field in the packet is only
                ## 8 bit but the command value exceeds this space. The send value is 400 but the data in
                ## the package is 400 mod 256
                if payload[28] == (mavlink0.MAV_CMD_COMPONENT_ARM_DISARM % 256) and comp == 250:
                   
                    arm_value = int((struct.unpack('f',"".join(chr(payload[i]) for i in range (0 ,4))))[0])
                   
                    ## if it should be armed
                    if arm_value == 1:
                        self.isArmed = True
                        self.mode = mavlink0.MAV_MODE_MANUAL_ARMED

                    else:
                        self.isArmed = False
                        self.mode = mavlink0.MAV_MODE_MANUAL_DISARMED
                       
                    self.mavlink_obj.command_ack_send(mavlink0.MAV_CMD_COMPONENT_ARM_DISARM, mavlink0.MAV_RESULT_ACCEPTED)
                   
                ## if mission start command hat been received   
                elif payload[28] == (mavlink0.MAV_CMD_MISSION_START % 256):
                   
                    ## set quadrocopter is on a mission
                    self.arm_mission = True
                   
                    self.mavlink_obj.mission_ack_send(self.system_id, comp, mavlink0.MAV_MISSION_ACCEPTED)
                       
            ## if received packet is an overwrite channel packet
            elif msgType == mavlink0.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                             
                ## only react on changes if device is armed
                if self.isArmed == 1:
 
                    ## throttle values were sent using channel three
                    self.throttle = (payload[5] << 8) + payload[4]
                    #print 'emulator::throttle: '+str(self.throttle)
           
                    ## roll values were sent using channel one
                    self.roll = (payload[1] << 8) + payload[0]
                    #print 'emulator::roll: '+str(self.roll)
           
                    ## yaw values were sent using channel four
                    self.yaw = (payload[7] << 8) + payload[6]
                    #print 'emulator::yaw: '+str(self.yaw)
           
                    ## pitch values were sent using channel two
                    self.pitch = (payload[3] << 8) + payload[2]
                    #print 'emulator::pitch: '+str(self.pitch)
                   
                    ## if qc is in auto mode and throttle value is > 10 % start the insert mission
                    if self.mode == mavlink0.MAV_MODE_MANUAL_ARMED and self.arm_mission == True:
                       
                        self.on_mission = True
           
            ## if received command sets the amount of mission items
            elif msgType == mavlink0.MAVLINK_MSG_ID_MISSION_COUNT:
               
                ## set mission items
                self.mission_items = (payload[1] << 8) + payload[0]
               
                ## request first mission item
                self.mavlink_obj.mission_request_send(self.system_id, 1, 0)

             
            ## if mission item has been received   
            elif msgType == mavlink0.MAVLINK_MSG_ID_MISSION_ITEM:
               
               
                item_id = (payload[29] << 8) + payload[28]
               
                ## if not all mission items have been received
                if  item_id < self.mission_items:
                   
                    ## insert mission item
                    self.mission_items_array[item_id].frame           = payload[34]
                    self.mission_items_array[item_id].command         = (payload[31] << 8) + payload[30]
                    self.mission_items_array[item_id].current         = payload[35]
                    self.mission_items_array[item_id].autocontinue    = payload[36]
                    self.mission_items_array[item_id].pos_x           = int((struct.unpack('f',"".join(chr(payload[i]) for i in range (16 ,20))))[0])
                    self.mission_items_array[item_id].pos_y           = int((struct.unpack('f',"".join(chr(payload[i]) for i in range (20 ,24))))[0])
                    self.mission_items_array[item_id].pos_z           = int((struct.unpack('f',"".join(chr(payload[i]) for i in range (24 ,28))))[0]*1000)
                   
                    ## request first mission item
                    self.mavlink_obj.mission_request_send(self.system_id, 1, item_id + 1)
           
            ## set qc mode   
            elif msgType == mavlink0.MAVLINK_MSG_ID_SET_MODE:

                ## if quadrocopter is in auto mode
                if payload[5] == mavlink0.MAV_MODE_FLAG_AUTO_ENABLED:
                   
                    ## set quadrocopter is on a mission
                    self.arm_mission = True               
           
           
    ###############################################################################
    ## This method closes the emulator thread
    ###############################################################################
    def close(self):
        print "close"
        #self.join(0)
        self.isStoped = True
        #sys.exit()