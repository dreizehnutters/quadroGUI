###############################################################################
## Parser to parse mavlink messages.
## Messages are send by the quadrocopter emulator and the real quadrocopter
##
## Author:     n.todtenberg, t.basmer - IHP
## Date:       2014-08
## Version     0.01
##
## changes
##    - v0.00 - 2014-08
##        - initial version
##    - v0.01 - 2015-06-10
##        - handlers for mission items inserted
###############################################################################

import mavlink0
import datetime, time
from random import randint

#from os import sys

from QCEmulator import QCEmulator

DEBUG = 0

qc = None

def init(mavObj):
    
    global qc
    
    qc = QCEmulator(mavObj,1)
    
    qc.isStoped = False
    
    qc.start()

###############################################################################
## This method checks if a received message is a request to the emulator
## only used in debug mode
## normal-user should not use these method
## return True if message was for the QC emulator and else false
###############################################################################
def sendToEmulator(mavObj,msg, debug):
    
    if parse(mavObj,msg, debug) == None:
        return True
    else:
        return False

def startQCEmulator(mavObj):
    init(mavObj)
    
def stopQCEmulator():
    qc.close()   
    
def parse(mavObj,msg, debug):    

    global DEBUG
    
    DEBUG = debug
    # cast event to MAVLink_message
    #print msg.encode('hex')
    try:
        MAVmsg = mavObj.parse_char(msg)
    except:
        MAVmsg = None

    if MAVmsg != None:
        
        # switch message type
        if MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_BAD_DATA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_BAD_DATA: should not happen'
            #return EBADM 
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HEARTBEAT:

            ## if system is in debug mode return a heartbeat message with random values to fill table
            if DEBUG == 1:
                # lab1
                #return mavlink0.MAVLINK_MSG_ID_HEARTBEAT, randint(0,1), randint(0,2), MAVmsg.get_payload()
                #lab<1
                return mavlink0.MAVLINK_MSG_ID_HEARTBEAT, qc.system_id, qc.component_id, MAVmsg.get_payload()
            else:
                return mavlink0.MAVLINK_MSG_ID_HEARTBEAT, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()   
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SYS_STATUS:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_SYS_STATUS | not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_SYS_STATUS, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
            #return ENOSYS
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SYSTEM_TIME:
            
            return mavlink0.MAVLINK_MSG_ID_SYSTEM_TIME, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        # if ping message received, response with correct ping
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_PING:
            # send a ping message with appropriate parameters as response
            return handle_ping(MAVmsg, mavObj)
            
                
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SCHANGE_OPERATOR_CONTROL | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_AUTH_KEY:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_AUTH_KEY | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_MODE:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_MODE | not implemented yet'
            if DEBUG == 1:
                handle_mode_set(MAVmsg, mavObj)
        
            return None
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_PARAM_REQUEST_READ:
           
            handle_param_request_read(MAVmsg, mavObj)
        
            return None
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_PARAM_REQUEST_LIST | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_PARAM_VALUE:
            
            return mavlink0.MAVLINK_MSG_ID_PARAM_VALUE, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_PARAM_SET:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_PARAM_SET | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GPS_RAW_INT:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_GPS_RAW_INT | not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_GPS_RAW_INT, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GPS_STATUS:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GPS_STATUS | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SCALED_IMU:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SCALED_IMU | not implemented yet'
            
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_RAW_IMU:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_RAW_IMU | not implemented yet'
            return handle_raw_imu(MAVmsg, mavObj)
            
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_RAW_PRESSURE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_RAW_PRESSURE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SCALED_PRESSURE:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_SCALED_PRESSURE | not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_SCALED_PRESSURE, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_ATTITUDE:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_ATTITUDE | not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_ATTITUDE, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_ATTITUDE_QUATERNION | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOCAL_POSITION_NED | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_GLOBAL_POSITION_INT | not implemented yet'
            return handle_global_position_int(MAVmsg, mavObj)
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_RC_CHANNELS_SCALED | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_RC_CHANNELS_RAW:
           
            return mavlink0.MAVLINK_MSG_ID_RC_CHANNELS_RAW, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
           
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_SERVO_OUTPUT_RAW | not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_STATUSTEXT, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_ITEM:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_ITEM: not implemented yet'
            if DEBUG == 1:
                handle_mission_item_send(MAVmsg, mavObj)
            
            return None
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_REQUEST:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_REQUEST: not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_MISSION_REQUEST, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_SET_CURRENT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_SET_CURRENT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_CURRENT:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_CURRENT: not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_MISSION_CURRENT, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_REQUEST_LIST: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_COUNT:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_COUNT: not implemented yet'
            ## if system is in debug mode return a heartbeat message with random values to fill table
            if DEBUG == 1:
                handle_mission_item_count(MAVmsg, mavObj)
                #return mavlink0.MAVLINK_MSG_ID_HEARTBEAT, randint(0,9), randint(50,60), MAVmsg.get_payload()
            
            return None
            
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_CLEAR_ALL: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_ITEM_REACHED: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MISSION_ACK:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MISSION_ACK: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_GLOBAL_POSITION_SETPOINT_INT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_GLOBAL_POSITION_SETPOINT_INT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA: not implemented yet'
           
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_STATE_CORRECTION:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_STATE_CORRECTION: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_RC_CHANNELS:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_RC_CHANNELS: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_REQUEST_DATA_STREAM: not implemented yet'
            handle_request_data_stream(MAVmsg, mavObj)
            
            return None
            
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_DATA_STREAM:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_DATA_STREAM: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MANUAL_CONTROL:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MANUAL_CONTROL: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: not implemented yet'

            handle_channels_overwrite(MAVmsg, mavObj)
            
            return None
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_VFR_HUD:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_VFR_HUD: not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_VFR_HUD, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_COMMAND_LONG:
            handle_long_command(MAVmsg, mavObj)
            
            return None
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_COMMAND_ACK:
            return mavlink0.MAVLINK_MSG_ID_COMMAND_ACK, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MANUAL_SETPOINT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MANUAL_SETPOINT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_ATTITUDE_SETPOINT_EXTERNAL:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_ATTITUDE_SETPOINT_EXTERNAL: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOCAL_NED_POSITION_SETPOINT_EXTERNAL: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_EXTERNAL_INT: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIL_STATE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIL_STATE: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIL_CONTROLS:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIL_CONTROLS: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_OPTICAL_FLOW:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_OPTICAL_FLOW: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIGHRES_IMU:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIGHRES_IMU | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_OMNIDIRECTIONAL_FLOW | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIL_SENSOR:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIL_SENSOR | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SIM_STATE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SIM_STATE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_RADIO_STATUS:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_RADIO_STATUS | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_FILE_TRANSFER_START:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_FILE_TRANSFER_START: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_FILE_TRANSFER_DIR_LIST | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_FILE_TRANSFER_RES:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_FILE_TRANSFER_RES: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIL_GPS:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIL_GPS | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIL_OPTICAL_FLOW | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_HIL_STATE_QUATERNION | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SCALED_IMU2:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_SCALED_IMU2 | not implemented yet'
            
            return handle_scaled_imu2(MAVmsg, mavObj)
             
            
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOG_REQUEST_LIST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOG_REQUEST_LIST | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOG_ENTRY:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOG_ENTRY | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOG_REQUEST_DATA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOG_REQUEST_DATA | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOG_DATA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOG_DATA | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOG_ERASE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOG_ERASE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_LOG_REQUEST_END:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_LOG_REQUEST_END | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GPS_INJECT_DATA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GPS_INJECT_DATA | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GPS2_RAW:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GPS2_RAW | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_POWER_STATUS:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_POWER_STATUS | not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_POWER_STATUS, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SERIAL_CONTROL:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SERIAL_CONTROL | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GPS_RTK:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GPS_RTK | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_GPS2_RTK:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_GPS2_RTK | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_ENCAPSULATED_DATA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_ENCAPSULATED_DATA: not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_DISTANCE_SENSOR:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_DISTANCE_SENSOR | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_TERRAIN_REQUEST:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_TERRAIN_REQUEST | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_TERRAIN_DATA:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_TERRAIN_DATA | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_TERRAIN_CHECK:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_TERRAIN_CHECK | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_TERRAIN_REPORT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_TERRAIN_REPORT | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_BATTERY_STATUS:
            return mavlink0.MAVLINK_MSG_ID_BATTERY_STATUS, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_BATTERY_STATUS | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SETPOINT_8DOF:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SETPOINT_8DOF | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_SETPOINT_6DOF:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_SETPOINT_6DOF | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_MEMORY_VECT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_MEMORY_VECT | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_DEBUG_VECT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_DEBUG_VECT | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_NAMED_VALUE_FLOAT | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_NAMED_VALUE_INT:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_NAMED_VALUE_INT | not implemented yet'
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_STATUSTEXT:
            #print 'mavlink_parser.py::MAVLINK_MSG_ID_STATUSTEXT | not implemented yet'
            return mavlink0.MAVLINK_MSG_ID_STATUSTEXT, MAVmsg.get_header().srcSystem, MAVmsg.get_header().srcComponent, MAVmsg.get_payload()
        
        elif MAVmsg.get_header().msgId == mavlink0.MAVLINK_MSG_ID_DEBUG:
            print 'mavlink_parser.py::MAVLINK_MSG_ID_DEBUG | not implemented yet'
        
        else:
            print 'mavlink_parser.py::unknown message ID!'
        
        return None#'None', '', '', ''
    
    else:
        print 'mavlink_parser.py::error mavlink parse!'
        return 'Error','','',''

###############################################################################
## handle the reception of a ping request
###############################################################################
def handle_ping(msg,obj):
    # differentiate between request + response
    pay = msg.get_payload()
    target_sys = pay[12]
    target_comp = pay[13]
        
    if target_sys == 0 and target_comp == 0:
        # ping request --> send ping response
        pingSeq = hex(pay[8])+hex(pay[9])[2:]+hex(pay[10])[2:]+hex(pay[11])[2:]
        pSeq = int(pingSeq,16)
        #print 'mavlink_parser.py::MAVLINK_MSG_ID_PING | Sequence Number: ' + str(pSeq)
        now = datetime.datetime.now()
        usec = time.mktime(now.timetuple())*1000000
        
        src = msg.get_header().srcSystem
        comp = msg.get_header().srcComponent
        
        pingMsg = obj.ping_encode(time_usec=usec, seq=pSeq, target_system=src, target_component=comp)
        packedPing = pingMsg.pack(obj)
        #print packedPing.encode("hex")
        print 'ping request received'
        return mavlink0.MAVLINK_MSG_ID_PING, packedPing, '', msg.get_payload()
    else:
        # ping response --> processing not necessary
        print 'ping response received'
        return mavlink0.MAVLINK_MSG_ID_PING, msg, '', msg.get_payload()

###############################################################################
## handle the reception of a paramete value request
###############################################################################
def handle_param_request_read(msg,obj):
    # read out systemID and componentID and return values
    sysID = msg.get_payload()[2]
    compID = msg.get_payload()[3]
    
    global DEBUG, qc
    
    if DEBUG == 0:
        
        return mavlink0.MAVLINK_MSG_ID_PARAM_REQUEST_READ, sysID, compID, msg.get_payload()
    
    else:
        
        qc.generate(mavlink0.MAVLINK_MSG_ID_PARAM_REQUEST_READ, sysID, compID, msg.get_payload(),obj)

###############################################################################
## handle the reception of a scaled IMU packet
###############################################################################
def handle_scaled_imu2(msg,obj):
    # read out systemID and componentID and return values
    sysID = msg.get_payload()[2]
    compID = msg.get_payload()[3]
    
    return mavlink0.MAVLINK_MSG_ID_SCALED_IMU2, sysID, compID, msg.get_payload()

###############################################################################
## handle the reception of a raw IMU packet
###############################################################################
def handle_raw_imu(msg,obj):
    # read out systemID and componentID and return values
    sysID = msg.get_payload()[2]
    compID = msg.get_payload()[3]
        
    return mavlink0.MAVLINK_MSG_ID_RAW_IMU, sysID, compID, msg.get_payload()
    
###############################################################################
## handle the reception of GPS packet
###############################################################################
def handle_global_position_int(msg,obj):
    # read out systemID and componentID and return values
    sysID = msg.get_payload()[2]
    compID = msg.get_payload()[3]

    return mavlink0.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, sysID, compID, msg.get_payload()

###############################################################################
## handle the reception of a dat stream request
###############################################################################
def handle_request_data_stream(msg,obj):
    global DEBUG, qc
    
    # read out systemID and componentID and return values  
    sysID = msg.get_payload()[2]
    compID = msg.get_payload()[3]
    
    
    if DEBUG == 0:
        
        return mavlink0.MAVLINK_MSG_ID_REQUEST_DATA_STREAM, sysID, compID, msg.get_payload()
    
    else:
        
        qc.generate(mavlink0.MAVLINK_MSG_ID_REQUEST_DATA_STREAM, sysID, compID, msg.get_payload(),obj)
        
    
###############################################################################
## handle the reception system time packet
###############################################################################
def handle_system_time(msg,obj):
    # read out systemID and componentID and return values
    sysID = msg.get_payload()[2]
    compID = msg.get_payload()[3]
    
    global DEBUG, qc
    
    if DEBUG == 0:    
        return mavlink0.MAVLINK_MSG_ID_SYSTEM_TIME, sysID, compID, msg.get_payload()
    else:
        qc.generate( mavlink0.MAVLINK_MSG_ID_SYSTEM_TIME, sysID, compID, msg.get_payload(), obj)

###############################################################################
## handle the reception of a long command packet
###############################################################################
def handle_long_command(msg, obj):

    # read out systemID and componentID and return values
    sysID = msg.get_payload()[30]
    compID = msg.get_payload()[31]

    global DEBUG, qc
    
    print "arm **********************************************************"
    print msg.get_payload()
    
    ## if system s not in debug mode return message data to the user
    if DEBUG == 0:
        return mavlink0.MAVLINK_MSG_ID_COMMAND_LONG, sysID, compID, msg.get_paload()
    else:
        qc.generate(mavlink0.MAVLINK_MSG_ID_COMMAND_LONG, sysID, compID, msg.get_payload(), obj)

###############################################################################
## handles the overwriting of rc channel values
###############################################################################
def handle_channels_overwrite(msg, obj):
    # read out systemID and componentID and return values
    sysID = msg.get_payload()[16]
    compID = msg.get_payload()[17]
    
    global DEBUG, qc
    
    if DEBUG == 0:
        return mavlink0.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, sysID, compID, msg.get_paload()
    else:
        qc.generate(mavlink0.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, sysID, compID, msg.get_payload(), obj)

###############################################################################
## handles setting of mission item count
###############################################################################
def handle_mission_item_count(msg, obj): 
    ## read out systemID and componentID and return values
    sysID = msg.get_payload()[2]
    compID = msg.get_payload()[3]
    
    global DEBUG, qc
    
    if DEBUG == 0:
        return mavlink0.MAVLINK_MSG_ID_MISSION_COUNT, sysID, compID, msg.get_paload()
    else:
        qc.generate(mavlink0.MAVLINK_MSG_ID_MISSION_COUNT, sysID, compID, msg.get_payload(), obj)
       
###############################################################################
## handles receiving of mission items
###############################################################################
def handle_mission_item_send(msg, obj): 
    # read out systemID and componentID and return values
    sysID = msg.get_payload()[32]
    compID = msg.get_payload()[33]
    
    global DEBUG, qc
    
    if DEBUG == 0:
        return mavlink0.MAVLINK_MSG_ID_MISSION_ITEM, sysID, compID, msg.get_paload()
    else:
        qc.generate(mavlink0.MAVLINK_MSG_ID_MISSION_ITEM, sysID, compID, msg.get_payload(), obj)    

def handle_mode_set(msg, obj): 
    
    ## read out systemID and componentID and return values
    sysID = msg.get_payload()[4]
    compID = msg.get_payload()[5]
    
    if DEBUG == 1:
        qc.generate(mavlink0.MAVLINK_MSG_ID_SET_MODE, sysID, compID, msg.get_payload(), obj)       