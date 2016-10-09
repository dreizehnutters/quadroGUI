###############################################################################
###############################################################################
## SerialListen
##
## This file implements a Qt thread object to listen on incoming packages
## at the serial port without blocking the normal work of the GUI
##
## author: Nicole Todtenberg, Thomas Basmer
##
## date: 2014-08-05
##
## version: 0.00
##
## changes:
##    - 0.00 2014-08-05
##        - initial version
###############################################################################
###############################################################################

## import the QtCore class from the PyQt4 module. It provides QtThreads, signals
## and slots. Signals and slots are needed to exchange data between the GUI Thread
## and the listener thread
from PyQt4 import QtCore

## import sys class from os module. it is used to print system information when
## an exception is thrown
from os import sys

################################################################################
################################################################################
## Listening to serial port thread
################################################################################    
################################################################################
class SerialListen(QtCore.QThread):

    ################################################################################
    ## constructor of the thread class
    ##
    ## parameter:
    ## - parent:     parent object instantiating this thread
    ## - comPort:    serial object where the thread should listen to
    ## - mavLinkObj: object of the Mavlink library to provide MAvlink related methods
    ################################################################################   
    def __init__(self,parent,comPort, mavLinkObj):
        """
        @param parent:  todo
        @param frame:   todo  
        """
        ## init a Qt Thread
        QtCore.QThread.__init__(self)
        
        ## assign the given parameters to internal variables of the class
        self._parent = parent
        self._comPort = comPort
        self._mavLinkObj = mavLinkObj
        
        ## create a Qt signal. It is needed to send data from the listener
        ## thread and the GUI thread
        self.new_message = QtCore.SIGNAL("new_message")

    ###########################################################################
    ## run method
    ##
    ## implements the behavior of the listener thread object while run
    ###########################################################################
    def run(self):
       
        ## run to infinity or until the thread is terminated
        while 1:

            if self._comPort.isOpen() == True:
            
                ## if there is incoming data on the serial port read the first byte.
                ## it is the Start-of-Frame-Delimiter (SFD) of the packet
                sfd = self._comPort.read(1)
                
                ## read the next byte. it represents the payload length of the packet
                length = self._comPort.read(1)
                
                ## if packet length was read and the SFD has the proper value of 0xFE
                if length != False and length != '' and sfd == "\xFE":
                    
                    ## convert the received packet length into a intger value
                    lField = ord(length)
                    
                    ## reading the payload of the packet and the 6 resuming bytes of the
                    ## header and the CRC checksum
                    rxFrame = self._comPort.read(lField + 6)
                    
                    ## concatenate all data to form a proper Mavlink packet
                    rxMessage = sfd + length + rxFrame
                    
                    ## send the received packet to the main GUI thread via the new_message signal
                    self.emit(self.new_message, rxMessage)
                    
    ## close the thread
    def close(self):
        sys.exit()