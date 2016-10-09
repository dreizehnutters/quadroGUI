class MissionItem(object):
    
    def __init__(self):
        
        self.frame = 0 # coordinate system of the mission
        self.command = 0 # MAvlink Command
        self.current = 0 # is the item the current item? (0: False; 1: True)
        self.autocontinue = 0 # autocontinue to next item or not
        self.param1 = 0.0
        self.param2 = 0.0
        self.param3 = 0.0
        self.param4 = 0.0
        self.pos_x = 0 # local: x position, global: latitude (float)
        self.pos_y = 0 # y position: global: longitude (float)
        self.pos_z = 0 # z position: global: altitude (relative or absolute, depending on frame. (float)