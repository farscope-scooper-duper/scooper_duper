#!/usr/bin/env python

RUN_TIME_LIMIT = 600 #Seconds
OPERATION_TIME_LIMIT = 15 #Seconds

NUMBER_OF_VIEWPOINTS = 13
LAST_VIEWPOINT = 12

SHUFFLE_COUNTER_MAX = 3
DIP_COUNTER_MAX = 3
DIP_TIME_LIMIT = 15 #Seconds

VISION_LOOK_TIME = 0#Seconds
STARE_TIME = 1 #Second

#0.1 is speedy probably don't want to go much higher on the real arm
#0.03 is a slowish but better for giving you more time to press the E stop
SPEED_SCALE = 0.05

VACUUM_OFF_THRESH = 1010 #Above this and we assume no suction
OBJECT_CONTACT_THRESH = 875 #Below this is a blockage - assume we hold an item
