#!/usr/bin/env python

RUN_TIME_LIMIT = 600 #Seconds
OPERATION_TIME_LIMIT = 15 #Seconds

NUMBER_OF_VIEWPOINTS1 = 13


NUMBER_OF_VIEWPOINTS2 = 7

SHUFFLE_COUNTER_MAX = 3
DIP_COUNTER_MAX = 3
DIP_TIME_LIMIT = 15 #Seconds

STARE_TIME = 1.5 #Seconds

#0.1 is speedy probably don't want to go much higher on the real arm
#0.03 is a slowish but better for giving you more time to press the E stop
SPEED_SCALE = 0.1

VACUUM_OFF_THRESH = 1005 #Above this and we assume no suction
OBJECT_CONTACT_THRESH = 876 #Below this is a blockage - assume we hold an item

PRINT_DELAY = 1 #Second

DIP_UP_SPEED = 0.02
DIP_DOWN_SPEED = 0.03

DOPESCOPE_OFFSET = 0.04
