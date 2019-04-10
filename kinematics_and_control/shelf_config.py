#!/usr/bin/env python

"""

         ^ shelf_z
Shelf_x  |
      <--
                   Shelf width
        <------------------------------->
        _________________________________
        |  |  /|\                    |  | /|\  /|\ 
        |  |   | shelf               |  |  |    | 
        |__|___|seperation __________|__|  |shelf1_z
        |  |   |    |       |        |  |  |    | 
        |  |   |    |       |        |  | \|/   |
        |  |   |    |       |        |  |       | 
        |  |   |    bin width        |  |       |
        |  |   |    |<----->|        |  |       |
        |  |   |    |       |        |  |       |
        |__|___|____|_______|________|__|       | 
        |  |   |                     |  |       | 
        |  |   |                     |  |       | 
        |__|__\|/____________________|__|       |
        |  |        |       |        |  |       | 
        |  |        |       |        |  |       |
        |  |        |       |        |  |       | side height 
        |  |        |       |        |  |       |
        |  |        |       |        |  |       |
        |  |        |       |        |  |       |
        |__|________|_______|________|__|       | 
        |  |   /'\                   |  |       | 
        |  |    |  shelf height      |  |       | 
        |__|___\./___________________|__|       |
        |  |        |       |        |  |       | 
        |  |        |       |        |  |       |
        |  |        |       |        |  |       | 
        |  |        |       |        |  |       |
        |  |        |       |        |  |       |
        |  |        |       |        |  |       |
        |__|________|_______|________|__|       | 
        |  |                         |  |       | 
        |  |                         |  |       | 
        |__|_________________________|__|       | 
        | bin_left  |       |        |  |       |
        |<----->    |       |        |  |       | 
        |  |bin _center     |        |  |       |
        |<-------------->   |        |  |       |
        |  |    bin_right   |        |  |       |
        |<---------------------->    |  |       |
        |__|________|_______|________|__|       | 
        |  |                         |  |       | 
        |  |                         |  |       | 
        |  |                         |  |       | 
        |  |                         |  |       | 
        |  |                         |  |      \|/
        <-->
    side_width


       





"""

#-----------------------Shelving config------------------------------
#Position of the shelving in the base frame of the UR10
shelving_position = (0.448,-1.11,1.75-0.77)

#Depth of shelving (in negative y of the shelf frame)
shelf_depth = 0.590
#width of the shelving, the distance in negative x to go from shelving frame to the right hand side of the shelving
shelf_width = 0.89

#The height of the metal seperators between each shelf
shelf_height = 0.075

#Distance between each shelf (from the wood to the next piece of wood)
shelf_separation = 0.32  

#width and height of the left and right pillars of shelving
side_width = 0.04
side_height = 1.75


