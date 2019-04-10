from geometry_msgs.msg import Pose,PoseStamped
import tf
import math
import rospy
import shelf_config
def get_waypoint_pose(waypoint_id):

    #-----------------------Bin frame config------------------------------

    bin_width = 0.25 #(shelf_width-side_width*2)/3

    #distance from the shelf frame origin the centre of the middle bin (in x, postive x goes left, so centre is in negative x)
    bin_centre = -shelf_config.shelf_width/2
    bin_left = bin_centre + bin_width
    bin_right = bin_centre - bin_width

    shelf1_z = -(shelf_config.shelf_height+0.01)
    shelf2_z = shelf1_z - shelf_config.shelf_separation
    shelf3_z = shelf2_z - shelf_config.shelf_separation
    shelf4_z = shelf3_z - shelf_config.shelf_separation

    #distance from the shelves to move between bins (in positive y of shelf frame)
    move_plane_y = 0.71#-1.11+0.400

    #tote location
    tote_x = bin_centre
    tote_y = move_plane_y
    tote_z = 0.203

    #----------------------------end of bin frame config---------------------------------
    bin_waypoints = {
        "bin_A": (bin_left,move_plane_y,shelf1_z),
        "bin_B": (bin_centre,move_plane_y,shelf1_z),
        "bin_C": (bin_right,move_plane_y,shelf1_z),
        "bin_D": (bin_left,move_plane_y,shelf2_z),
        "bin_E": (bin_centre,move_plane_y,shelf2_z),
        "bin_F": (bin_right,move_plane_y,shelf2_z),
        "bin_G": (bin_left,move_plane_y,shelf3_z),
        "bin_H": (bin_centre,move_plane_y,shelf3_z),
        "bin_I": (bin_right,move_plane_y,shelf3_z),
        "bin_J": (bin_left,move_plane_y,shelf4_z),
        "bin_K": (bin_centre,move_plane_y,shelf4_z),
        "bin_L": (bin_right,move_plane_y,shelf4_z),
    }

    tote_pose = Pose()
    tote_pose.position.x = tote_x
    tote_pose.position.y = tote_y
    tote_pose.position.z = tote_z

    q = tf.transformations.quaternion_from_euler(float(math.pi),float(math.pi/2),float(math.pi/2))
    tote_pose.orientation.x = q[0];
    tote_pose.orientation.y = q[1];
    tote_pose.orientation.z = q[2];
    tote_pose.orientation.w = q[3];
    if waypoint_id == "tote":
        pose = tote_pose
    else: #bin pose
        bin_waypoint = Pose()
        binpos = bin_waypoints[waypoint_id]
        bin_waypoint.position.x = binpos[0]
        bin_waypoint.position.y = binpos[1]
        bin_waypoint.position.z = binpos[2]
        bin_waypoint.orientation.x = q[0];
        bin_waypoint.orientation.y = q[1];
        bin_waypoint.orientation.z = q[2];
        bin_waypoint.orientation.w = q[3];
        pose = bin_waypoint
    
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "/shelves"
    pose_stamped.pose = pose
    return pose_stamped

