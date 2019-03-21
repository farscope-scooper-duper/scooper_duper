from geometry_msgs.msg import Pose,PoseStamped
import tf
import math
import rospy

def get_waypoint_pose(waypoint_id):
    move_plane_y = -0.542
    bin_centre = 0
    bin_width = 0.25
    #arm_base x axis goes from right to left 
    bin_left = bin_centre + bin_width
    bin_right = bin_centre - bin_width
    #rospy.init_node('move_group_python_interface_tutorial',
    #                   anonymous=True)
    shelf1_z = 0.750
    shelf2_z = shelf1_z - 0.32
    shelf3_z = shelf2_z - 0.32
    shelf4_z = shelf3_z - 0.32

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
    tote_pose.position.x = bin_centre
    tote_pose.position.y = move_plane_y
    tote_pose.position.z = 0.203
    q = tf.transformations.quaternion_from_euler(
       float(math.pi/2),float(-math.pi/2),float(0))
    #tote_pose.orientation.x = 0#q[0];
    #tote_pose.orientation.y = 0#q[1];
    #tote_pose.orientation.z = 0.688#q[2];
    #tote_pose.orientation.w = 0.72#q[3];
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
        #bin_waypoint.orientation = tf.transformations.quaternion_from_euler(
         #          float(0),float(0),float(0))
        bin_waypoint.orientation.x = q[0];
        bin_waypoint.orientation.y = q[1];
        bin_waypoint.orientation.z = q[2];
        bin_waypoint.orientation.w = q[3];
        pose = bin_waypoint
    
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "/base"
    pose_stamped.pose = pose
    return pose_stamped
