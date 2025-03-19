# Since ROS doesn't support union types for messages, this message
# can represent any type of action. The action represented by this message
# is determined by the action_type field

string FOLLOW="FOLLOW"
string GAZE="GAZE"
string PICK="PICK"
string PLACE="PLACE"

string action_type

# Definition for FOLLOW
# presumably there's some other params here
nav_msgs/Path path

# Definition for GAZE
# Should gaze at the position given by gaze_pose, with the direction given by the orientation of gaze_pose.
string gaze_frame
geometry_msgs/Point robot_point
geometry_msgs/Point gaze_point

# Definition for PICK
# Should gaze at the position given by object_pose, then pick the object there with the direction given by the orientation of object_pose.
string pick_frame
string object_class
geometry_msgs/Point object_point

# Definition for PLACE
string place_frame
# Should drop the object for now.
