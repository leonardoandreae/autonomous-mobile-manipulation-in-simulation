# A detected object with its 6-DoF pose in the world frame
std_msgs/Header header

string   name             # Object class label
geometry_msgs/PoseStamped pose
float32  confidence       # Detection confidence [0, 1]
