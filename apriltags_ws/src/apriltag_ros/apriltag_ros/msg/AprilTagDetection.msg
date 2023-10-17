# Tag ID(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the IDs of each tag in the bundle.
int32[] id

# Tag size(s). If a standalone tag, this is a vector of size 1. If a tag bundle,
# this is a vector containing the sizes of each tag in the bundle, in the same
# order as the IDs above.
float64[] size

# Pose in the camera frame, obtained from homography transform. If a standalone
# tag, the homography is from the four tag corners. If a tag bundle, the
# homography is from at least the four corners of one member tag and at most the
# four corners of all member tags.
geometry_msgs/PoseWithCovarianceStamped pose
geometry_msgs/Point corner1
geometry_msgs/Point corner2
geometry_msgs/Point corner3
geometry_msgs/Point corner4
geometry_msgs/Point centerpx
geometry_msgs/Point coordinate1
geometry_msgs/Point coordinate2
geometry_msgs/Point coordinate3
geometry_msgs/Point coordinate4
geometry_msgs/Point coordinate_center
geometry_msgs/Quaternion orientation
std_msgs/Bool detected