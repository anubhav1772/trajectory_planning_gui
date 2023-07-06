# This is a message to hold data to jog by specifying joint
# displacement. You only need to set relative displacement to joint
# angles (or displacements for linear joints).

# Header message. You must set frame_id to define the reference
# coordinate system of the displacament
Header header

# Name list of the joints. You don't need to specify all joint of the
# robot. Joint names are case-sensitive.
string[] joint_names

# The positions we want each joint to move. The order must be
# identical to joint_names. Unit is in radian for revolutive and continuous joints.
float64[] positions
float64[] velocities
float64[] accelerations
