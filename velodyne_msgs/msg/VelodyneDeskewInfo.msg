# Velodyne Deskew Info

# The "seq" and "stamp" fields of the header of this message matches the 
# same fields of the header in the corresponding "VelodyneScan" message
Header           header         # standard ROS message header

# Velodyne LIDAR reports 3D points by sweeping laser beams radially from 
# 0 to 360 degrees azimuth angle. The "sweep_info" field reports an array of 
# "VelodyneSweepInfo" messages, each containing a pair of ROS timestamp and 
# angle (in degrees). The angle field "start_angle" indicates the starting 
# azimuth sweep angle of a particular section of pointcloud and the timestamp 
# field "stamp" indicates the time when the points in that section of the sweep 
# were captured.
# The ending angle for a sweep section should be inferred by the "start_angle" 
# of the next sweep section which is in the next element of the array.
# The first sweep section always starts at 0 degrees and the ending angle for 
# the last sweep section should always be assumed by the subscriber to be 360 
# degrees.

# Angle 0 corresponds to the positive Y direction of Velodyne's local 
# co-ordinate frame. Refer the Velodyne user manual for coordinate system 
# definition.

VelodyneSweepInfo[] sweep_info  # vector of sweep angle and timing info
