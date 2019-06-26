# Velodyne LIDAR Sweep Angle and Timestamp info.

# Velodyne LIDAR reports 3D points by sweeping laser beams radially from 
# 0 to 360 degrees azimuth angle. The angle field "start_angle" indicates the starting 
# azimuth sweep angle of a particular section of pointcloud and the timestamp field 
# "stamp" indicates the time when the points in that section of the sweep were 
# captured.

# Angle 0 corresponds to the positive Y direction of Velodyne's local 
# co-ordinate frame. Refer the Velodyne user manual for coordinate system 
# definition.

time stamp              # packet timestamp
float32 start_angle     # start of the sweep
