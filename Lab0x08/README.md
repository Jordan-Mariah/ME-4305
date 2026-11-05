# Simple startup

1) Connect to Bingus (the ROMI) via Putty and reload running code (ctrl+c, ctrl+d)
2) Set operating speed by pressing 'V' and entering speed in rad/s. 4 is recommended for simple tests
3) Place Bingus on the line, or on a path that intersects the line, and press 'W'.
4) Use the 'A' and 'D' keys to switch lanes left and right respectively. 'S' can be used stop Bingus.

# Drive behavior interface for development

The drive behavior of Bingus is controlled by setting the value of various behavior Boolean flags. The most relevant flags for Lab0x07 and prior behavior are:
- line_follow_flag: bool (set to True to engage line follow behavior)
- lane_change_flag: bool (set to True to engage lane change behavior)
- lane_change_dir: Literal["left", "right"] (set to dictate the direction of lane change)
- next_effort: float (represents target speed in rad/s)

All of the above objects are of type Share. Assign <flag_name>.value or use <flag_name>.set() to change the actual shared value.

Sensor data can be read through the sensor_data Share, whose .value is a dictionary containing all useful sensor data. The keys for sensor_data.value are:
- "line": float (Position of line centroid in cm. A value of 0 indicates Bingus is perfectly on the line)
- "all_black": bool (Flag indicating whether the line sensor detected an all black line. You will need to manually reset to False in order to detect subsequent line crossings.)
- "imu": Tuple() (Tuple containing: heading in degrees, yaw rate in rad/s, accumulated time of IMU)