#testing:
    # rudder+winch servo control
    # Rudder sweep
    #ros2 topic pub /actuators/rudder/angle_deg std_msgs/Float32 '{data: 30.0}'
    #  Winch slow out (+) and in (-)
    #ros2 topic pub /actuators/winch/pct std_msgs/Float32 '{data: 40.0}'