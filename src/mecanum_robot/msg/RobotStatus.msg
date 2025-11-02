# Message pour le statut du robot
Header header
string robot_id
string status              # "idle", "moving", "scanning", "error"
geometry_msgs/Pose2D pose  # Position actuelle du robot
float64 battery_voltage
float64 cpu_temperature
bool lidar_active
bool microcontroller_connected
string[] active_sensors
string error_message