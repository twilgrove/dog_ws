std_msgs/Header header

# --- [1] 基座姿态 (Base Pose - World Frame) ---
# x: yaw, y: pitch, z: roll (单位: rad)
geometry_msgs/Vector3 orientation_zyx
# 基座在世界系下的位置 (单位: m)
geometry_msgs/Point position

# --- [2] 基座运动学 (Base Kinematics) ---
# 基座相对于自身的角速度 (Body Frame, 单位: rad/s)
geometry_msgs/Vector3 base_angular_velocity
# 基座在世界系下的线速度 (World Frame, 单位: m/s)
geometry_msgs/Vector3 world_linear_velocity
# 基座相对于自身的线加速度 (Body Frame/IMU, 单位: m/s^2)
geometry_msgs/Vector3 base_linear_acceleration

# --- [3] 关节位置 (Joint Positions - 12 DoF) ---
# 顺序: [Hip, Thigh, Calf]
float64[3] q_lf
float64[3] q_lh
float64[3] q_rf
float64[3] q_rh

# --- [4] 关节速度 (Joint Velocities - 12 DoF) ---
# 顺序: [Hip, Thigh, Calf]
float64[3] dq_lf
float64[3] dq_lh
float64[3] dq_rf
float64[3] dq_rh

# --- [5] 触地状态 (Contact States) ---
# 顺序: [Left Front, Left Hind, Right Front, Right Hind]
bool[4] contact_state