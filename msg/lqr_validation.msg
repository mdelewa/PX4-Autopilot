uint64 timestamp		# time since system start (microseconds)

float32 u		# body linear velocity in FRD frame (can be NaN for FW)
float32 v		# body linear velocity in FRD frame (can be NaN for FW)
float32 w		# body linear velocity in FRD frame (can be NaN for FW)

float32 phi		# body angle in NED frame (can be NaN for FW)
float32 theta		# body angle in NED frame (can be NaN for FW)
float32 psi		# body angle in NED frame (can be NaN for FW)

float32 p		# body angular velocity in FRD frame (can be NaN for FW)
float32 q		# body angular velocity in FRD frame (can be NaN for FW)
float32 r		# body angular velocity in FRD frame (can be NaN for FW)

float32 phi_sp		# body angle setpoint in NED frame (can be NaN for FW)
float32 theta_sp	# body angle setpoint in NED frame (can be NaN for FW)

float32 roll_u		# Commanded aileron
float32 pitch_u		# Commanded elevator
float32 yaw_u		# Commanded rudder
