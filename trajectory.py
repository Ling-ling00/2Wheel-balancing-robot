def generate_trajectory_ramp(t, v_max, omega_max, ramp_time):

    # Ramp logic
    if t <= ramp_time:
        x_velocity = (v_max / ramp_time) * t  # Linear ramp for x_velocity
        yaw_velocity = (omega_max / ramp_time) * t  # Linear ramp for yaw_velocity
    else:
        x_velocity = v_max  # Maintain maximum velocity
        yaw_velocity = omega_max  # Maintain maximum angular velocity
    
    return x_velocity, yaw_velocity