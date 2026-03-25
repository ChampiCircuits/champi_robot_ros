import os

def generate_cpp_code(pami_id, config_data):
    trajectories = config_data.get("trajectories", {})
    speed = config_data.get("globalSpeed", 10.0)
    start_after_delay_s = config_data.get("startAfterDelayS", 10.0)

    pami_id_str = str(pami_id)
    waypoints = trajectories.get(pami_id_str, [])

    num_points = len(waypoints)

    cpp_code = f"""#ifndef GENERATED_TRAJECTORY_H
#define GENERATED_TRAJECTORY_H

// Auto-generated configuration for PAMI {pami_id}
const float GLOBAL_SPEED_MM_S = {speed * 10.0};
const float START_AFTER_DELAY_S = {start_after_delay_s};
const int TRAJECTORY_POINTS_COUNT = {num_points};

struct Waypoint {{
    float x;
    float y;
}};

"""
    if num_points > 0:
        cpp_code += "const Waypoint EXPERIMENT_TRAJECTORY[] = {\n"
        for wp in waypoints:
            cpp_code += f"    {{{float(wp['x']):.2f}, {float(wp['y']):.2f}}},\n"
        cpp_code += "};\n"
    else:
        cpp_code += "const Waypoint EXPERIMENT_TRAJECTORY[1] = {{0, 0}};\n"

    cpp_code += "\n#endif // GENERATED_TRAJECTORY_H\n"

    # Save to firmware/include/generated_trajectory.h
    firmware_include_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'firmware', 'include')
    os.makedirs(firmware_include_dir, exist_ok=True)
    header_path = os.path.join(firmware_include_dir, 'generated_trajectory.h')
    
    with open(header_path, "w") as f:
        f.write(cpp_code)
    
    return header_path

