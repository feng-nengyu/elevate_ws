from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit").to_moveit_configs()
    # Preserve narrow press/retract corridors during trajectory retiming.
    moveit_config.planning_pipelines['ompl'].update({
        'path_tolerance': 0.001,
        'resample_dt': 0.02,
    })
    return generate_move_group_launch(moveit_config)
