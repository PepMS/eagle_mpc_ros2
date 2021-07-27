from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
    Node(
      package="eagle_mpc_2_control",
      executable="mpc_runner",
      name="MpcRunner",
      output="screen",
      emulate_tty=True,
      parameters=[
        {"trajectory_config_path": "/home/pepms/wsros/mpc-ws/src/eagle_mpc_ros/eagle_mpc_yaml/trajectories/iris_px4_displacement.yaml"},
        {"trajectory_dt": 20},
        {"trajectory_solver": "SolverSbFDDP"},
        {"trajectory_integration": "IntegratedActionModelEuler"},
        {"mpc_config_path": "/home/pepms/wsros/mpc-ws/src/eagle_mpc_ros/eagle_mpc_yaml/mpc/iris_px4_mpc.yaml"},
        {"mpc_type": "Carrot"}
        ]
    )
  ])