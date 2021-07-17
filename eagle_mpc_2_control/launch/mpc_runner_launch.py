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
        {"trajectory_config_path": "/home/pepms/wsros/mpc-ws/src/eagle_mpc_ros/eagle_mpc_yaml/trajectories/iris_hover.yaml"},
        {"trajectory_dt": 10},
        {"trajectory_solver": "SolverSbFDDP"},
        {"trajectory_integration": "IntegratedActionModelEuler"},
        {"mpc_config_path": "/home/pepms/wsros/mpc-ws/src/eagle_mpc_ros/eagle_mpc_yaml/mpc/iris_mpc.yaml"},
        {"mpc_type": "Carrot"}
        ]
    )
  ])