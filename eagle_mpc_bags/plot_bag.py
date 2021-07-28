import sqlite3

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import matplotlib.pyplot as plt
import numpy as np
import copy

from eagle_mpc.utils.plots import PlotStates

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {
            name_of: type_of
            for id_of, name_of, type_of in topics_data
        }
        self.topic_id = {
            name_of: id_of
            for id_of, name_of, type_of in topics_data
        }
        self.topic_msg_message = {
            name_of: get_message(type_of)
            for id_of, name_of, type_of in topics_data
        }

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(
                topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [(timestamp,
                 deserialize_message(data, self.topic_msg_message[topic_name]))
                for timestamp, data in rows]


if __name__ == "__main__":

    bag_file = '/home/pepms/robotics/methods-test/px4_checkings/bags/rosbag2_2021_07_13-16_01_34/rosbag2_2021_07_13-16_01_34_0.db3'

    parser = BagFileParser(bag_file)

    trajectory = parser.get_messages("/PlatformState")

    xs = []
    ts = np.zeros(len(trajectory))
    for idx, x_msg in enumerate(trajectory):
        x = np.zeros(13)
        x[0] = x_msg[1].pose.position.x
        x[1] = x_msg[1].pose.position.y
        x[2] = x_msg[1].pose.position.z

        x[3] = x_msg[1].pose.orientation.x
        x[4] = x_msg[1].pose.orientation.y
        x[5] = x_msg[1].pose.orientation.z
        x[6] = x_msg[1].pose.orientation.w

        x[7] = x_msg[1].motion.linear.x
        x[8] = x_msg[1].motion.linear.y
        x[9] = x_msg[1].motion.linear.z

        x[10] = x_msg[1].motion.angular.x
        x[11] = x_msg[1].motion.angular.y
        x[12] = x_msg[1].motion.angular.z

        xs.append(np.copy(x))
        ts[idx] = x_msg[1].header.stamp.sec + x_msg[1].header.stamp.nanosec / 1e9
    
    xs_array = np.vstack(xs).T


    PlotStates(xs_array, ts)
    plt.show()
    
