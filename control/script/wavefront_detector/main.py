#!/usr/bin/env python3

import rospy
from wavefront_detector import WaveFrontDetector


def main() -> None:
    """
    Main function to execute the wavefront detection algorithm.

    Returns:
        None
    """

    # Set the starting pose
    starting_pose = [0.0, 0.0]
    # Set initial waypoints
    waypoints = [(0.0, 0.0), (0.1, 0.1), (0.2, 0.2)]

    # Initialize the WaveFrontDetector
    agent = WaveFrontDetector()
    agent.set_waypoints(waypoints)

    # Set the initial pose for the agent
    agent.info_msg('Setting initial pose')
    agent.set_initial_pose(starting_pose)
    agent.info_msg('Waiting for pose to be received')

    # Move the agent to frontiers based on the wavefront detection algorithm
    agent.move_to_frontiers()

    # Spin to keep the program running
    rospy.spin()


if __name__ == '__main__':
    main()
