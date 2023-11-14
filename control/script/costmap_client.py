#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
import map_msgs.msg
import tf
import costmap_2d


class Costmap2DClient:
    def __init__(self, param_nh, subscription_nh, tf):
        self.tf = tf
        self.global_frame = ""
        self.robot_base_frame = ""
        self.transform_tolerance = 0.3

        param_nh.param("robot_base_frame", self.robot_base_frame, "base_link")
        param_nh.param("transform_tolerance", self.transform_tolerance, 0.3)

        self.costmap_sub = subscription_nh.subscribe("costmap", 1000, self.update_full_map)
        rospy.loginfo("Waiting for costmap to become available, topic: costmap")
        costmap_msg = rospy.wait_for_message("costmap", OccupancyGrid)
        self.update_full_map(costmap_msg)

        self.costmap_updates_sub = subscription_nh.subscribe("costmap_updates", 1000, self.update_partial_map)

        tf_prefix = tf.getPrefixParam(param_nh)
        self.robot_base_frame = tf.resolve(tf_prefix, self.robot_base_frame)

        last_error = rospy.Time.now()
        tf_error = ""
        while rospy.ok() and not self.tf.waitForTransform(self.global_frame, self.robot_base_frame,
                                                           rospy.Time(), rospy.Duration(0.1),
                                                           rospy.Duration(0.01), tf_error):
            rospy.spinOnce()
            if last_error + rospy.Duration(5.0) < rospy.Time.now():
                rospy.logwarn("Timed out waiting for transform from %s to %s to become available "
                              "before subscribing to costmap, tf error: %s",
                              self.robot_base_frame, self.global_frame, tf_error)
                last_error = rospy.Time.now()
            tf_error = ""

    def update_full_map(self, msg):
        self.global_frame = msg.header.frame_id

        size_in_cells_x = msg.info.width
        size_in_cells_y = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        rospy.logdebug("Received full new map, resizing to: %d, %d", size_in_cells_x, size_in_cells_y)

        costmap_2d.resizeMap(size_in_cells_x, size_in_cells_y, resolution, origin_x, origin_y)

        mutex = costmap_2d.getMutex()
        with mutex:
            costmap_data = costmap_2d.getCharMap()
            costmap_size = costmap_2d.getSizeInCellsX() * costmap_2d.getSizeInCellsY()
            rospy.logdebug("Full map update, %d values", costmap_size)
            for i in range(min(costmap_size, len(msg.data))):
                cell_cost = msg.data[i]
                costmap_data[i] = init_translation_table()[cell_cost]
            rospy.logdebug("Map updated, written %d values", costmap_size)

    def update_partial_map(self, msg):
        rospy.logdebug("Received partial map update")
        self.global_frame = msg.header.frame_id

        if msg.x < 0 or msg.y < 0:
            rospy.logerr("Negative coordinates, invalid update. x: %d, y: %d", msg.x, msg.y)
            return

        x0 = msg.x
        y0 = msg.y
        xn = msg.width + x0
        yn = msg.height + y0

        mutex = costmap_2d.getMutex()
        with mutex:
            costmap_xn = costmap_2d.getSizeInCellsX()
            costmap_yn = costmap_2d.getSizeInCellsY()

            if xn > costmap_xn or x0 > costmap_xn or yn > costmap_yn or y0 > costmap_yn:
                rospy.logwarn("Received update doesn't fully fit into existing map, "
                              "only part will be copied. received: [%d, %d], [%d, %d] "
                              "map is: [0, %d], [0, %d]",
                              x0, xn, y0, yn, costmap_xn, costmap_yn)

            costmap_data = costmap_2d.getCharMap()
            i = 0
            for y in range(y0, min(yn, costmap_yn)):
                for x in range(x0, min(xn, costmap_xn)):
                    idx = costmap_2d.getIndex(x, y)
                    cell_cost = msg.data[i]
                    costmap_data[idx] = init_translation_table()[cell_cost]
                    i += 1

    def get_robot_pose(self):
        global_pose = tf.StampedPose()
        global_pose.setIdentity()
        robot_pose = tf.StampedPose()
        robot_pose.setIdentity()
        robot_pose.frame_id = self.robot_base_frame
        robot_pose.stamp = rospy.Time()
        current_time = rospy.Time.now()

        try:
            self.tf.transformPose(self.global_frame, robot_pose, global_pose)
        except tf.LookupException as ex:
            rospy.logerr_throttle(1.0, "No Transform available Error looking up robot pose: %s", ex)
            return Pose()
        except tf.ConnectivityException as ex:
            rospy.logerr_throttle(1.0, "Connectivity Error looking up robot pose: %s", ex)
            return Pose()
        except tf.ExtrapolationException as ex:
            rospy.logerr_throttle(1.0, "Extrapolation Error looking up robot pose: %s", ex)
            return Pose()

        if current_time.to_sec() - global_pose.stamp_.to_sec() > self.transform_tolerance:
            rospy.logwarn_throttle(1.0, "Costmap2DClient transform timeout. Current time: "
                                       "%.4f, global_pose stamp: %.4f, tolerance: %.4f",
                                   current_time.to_sec(), global_pose.stamp_.to_sec(), self.transform_tolerance)
            return Pose()

        msg = PoseStamped(header=Header(stamp=global_pose.stamp_, frame_id=self.global_frame), pose=global_pose.pose)
        return msg.pose


def init_translation_table():
    cost_translation_table = [0] * 256

    for i in range(256):
        cost_translation_table[i] = int(1 + (251 * (i - 1)) / 97)

    cost_translation_table[0] = 0  # NO obstacle
    cost_translation_table[99] = 253  # INSCRIBED obstacle
    cost_translation_table[100] = 254  # LETHAL obstacle
    cost_translation_table[-1] = 255  # UNKNOWN

    return cost_translation_table


if __name__ == "__main__":
    rospy.init_node("costmap_2d_client")

    tf_listener = TransformListener()
    costmap_2d = costmap_2d.Costmap2D()
    costmap_2d_client = Costmap2DClient(rospy.get_param("~"), rospy.get_param("~"), tf_listener)

    rospy.spin()
