/*
  @file

  @author adriaroig

  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
*/
#include <ros/ros.h>
#include <pal_wbc_utils/pal_wbc_utils.h>
#include <pal_ros_utils/reference/pose/pose_pointer_reference.h>
#include <wbc_tasks/go_to_kinematic_task.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
 *  Node that mantains the torso at a specific height
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "push_pose_task");
  ros::NodeHandle nh;

  // Name of the link to be constrained
  std::string link_name;
  nh.param<std::string>("link_name", link_name, "gripper_left_base_link");

  // Name of the base frame
  std::string base_frame;
  nh.param<std::string>("base_frame", base_frame, "odom");

  // Reference height (talos squating)
  double reference_height;
  nh.param<double>("reference_height", reference_height, -0.1);

  // Task which respect the pose task is pushed
  std::string previous_task_id;
  nh.param<std::string>("respect_task_id", previous_task_id, "default_reference");

  std::string reference_type;
  nh.param<std::string>("source_data", reference_type, "topic");

  // Namespace of the controller
  std::string ns;
  // Uncomment next line if Kinematic WBC is launched
  nh.param<std::string>("ns", ns, "/whole_body_kinematic_controller");
  // Uncomment next line if Dynamic WBC is launched
  // nh.param<std::string>("ns", ns, "/biped_walking_dcm_controller/wbc");

  std::string topic_name;
  nh.param<std::string>("topic_name", topic_name, ns+"/gripper_left");


  // Creates a service helper that comunicates with all the WBC services
  pal::WBCServiceHelper srv_helper(nh, ns);

  // Order respect the previous task id
  pal_wbc_msgs::Order::_order_type order = pal_wbc_msgs::Order::Before;

  // Set all he properties in a property bag to push the task online
  property_bag::PropertyBag properties;
  std::string task_id = link_name + "_reference_pose";
  properties.addProperty("taskType", std::string("pal_wbc/GoToPoseMetaTask"));
  properties.addProperty("task_id", task_id);
  properties.addProperty("tip_name", link_name);
  properties.addProperty("damping", 1.0);
  properties.addProperty("signal_reference", reference_type);
  properties.addProperty("topic_name", topic_name);


  /*
    Forces the link name frame at a certain height respect the base frame.
    Since the torso frame has the Z coordinate pointing up, constraint the Z coordinate
    at a specific position is equivalent to force it to be a certain height.
  */

  // Conrolled coordinates (by default the task controls all the three coordinates)
//   pal_wbc::coord_t c = pal_wbc::coord_t::Z;
//   std::vector<int> coordinates = { c._to_integral() };
//   properties.addProperty("coordinates", coordinates);
  
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.stamp = ros::Time::now();
  target_pose.header.frame_id = base_frame;
  target_pose.pose.position.x = 0.0;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = reference_height;

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.5, 0.0);
  geometry_msgs::Quaternion quat_msg;
  quat_msg = tf2::toMsg(quat_tf);

  target_pose.pose.orientation = quat_msg;
  
//   properties.addProperty("target_pose", target_pose);
  properties.addProperty("p_pos_gain", 10.0);
  properties.addProperty("p_orient_gain", 10.0);

  if(!srv_helper.pushTask(properties, "task_id", order, previous_task_id))
  {
    ROS_ERROR_STREAM("There was a problem pushing torso heigh reference task");
  }

  return (0);
}

