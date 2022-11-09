#include "pr_utils/pr_wheel.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <fmt/format.h>
#include <string>

double rpm_to_rad_per_sec(const double rpm) {
  return rpm * 2*M_PI / 60.0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pitranger");
  ros::NodeHandle nh;

  // TODO(Jordan): Should these come from the launch file?
  const std::string wheel_odom_frame_id = "odom";
  const std::string wheel_odom_child_frame_id = "base_link";
  const double wheel_diam_m = 0.18;
  const double wheel_spacing_m = 0.62;

  pr::WheelController wheels;

  // Attach a subscriber to set wheel velocities.
  auto wheel_cb = [&wheels, wheel_diam_m, wheel_spacing_m]
    (const geometry_msgs::TwistConstPtr& msg) {
    try {
      double lin = msg->linear.x;
      double ang = msg->angular.z;

      double v_delta = std::tan(ang)*wheel_spacing_m;
      double l_mps = lin - v_delta/2.0;
      double r_mps = lin + v_delta/2.0;

      wheels.set_left_rpm(l_mps * 60.0 / wheel_diam_m);
      wheels.set_right_rpm(r_mps * 60.0 / wheel_diam_m);
    } catch(const std::exception& e) {
      fmt::print("WARN: pitranger node failed to set motor velocities.\n");
    }
  };
  auto wheel_vel_sub = nh.subscribe<geometry_msgs::Twist>("/pitranger/in/twist_cmd", 1, wheel_cb);

  // Create a publisher for the wheel odometry.
  auto wheel_odom_pub = nh.advertise<nav_msgs::Odometry>("/pitranger/out/wheel_odom", 100);

  // Publish wheel encoder and current data along with wheel odometry
  auto wheel_data_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/pitranger/out/wheel_data", 1);

  // Track the robot state in x,y,yaw.
  double robot_x   = 0.0;
  double robot_y   = 0.0;
  double robot_yaw = 0.0;

  ros::WallTime prev_iter_time = ros::WallTime::now();
  ros::WallTime curr_iter_time = prev_iter_time;
  
  ros::Rate rate(10);
  unsigned long iter = 0;
  while( ros::ok() ) {
    try {
      // Compute and publish wheel odometry
      const double fr_rpm = wheels.get_front_right_rpm();
      const double fl_rpm = wheels.get_front_left_rpm();
      const double rr_rpm = wheels.get_rear_right_rpm();
      const double rl_rpm = wheels.get_rear_left_rpm();

      const double  left_rpm = (fl_rpm + rl_rpm)/2.0;
      const double right_rpm = (fr_rpm + rr_rpm)/2.0;
      const double  left_rps =  left_rpm * 2.0*M_PI/60.0;  // Convert rpm to rad/sec
      const double right_rps = right_rpm * 2.0*M_PI/60.0;  // Convert rpm to rad/sec

      // Get elapsed time since previous iteration.
      curr_iter_time = ros::WallTime::now();
      const double dt = (curr_iter_time-prev_iter_time).toNSec() * 1.0e-9;
      prev_iter_time = curr_iter_time;

      // Get average velocity (m/s) of the left and ride sides of the robot.
      const double wheel_radius_m = wheel_diam_m / 2.0;
      const double  v_left_mps =  left_rps*wheel_radius_m;
      const double v_right_mps = right_rps*wheel_radius_m;
      const double v_mps = (v_left_mps + v_right_mps) / 2.0;

      // Calculate velocity of robot in x,y,yaw.
      const double vx = std::cos(robot_yaw) * v_mps;
      const double vy = std::sin(robot_yaw) * v_mps;
      const double vyaw = std::atan((v_right_mps-v_left_mps)/wheel_spacing_m);

      // Calculate displacement in x,y,yaw.
      const double dx = vx*dt;
      const double dy = vy*dt;
      const double dyaw = vyaw*dt;

      // Update the robot state.
      robot_x   += dx;
      robot_y   += dy;
      robot_yaw += dyaw;

      // Construct the wheel odometry message header.
      nav_msgs::Odometry msg;
      msg.header.seq = iter++;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = wheel_odom_frame_id;
      msg.child_frame_id = wheel_odom_child_frame_id;

      // Construct the wheel odometry message pose.
      msg.pose.pose.position.x = robot_x;
      msg.pose.pose.position.y = robot_y;
      msg.pose.pose.position.z = 0.0;

      msg.pose.pose.orientation.x = 0.0;
      msg.pose.pose.orientation.y = 0.0;
      msg.pose.pose.orientation.z = std::sin(robot_yaw/2.0);
      msg.pose.pose.orientation.w = std::cos(robot_yaw/2.0);

      msg.pose.covariance[0*6+0] = 0.3;        // X-to-X
      msg.pose.covariance[1*6+1] = 0.3;        // Y-to-Y
      msg.pose.covariance[2*6+2] = 1000.0;     // Z-to-Z
      msg.pose.covariance[3*6+3] = 1000.0;     // Roll-to-Roll
      msg.pose.covariance[4*6+4] = 1000.0;     // Pitch-to-Pitch
      msg.pose.covariance[5*6+5] = 10.0;       // Yaw-to-Yaw

      // Construct the wheel odometry message twist.
      // Velocities are in the robot local frame.
      msg.twist.twist.linear.x = v_mps;   // Average forward speed.
      msg.twist.twist.linear.y = 0.0;     // We can't move sideways.
      msg.twist.twist.linear.z = 0.0;     // We can't jump!

      msg.twist.twist.angular.x = 0.0;    // We can't measure roll here.
      msg.twist.twist.angular.y = 0.0;    // We can't measure pitch here.
      msg.twist.twist.angular.z = vyaw;   // We can yaw around the robot center!

      msg.twist.covariance[0*6+0] =   0.01;     // X-to-X
      msg.twist.covariance[1*6+1] =   10.0;     // Y-to-Y
      msg.twist.covariance[2*6+2] = 1000.0;     // Z-to-Z
      msg.twist.covariance[3*6+3] = 1000.0;     // Roll-to-Roll
      msg.twist.covariance[4*6+4] = 1000.0;     // Pitch-to-Pitch
      msg.twist.covariance[5*6+5] = 0.3;        // Yaw-to-Yaw

      wheel_odom_pub.publish(msg);

      // Publish miscellaneous wheel data
      const int fr_enc = wheels.get_front_right_encoder();
      const int fl_enc = wheels.get_front_left_encoder();
      const int rr_enc = wheels.get_rear_right_encoder();
      const int rl_enc = wheels.get_rear_left_encoder();
      diagnostic_msgs::KeyValue fr_enc_kv;
      diagnostic_msgs::KeyValue fl_enc_kv;
      diagnostic_msgs::KeyValue rr_enc_kv;
      diagnostic_msgs::KeyValue rl_enc_kv;
      fr_enc_kv.key = 'FR Encoder';
      fl_enc_kv.key = 'FL Encoder';
      rr_enc_kv.key = 'RR Encoder';
      rl_enc_kv.key = 'RL Encoder';
      fr_enc_kv.value = std::tostring(fr_enc);
      fl_enc_kv.value = std::tostring(fl_enc);
      rr_enc_kv.value = std::tostring(rr_enc);
      rl_enc_kv.value = std::tostring(rl_enc);

      const double fr_current = wheels.get_front_right_amps();
      const double fl_current = wheels.get_front_left_amps();
      const double rr_current = wheels.get_rear_right_amps();
      const double rl_current = wheels.get_rear_left_amps();
      diagnostic_msgs::KeyValue fr_curr_kv;
      diagnostic_msgs::KeyValue fl_curr_kv;
      diagnostic_msgs::KeyValue rr_curr_kv;
      diagnostic_msgs::KeyValue rl_curr_kv;
      fr_curr_kv.key = 'FR Current';
      fl_curr_kv.key = 'FL Current';
      rr_curr_kv.key = 'RR Current';
      rl_curr_kv.key = 'RL Current';
      fr_curr_kv.value = std::tostring(fr_current);
      fl_curr_kv.value = std::tostring(fl_current);
      rr_curr_kv.value = std::tostring(rr_current);
      rl_curr_kv.value = std::tostring(rl_current);

      diagnostic_msgs::DiagnosticArray data_msg;
      data_msg.header.seq = iter++;
      data_msg.header.stamp = ros::Time::now();
      data_msg.header.frame_id = wheel_odom_frame_id;

      diagnostic_msgs::DiagnosticStatus enc_status;
      enc_status.name = 'Wheel Encoder';
      enc_status.message = 'FR FL RR RL';
      enc_status.values = {fr_enc_kv, fl_enc_kv, rr_enc_kv, rl_enc_kv};

      diagnostic_msgs::DiagnosticStatus current_status;
      current_status.name = 'Wheel Current';
      current_status.message = 'Measured in amps. FR FL RR RL';
      current_status.values = {fr_curr_kv, fl_curr_kv, rr_curr_kv, rl_curr_kv};

      data_msg.status = {enc_status, current_status};
      wheel_data_pub.publish(data_msg);
    } catch (const std::exception& e) {
      fmt::print("WARNING: {}", e.what());
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
