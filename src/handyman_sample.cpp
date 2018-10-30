#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <handyman/HandymanMsg.h>
#include <nodelet/nodelet.h>

class HandymanSample
{
private:
  enum Step
  {
    Initialize,
    Ready,
    WaitForInstruction,
    GoToRoom1,
    GoToRoom2,
    MoveToInFrontOfTarget,
    Grasp,
    WaitForGrasping,
    ComeBack,
    TaskFinished,
  };

  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_INSTRUCTION      = "Instruction";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";

  const std::string MSG_I_AM_READY     = "I_am_ready";
  const std::string MSG_ROOM_REACHED   = "Room_reached";
  const std::string MSG_OBJECT_GRASPED = "Object_grasped";
  const std::string MSG_TASK_FINISHED  = "Task_finished";

  trajectory_msgs::JointTrajectory arm_joint_trajectory_;
  trajectory_msgs::JointTrajectory gripper_joint_trajectory_;

  int step_;

  std::string instruction_msg;

  bool is_started_;
  bool is_finished_;
  bool is_failed_;

  void init()
  {
    // Arm Joint Trajectory
    std::vector<std::string> arm_joint_names {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

    trajectory_msgs::JointTrajectoryPoint arm_joint_point;

    arm_joint_trajectory_.joint_names = arm_joint_names;
    arm_joint_trajectory_.points.push_back(arm_joint_point);

    // Gripper Joint Trajectory
    std::vector<std::string> gripper_joint_names {"hand_l_proximal_joint", "hand_r_proximal_joint"};

    trajectory_msgs::JointTrajectoryPoint gripper_joint_point;

    gripper_joint_trajectory_.joint_names = gripper_joint_names;
    gripper_joint_trajectory_.points.push_back(gripper_joint_point);

    step_ = Initialize;

    reset();
  }

  void reset()
  {
    instruction_msg = "";
    is_started_  = false;
    is_finished_ = false;
    is_failed_   = false;

    std::vector<double> arm_positions { 0.0, 0.0, 0.0, 0.0, 0.0 };
    arm_joint_trajectory_.points[0].positions = arm_positions;

    std::vector<double> gripper_positions { 0.0, 0.0 };
    gripper_joint_trajectory_.points[0].positions = gripper_positions;
  }


  void messageCallback(const handyman::HandymanMsg::ConstPtr& message)
  {
    ROS_INFO("Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message.c_str()==MSG_ARE_YOU_READY)
    {
      if(step_==Ready)
      {
        is_started_ = true;
      }
    }
    if(message->message.c_str()==MSG_INSTRUCTION)
    {
      if(step_==WaitForInstruction)
      {
        instruction_msg = message->detail.c_str();
      }
    }
    if(message->message.c_str()==MSG_TASK_SUCCEEDED)
    {
      if(step_==TaskFinished)
      {
        is_finished_ = true;
      }
    }
    if(message->message.c_str()==MSG_TASK_FAILED)
    {
      is_failed_ = true;
    }
    if(message->message.c_str()==MSG_MISSION_COMPLETE)
    {
      exit(EXIT_SUCCESS);
    }
  }

  void sendMessage(ros::Publisher &publisher, const std::string &message)
  {
    ROS_INFO("Send message:%s", message.c_str());

    handyman::HandymanMsg handyman_msg;
    handyman_msg.message = message;
    publisher.publish(handyman_msg);
  }

  tf::StampedTransform getTfBase(tf::TransformListener &tf_listener)
  {
    tf::StampedTransform tf_transform;

    try
    {
      tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), tf_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    return tf_transform;
  }

  void moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z)
  {
    geometry_msgs::Twist twist;

    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.linear.z  = linear_z;
    twist.angular.x = angular_x;
    twist.angular.y = angular_y;
    twist.angular.z = angular_z;

    publisher.publish(twist);
  }

  void stopBase(ros::Publisher &publisher)
  {
    moveBase(publisher, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  void moveArm(ros::Publisher &publisher, const std::vector<double> &positions, ros::Duration &duration)
  {
    arm_joint_trajectory_.points[0].positions = positions;
    arm_joint_trajectory_.points[0].time_from_start = duration;

    publisher.publish(arm_joint_trajectory_);
  }

  void moveArm(ros::Publisher &publisher, const std::vector<double> &positions)
  {
    ros::Duration duration;
    duration.sec = 1;
    moveArm(publisher, positions, duration);
  }

  void grasp(ros::Publisher &publisher)
  {
    ros::Duration duration;
    duration.sec = 2;
    std::vector<double> gripper_positions { -0.05, +0.05 };
    gripper_joint_trajectory_.points[0].positions = gripper_positions;
    gripper_joint_trajectory_.points[0].time_from_start = duration;

    publisher.publish(gripper_joint_trajectory_);
  }

  void openHand(ros::Publisher &publisher)
  {
    ros::Duration duration;
    duration.sec = 2;
    std::vector<double> gripper_positions { +0.611, -0.611 };
    gripper_joint_trajectory_.points[0].positions = gripper_positions;
    gripper_joint_trajectory_.points[0].time_from_start = duration;

    publisher.publish(gripper_joint_trajectory_);
  }

public:
  int run(int argc, char **argv)
  {
    ros::init(argc, argv, "handyman_sample");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);

    std::string sub_msg_to_robot_topic_name;
    std::string pub_msg_to_moderator_topic_name;
    std::string pub_base_twist_topic_name;
    std::string pub_arm_trajectory_topic_name;
    std::string pub_gripper_trajectory_topic_name;

    node_handle.param<std::string>("sub_msg_to_robot_topic_name",       sub_msg_to_robot_topic_name,       "/handyman/message/to_robot");
    node_handle.param<std::string>("pub_msg_to_moderator_topic_name",   pub_msg_to_moderator_topic_name,   "/handyman/message/to_moderator");
    node_handle.param<std::string>("pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/command_velocity");
    node_handle.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
    node_handle.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_trajectory_controller/command");

    init();

    ros::Time waiting_start_time;

    ROS_INFO("Handyman sample start!");

    ros::Subscriber sub_msg                = node_handle.subscribe<handyman::HandymanMsg>(sub_msg_to_robot_topic_name, 100, &HandymanSample::messageCallback, this);
    ros::Publisher  pub_msg                = node_handle.advertise<handyman::HandymanMsg>(pub_msg_to_moderator_topic_name, 10);
    ros::Publisher  pub_base_twist         = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
    ros::Publisher  pub_arm_trajectory     = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    ros::Publisher  pub_gripper_trajectory = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);

    tf::TransformListener tf_listener;

    while (ros::ok())
    {
      if(is_failed_)
      {
        ROS_INFO("Task failed!");
        step_ = Initialize;
      }

      switch(step_)
      {
        case Initialize:
        {
          reset();
          step_++;
          break;
        }
        case Ready:
        {
          if(is_started_)
          {
            sendMessage(pub_msg, MSG_I_AM_READY);

            ROS_INFO("Task start!");

            step_++;
          }
          break;
        }
        case WaitForInstruction:
        {
          if(instruction_msg!="")
          {
            ROS_INFO("%s", instruction_msg.c_str());

            openHand(pub_gripper_trajectory);

            step_++;
          }
          break;
        }
        case GoToRoom1:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() >= -0.2)
          {
            moveBase(pub_base_twist, +1.0, 0.0, 0.0, 0.0, 0.0, -1.0);
          }
          else
          {
            stopBase(pub_base_twist);
            step_++;
          }

          break;
        }
        case GoToRoom2:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() >= -0.6)
          {
            moveBase(pub_base_twist, +1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          }
          else
          {
            stopBase(pub_base_twist);
            sendMessage(pub_msg, MSG_ROOM_REACHED);

            step_++;
          }
          break;
        }
        case MoveToInFrontOfTarget:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() >= -1.4)
          {
            std::vector<double> positions { 0.22, -1.57, 0.0, 0.0, 0.0 };
            ros::Duration duration;
            duration.sec = 1;

            moveBase(pub_base_twist, +0.7, 0.0, 0.0, 0.0, 0.0, 0.0);
            moveArm(pub_arm_trajectory, positions, duration);
          }
          else
          {
            stopBase(pub_base_twist);

            step_++;
          }

          break;
        }
        case Grasp:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() >= -1.5)
          {
            moveBase(pub_base_twist, +1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          }
          else
          {
            stopBase(pub_base_twist);
            grasp(pub_gripper_trajectory);

            waiting_start_time = ros::Time::now();
            step_++;
          }

          break;
        }
        case WaitForGrasping:
        {
          if(ros::Time::now() - waiting_start_time > ros::Duration(3, 0))
          {
            sendMessage(pub_msg, MSG_OBJECT_GRASPED);
            step_++;
          }

          break;
        }
        case ComeBack:
        {
          tf::StampedTransform tf_transform = getTfBase(tf_listener);

          if(tf_transform.getOrigin().y() <= 0.0)
          {
            moveBase(pub_base_twist, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          }
          else
          {
            stopBase(pub_base_twist);
            sendMessage(pub_msg, MSG_TASK_FINISHED);

            step_++;
          }

          break;
        }
        case TaskFinished:
        {
          if(is_finished_)
          {
            ROS_INFO("Task finished!");
            step_ = Initialize;
          }

          break;
        }
      }

      ros::spinOnce();

      loop_rate.sleep();
    }

    return EXIT_SUCCESS;
  }
};


int main(int argc, char **argv)
{
  HandymanSample handyman_sample;

  return handyman_sample.run(argc, argv);
};

