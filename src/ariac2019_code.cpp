#include "ros/ros.h"

#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "ur_kinematics/ur_kin.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "actionlib/client/service_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <sstream>

  std::vector<osrf_gear::Order> order_vector;
  osrf_gear::GetMaterialLocations location;
  ros::ServiceClient location_client;  	

  osrf_gear::LogicalCameraImage cameralist[10];
  tf2_ros::Buffer tfBuffer;

  osrf_gear::Model logical_camera;
  osrf_gear::Product product;
  trajectory_msgs::JointTrajectory joint_trajectory;
  geometry_msgs::PoseStamped part_pose, goal_pose;
  sensor_msgs::JointState joint_states_current;
  std::string str="";  
  




//Update the joint_states at every moment
void Callback_joint_states(const  sensor_msgs::JointState::ConstPtr& joint_states)
{
  joint_states_current = * joint_states;
}

// Must select which of the num_sols solution to use.
int find_sol_to_use(double q_sols[8][6], int num_sol)
{
  for(int i = num_sol - 1; i >= 0; i--)
  {
    //Try the range given by the professor, but it will hit the camera. The -pi/2 to pi/2 should be 0 to pi. Maybe my angle setting is wrong
    if(q_sols[i][0] > 0 && q_sols[i][0] < 3.1416)  
    {
      //The‌ ‌second‌ ‌joint
        if((q_sols[i][1] > -3.1416 && q_sols[i][1] < 0) || q_sols[i][1] > 3.1416 && q_sols[i][1] < 6.2832)
        {
          return i;
        }
        else
          continue;
    }
    else
      continue;
  }
  ROS_WARN("Can't find a proper solution");
  return -1;


}

//Used to fill in the trajectory so that it can be published later.
void JointTrajectory()
{
   double T_pose[4][4];
   double T_des [4][4]= {{0.0, -1.0, 0.0, goal_pose.pose.position.x}, \
                                               {0.0, 0.0, 1.0, goal_pose.pose.position.y}, \
                                               {-1.0, 0.0, 0.0, goal_pose.pose.position.z}, \
                                               {0.0, 0.0, 0.0, 1.0}};
  double q_pose[6];
  q_pose[0] = 3.14;
  q_pose[1] = -1.13;
  q_pose[2] = 1.51;
  q_pose[3] = 3.77;
  q_pose[4] =-1.51;
  q_pose[5] = 0;

  double q_des[8][6];
  ur_kinematics::forward(&q_pose[0], &T_pose[0][0]);

  int num_sols = ur_kinematics::inverse(&T_des[0][0], &q_des[0][0]);
  int count = 0;
  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.header.frame_id = "/world";

  joint_trajectory.joint_names.clear();
  joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
  joint_trajectory.joint_names.push_back("shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("shoulder_lift_joint");  
  joint_trajectory.joint_names.push_back("elbow_joint");
  joint_trajectory.joint_names.push_back("wrist_1_joint");
  joint_trajectory.joint_names.push_back("wrist_2_joint");
  joint_trajectory.joint_names.push_back("wrist_3_joint");

  joint_trajectory.points.resize(2);
  joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
  for(int indy = 0; indy < joint_trajectory.joint_names.size(); indy++)
  {
    for(int indz = 0; indz < joint_states_current.name.size(); indz++)
    {
      if(joint_trajectory.joint_names[indy] == joint_states_current.name[indz])
      {
        joint_trajectory.points[0].positions[indy] = joint_states_current.position[indz];
        break;
      }
    }
  }
  joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

  int q_des_indx = find_sol_to_use(q_des, num_sols);

  joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

  joint_trajectory.points[1].positions[0] = joint_states_current.position[1];
  for(int indy = 0; indy < 6; indy++)
  {
    joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
  }
  joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

}

//In the Appendix.The coordinate of the part based on the camera is converted to the coordinate xyz based on the basic joint
void goal_pose_Transform()
{
  geometry_msgs::TransformStamped tfStamped;
  
  try 
  {
    tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame", ros::Time(0.0), ros::Duration(1.0));
    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
  }
  catch(tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  part_pose.pose = logical_camera.pose;
  tf2::doTransform(part_pose, goal_pose, tfStamped);
  goal_pose.pose.position.z += 0.10;
  goal_pose.pose.orientation.w = 0.707;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.707;
  goal_pose.pose.orientation.z = 0.0;

}

void myordersCallback(const osrf_gear::Order::ConstPtr& message_holder)
{
 
  order_vector.push_back(*message_holder);
  ROS_INFO("received orderid is :%s", message_holder->order_id.c_str());
  ROS_INFO("the first storage is :%lu", order_vector.size());
  ROS_INFO("the type of the first object is :%s", message_holder->shipments[0].products[0].type.c_str());

  location.request.material_type = message_holder->shipments[0].products[0].type;
  str = message_holder->shipments[0].products[0].type;
  location_client.call(location);
  ROS_INFO("Location is [%s]", location.response.storage_units[0].unit_id.c_str());

}


void Callback_agv1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[0] = *msg;
  
}

void Callback_agv2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[1] = *msg;
  
}

void Callback_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[2] = *msg;
  
}

void Callback_bin2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[3] = *msg;
  
}

void Callback_bin3(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[4] = *msg;
  
}

void Callback_bin4(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[5] = *msg;
  
}

void Callback_bin5(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[6] = *msg;
  
}


void Callback_bin6(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[7] = *msg;
  
}

void Callback_quality_control_sensor_1(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[8] = *msg;
  
}


void Callback_quality_control_sensor_2(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{

  cameralist[9] = *msg;
  
}


int find_location(std::string unit_name)
{
  int i;
  if( unit_name == "agv1")
  {
    i=0;
  }

  if( unit_name == "agv2")
  {
    i=1;
  }  
  
  if( unit_name == "bin1")
  {
    i=2;
  }  
  
  if( unit_name == "bin2")
  {
    i=3;
  }  
  
  if( unit_name == "bin3")
  {
    i=4;
  }

    if( unit_name == "bin4")
  {
    i=5;
  }

    if( unit_name == "bin5")
  {
    i=6;
  }
  
    if( unit_name == "bin6")
  {
    i=7;
  }
    
    if( unit_name == "quality_control_sensor_1")
  {
    i=8;
  }
    
    if( unit_name == "quality_control_sensor_2")
  {
    i=9;
  }

  int length;
  length = end(cameralist[i].models)-begin(cameralist[i].models);
  for(int j=0; j<length; j++)
  {
    logical_camera = cameralist[i].models[j];
    ROS_INFO("position: x = %f, y = %f, z = %f", cameralist[i].models[j].pose.position.x, cameralist[i].models[j].pose.position.y, cameralist[i].models[j].pose.position.z);
    ROS_INFO("orientation: x = %f, y = %f, z = %f, w =%f", cameralist[i].models[j].pose.orientation.x, cameralist[i].models[j].pose.orientation.y, cameralist[i].models[j].pose.orientation.z, cameralist[i].models[j].pose.orientation.w);
    return i;
  }
}








int main(int argc, char* argv[])
{

  ros::init(argc, argv, "ariac_node");

  ros::NodeHandle n;

  std_srvs::Trigger begin_comp;

  ros::service::waitForService("/ariac/start_competition");
  ROS_INFO("Waiting for service");
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");


  int service_call_succeeded;
  ROS_INFO("Call service to start the competition");
  service_call_succeeded = begin_client.call(begin_comp);
  
  if (service_call_succeeded == 0) 
  {
    ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  }
  else
  {
    if (begin_comp.response.success) 
    {
      ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
    }
    else
    {
      ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
    }
  }

  ros::Rate loop_rate(1);

 

  order_vector.clear();


  ros::Subscriber sub_order = n.subscribe("/ariac/orders", 1000, myordersCallback);

  ros::Subscriber sub_agv1 = n.subscribe("/ariac/logical_camera_agv1", 1000, Callback_agv1);
  
  ros::Subscriber sub_agv2 = n.subscribe("/ariac/logical_camera_agv2", 1000, Callback_agv2);  
  
  ros::Subscriber sub_bin1 = n.subscribe("/ariac/logical_camera_bin1", 1000, Callback_bin1);  
  
  ros::Subscriber sub_bin2 = n.subscribe("/ariac/logical_camera_bin2", 1000, Callback_bin2);  
  
  ros::Subscriber sub_bin3 = n.subscribe("/ariac/logical_camera_bin3", 1000, Callback_bin3);
 
  ros::Subscriber sub_bin4 = n.subscribe("/ariac/logical_camera_bin4", 1000, Callback_bin4);  
  
  ros::Subscriber sub_bin5 = n.subscribe("/ariac/logical_camera_bin5", 1000, Callback_bin5);
 
  ros::Subscriber sub_bin6 = n.subscribe("/ariac/logical_camera_bin6", 1000, Callback_bin6);
  
  ros::Subscriber sub_quality_control_sensor_1 = n.subscribe("/ariac/quality_control_sensor_1", 1000, Callback_quality_control_sensor_1);
    
  ros::Subscriber sub_quality_control_sensor_2 = n.subscribe("/ariac/quality_control_sensor_2", 1000, Callback_quality_control_sensor_2);

  ros::Subscriber joint_states_h = n.subscribe("/ariac/arm1/joint_states",10,  Callback_joint_states);

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);

  ros::Publisher pub_command = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

  tf2_ros::TransformListener tfListener(tfBuffer);

  while(ros::ok())
  {
    int seq_num = 0;    
    unsigned int order_vector_length = 0;
    order_vector_length = order_vector.size();
    if (order_vector_length == 0)
    {
      ROS_INFO("No orders");
    }
    else
    {
       
       //move robot arm  to the first part
        int storage_location = find_location(location.response.storage_units[0].unit_id);
        goal_pose_Transform();
        JointTrajectory();
        pub_command.publish(joint_trajectory);
        loop_rate.sleep();
        //There is a problem here. Maybe my angle setting is wrong, causing the robotic arm to collide with the camera. But as long as I give it enough time, the robotic arm can hit it. So I set two loop_rate.sleep(); to make it easier for the robot arm to reach the part.
        loop_rate.sleep();
        
        
        //Move the robot from the first part to the ninth part. move the manipulator over every part in a bin.
        int models_length =  end(cameralist[storage_location].models) - begin(cameralist[storage_location].models);
        for(int i = 0; i < models_length; i++)
        {
          if(cameralist[storage_location].models[i].type == str)
          {
            logical_camera = cameralist[storage_location].models[i];
            goal_pose_Transform();
            JointTrajectory(); 
            control_msgs::FollowJointTrajectoryAction joint_trajectory_as;
            joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
            joint_trajectory_as.action_goal.header.seq = seq_num++;
            joint_trajectory_as.action_goal.header.stamp = ros::Time::now();
            joint_trajectory_as.action_goal.header.frame_id = "/world";
            actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
            ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
          }
        }
      order_vector.erase(order_vector.begin());
    }
   
    ros::spinOnce();

    loop_rate.sleep();


  }

  return 0;
}



