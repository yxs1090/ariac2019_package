#include "ros/ros.h"

#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include <sstream>

  std::vector<osrf_gear::Order> order_vector;
  osrf_gear::GetMaterialLocations location;
  ros::ServiceClient location_client;  	

  osrf_gear::LogicalCameraImage cameralist[10];
  
  std::string str="";  
  



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


void find_location(std::string part)
{

  for(int i=0; i<10; i++)
  {
    int length;
    length = end(cameralist[i].models)-begin(cameralist[i].models);
    for(int j=0; j<length; j++)
    {
      if(part == cameralist[i].models[j].type)
      {
        std::string someplace;
        switch(i)
        {
          case 0:
            someplace = "agv1";
            break;
     
          case 1:
            someplace = "agv1";
            break;
            
          case 2:
            someplace = "bin1";
            break;
            
          case 3:
            someplace = "bin2";
            break;
            
          case 4:
            someplace = "bin3";
            break;
          
          case 5:
            someplace = "bin4";
            break;
            
          case 6:
            someplace = "bin5";
            break;
            
          case 7:
            someplace = "bin6";
            break;
            
          case 8:
            someplace = "quality_control_sensor_1";
            break;
            
          case 9:
            someplace = "quality_control_sensor_2";
            break;       
        }
        ROS_INFO("%s in the %s", part.c_str(), someplace.c_str());
        ROS_INFO("pose info:");
        ROS_INFO("position: x = %f, y = %f, z = %f", cameralist[i].models[j].pose.position.x, cameralist[i].models[j].pose.position.y, cameralist[i].models[j].pose.position.z);
        ROS_INFO("orientation: x = %f, y = %f, z = %f, w =%f", cameralist[i].models[j].pose.orientation.x, cameralist[i].models[j].pose.orientation.y, cameralist[i].models[j].pose.orientation.z, cameralist[i].models[j].pose.orientation.w);
      }
    }
    
  }

 
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "read_node");

  ros::NodeHandle n;

  std_srvs::Trigger begin_comp;

  location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");



  ros::service::waitForService("/ariac/start_competition");
  ROS_INFO("Waiting for service");
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");



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

  ros::Rate loop_rate(10);

 

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

  while(ros::ok())
  {

    unsigned int order_vector_length = 0;
    order_vector_length = order_vector.size();
    if (order_vector_length == 0)
    {
      ROS_INFO("No orders");
    }
    else
    {
      find_location(str);
      order_vector.erase(order_vector.begin());
    }
   
    ros::spinOnce();

    loop_rate.sleep();


  }

  return 0;
}



