/*
Copyright (c) 2020, Marvelmind Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "localization_interfaces/msg/hedge_pos.hpp"
#include "localization_interfaces/msg/hedge_pos_a.hpp"
#include "localization_interfaces/msg/hedge_pos_ang.hpp"
#include "localization_interfaces/msg/beacon_pos_a.hpp"
#include "localization_interfaces/msg/hedge_imu_raw.hpp"
#include "localization_interfaces/msg/hedge_imu_fusion.hpp"
#include "localization_interfaces/msg/beacon_distance.hpp"
#include "localization_interfaces/msg/hedge_telemetry.hpp"
#include "localization_interfaces/msg/hedge_quality.hpp"
#include "localization_interfaces/msg/marvelmind_waypoint.hpp"
#include <visualization_msgs/msg/marker.hpp>


#define ROS_NODE_NAME "subscriber_test"
#define HEDGE_POSITION_TOPIC_NAME "hedge_pos"
#define HEDGE_POSITION_ADDRESSED_TOPIC_NAME "hedge_pos_a"
#define HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "hedge_pos_ang"

#define BEACONS_POSITION_ADDRESSED_TOPIC_NAME "beacons_pos_a"

#define HEDGE_IMU_RAW_TOPIC_NAME "hedge_imu_raw"
#define HEDGE_IMU_FUSION_TOPIC_NAME "hedge_imu_fusion"

#define BEACON_RAW_DISTANCE_TOPIC_NAME "beacon_raw_distance"

#define HEDGE_TELEMETRY_TOPIC_NAME "hedge_telemetry"
#define HEDGE_QUALITY_TOPIC_NAME "hedge_quality"
#define MARVELMIND_WAYPOINT_TOPIC_NAME "marvelmind_waypoint"

#define SCALE_HEDGE 3.0

uint32_t rviz_shape;

float orientation_qx= 0.0;
float orientation_qy= 0.0;
float orientation_qz= 0.0;
float orientation_qw= 1.0;

typedef enum {objHedge, objBeacon} DrawObjectType;

class SubscriberTest : public rclcpp::Node
{
  public:
    SubscriberTest()
    : Node("subscriber_test")
    {
      // Declare publisher for rviz visualization
      rviz_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);

      // Set our initial shape type to be a cube
      rviz_shape = visualization_msgs::msg::Marker::CUBE;

      // Declare subscribers
      subHedgeWithAngle = this->create_subscription
        <localization_interfaces::msg::HedgePosAng>(
        HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME, 10, std::bind(&SubscriberTest::hedgePosAngCallback, this, std::placeholders::_1));

      subIMURaw = this->create_subscription
        <localization_interfaces::msg::HedgeImuRaw>(
        HEDGE_IMU_RAW_TOPIC_NAME, 10, std::bind(&SubscriberTest::IMURawCallback, this, std::placeholders::_1));

      subBeacons = this->create_subscription
        <localization_interfaces::msg::BeaconPosA>(
        BEACONS_POSITION_ADDRESSED_TOPIC_NAME, 10, std::bind(&SubscriberTest::beaconsPosCallback, this, std::placeholders::_1));

      subIMUFusion = this->create_subscription
        <localization_interfaces::msg::HedgeImuFusion>(
        HEDGE_IMU_FUSION_TOPIC_NAME, 10, std::bind(&SubscriberTest::IMUFusionCallback, this, std::placeholders::_1));

      subRawDistance = this->create_subscription
        <localization_interfaces::msg::BeaconDistance>(
        BEACON_RAW_DISTANCE_TOPIC_NAME, 10, std::bind(&SubscriberTest::RawDistanceCallback, this, std::placeholders::_1));

      subTelemetry = this->create_subscription
        <localization_interfaces::msg::HedgeTelemetry>(
        HEDGE_TELEMETRY_TOPIC_NAME, 10, std::bind(&SubscriberTest::telemetryCallback, this, std::placeholders::_1));

      subQuality = this->create_subscription
        <localization_interfaces::msg::HedgeQuality>(
        HEDGE_QUALITY_TOPIC_NAME, 10, std::bind(&SubscriberTest::qualityCallback, this, std::placeholders::_1));

      subWaypoint = this->create_subscription
        <localization_interfaces::msg::MarvelmindWaypoint>(
        MARVELMIND_WAYPOINT_TOPIC_NAME, 10, std::bind(&SubscriberTest::waypointCallback, this, std::placeholders::_1));
    }

  private:

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub;

    rclcpp::Subscription<localization_interfaces::msg::HedgePosAng>::SharedPtr subHedgeWithAngle;
    
    rclcpp::Subscription<localization_interfaces::msg::HedgeImuRaw>::SharedPtr subIMURaw;

    rclcpp::Subscription<localization_interfaces::msg::BeaconPosA>::SharedPtr  subBeacons;

    rclcpp::Subscription<localization_interfaces::msg::HedgeImuFusion>::SharedPtr subIMUFusion; 

    rclcpp::Subscription<localization_interfaces::msg::BeaconDistance>::SharedPtr subRawDistance;

    rclcpp::Subscription<localization_interfaces::msg::HedgeTelemetry>::SharedPtr subTelemetry;

    rclcpp::Subscription<localization_interfaces::msg::HedgeQuality>::SharedPtr subQuality;

    rclcpp::Subscription<localization_interfaces::msg::MarvelmindWaypoint>::SharedPtr subWaypoint;

    void showRvizObject(uint8_t address, float x, float y, float z, DrawObjectType obj) 
    {
      uint8_t lifeTime;
      
      visualization_msgs::msg::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = this->now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = address;

        // Set the marker type
        marker.type = rviz_shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        if (obj == objHedge)
          {
            marker.pose.orientation.x = orientation_qx;
            marker.pose.orientation.y = orientation_qy;
            marker.pose.orientation.z = orientation_qz;
            marker.pose.orientation.w = orientation_qw;
          }
        else
          {
        marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
          }   

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.05*SCALE_HEDGE;
        marker.scale.y = 0.05*SCALE_HEDGE;
        marker.scale.z = 0.02*SCALE_HEDGE;
        // Set the color -- be sure to set alpha to something non-zero!
        if (obj == objHedge)
          {
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
          }
        else
          {
        marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
          }   
        marker.color.a = 1.0;

        if (obj == objHedge) lifeTime= 5;
                        else lifeTime= 25;
        marker.lifetime = rclcpp::Duration(lifeTime);
        
        this->rviz_marker_pub->publish(marker);
    }

    void hedgePosAngCallback(const localization_interfaces::msg::HedgePosAng::SharedPtr hedge_pos_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Hedgehog data: Address= %d, timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  Angle: %.1f  flags=%d", 	
            (int) hedge_pos_msg->address, 
            (int) hedge_pos_msg->timestamp_ms, 
            (float) hedge_pos_msg->x_m, (float) hedge_pos_msg->y_m, (float) hedge_pos_msg->z_m,
            (float) hedge_pos_msg->angle,  
            (int) hedge_pos_msg->flags);
            
      if ((hedge_pos_msg->flags&(1<<0))==0)
        {				
          showRvizObject(hedge_pos_msg->address,hedge_pos_msg->x_m, hedge_pos_msg->y_m, hedge_pos_msg->z_m, objHedge);
        }  
    }

    /*
    void hedgePosCallback(const marvelmind_nav::hedge_pos_a& hedge_pos_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Hedgehog data: Address= %d, timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  flags=%d", 	
            (int) hedge_pos_msg.address, 
            (int) hedge_pos_msg.timestamp_ms, 
            (float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,  
            (int) hedge_pos_msg.flags);
            
      if ((hedge_pos_msg.flags&(1<<0))==0)
        {				
          showRvizObject(hedge_pos_msg.address,hedge_pos_msg.x_m, hedge_pos_msg.y_m, hedge_pos_msg.z_m, objHedge);
        }  
    }

    void hedgePos_noaddressCallback(const marvelmind_nav::hedge_pos& hedge_pos_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Hedgehog data: Timestamp= %d, X=%.3f  Y= %.3f  Z=%.3f  flags=%d", 	
            (int) hedge_pos_msg.timestamp_ms, 
            (float) hedge_pos_msg.x_m, (float) hedge_pos_msg.y_m, (float) hedge_pos_msg.z_m,  
            (int) hedge_pos_msg.flags);
            
      if ((hedge_pos_msg.flags&(1<<0))==0)
        {				
          showRvizObject(0,hedge_pos_msg.x_m, hedge_pos_msg.y_m, hedge_pos_msg.z_m, objHedge);
        }  
    }
    * */

    void beaconsPosCallback(const localization_interfaces::msg::BeaconPosA::SharedPtr beacon_pos_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Stationary beacon data: Address= %d, X=%.3f  Y= %.3f  Z=%.3f", 	
            (int) beacon_pos_msg->address, 
            (float) beacon_pos_msg->x_m, (float) beacon_pos_msg->y_m, (float) beacon_pos_msg->z_m);
            
      showRvizObject(beacon_pos_msg->address, beacon_pos_msg->x_m, beacon_pos_msg->y_m, beacon_pos_msg->z_m, objBeacon);
    }


    void IMURawCallback(const localization_interfaces::msg::HedgeImuRaw::SharedPtr hedge_imu_raw_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Raw IMU: Timestamp: %08d, aX=%05d aY=%05d aZ=%05d  gX=%05d gY=%05d gZ=%05d  cX=%05d cY=%05d cZ=%05d", 	
            (int) hedge_imu_raw_msg->timestamp_ms,
            (int) hedge_imu_raw_msg->acc_x, (int) hedge_imu_raw_msg->acc_y, (int) hedge_imu_raw_msg->acc_z,
            (int) hedge_imu_raw_msg->gyro_x, (int) hedge_imu_raw_msg->gyro_y, (int) hedge_imu_raw_msg->gyro_z,
            (int) hedge_imu_raw_msg->compass_x, (int) hedge_imu_raw_msg->compass_y, (int) hedge_imu_raw_msg->compass_z);
    }

    void IMUFusionCallback(const localization_interfaces::msg::HedgeImuFusion::SharedPtr hedge_imu_fusion_msg)
    {
      RCLCPP_INFO(this->get_logger(),"IMU fusion: Timestamp: %08d, X=%.3f  Y= %.3f  Z=%.3f  q=%.3f,%.3f,%.3f,%.3f v=%.3f,%.3f,%.3f  a=%.3f,%.3f,%.3f", 	
            (int) hedge_imu_fusion_msg->timestamp_ms,
            (float) hedge_imu_fusion_msg->x_m, (float) hedge_imu_fusion_msg->y_m, (float) hedge_imu_fusion_msg->z_m,
            (float) hedge_imu_fusion_msg->qw, (float) hedge_imu_fusion_msg->qx, (float) hedge_imu_fusion_msg->qy, (float) hedge_imu_fusion_msg->qz,
            (float) hedge_imu_fusion_msg->vx, (float) hedge_imu_fusion_msg->vy, (float) hedge_imu_fusion_msg->vz,
            (float) hedge_imu_fusion_msg->ax, (float) hedge_imu_fusion_msg->ay, (float) hedge_imu_fusion_msg->az);
            
      orientation_qx= hedge_imu_fusion_msg->qx;
      orientation_qy= hedge_imu_fusion_msg->qy;
      orientation_qz= hedge_imu_fusion_msg->qz;
      orientation_qw= hedge_imu_fusion_msg->qw;
    }

    void RawDistanceCallback(const localization_interfaces::msg::BeaconDistance::SharedPtr beacon_raw_distance_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Raw distance: %02d ==> %02d,  Distance= %.3f ", 	
            (int) beacon_raw_distance_msg->address_hedge,
            (int) beacon_raw_distance_msg->address_beacon,
            (float) beacon_raw_distance_msg->distance_m);
    }

    void telemetryCallback(const localization_interfaces::msg::HedgeTelemetry::SharedPtr hedge_telemetry_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Vbat= %.3f V, RSSI= %02d ", 	
            (float) hedge_telemetry_msg->battery_voltage,
            (int) hedge_telemetry_msg->rssi_dbm);
    }

    void qualityCallback(const localization_interfaces::msg::HedgeQuality::SharedPtr hedge_quality_msg)
    {
      RCLCPP_INFO(this->get_logger(),"Quality: Address= %d,  Quality= %02d %% ", 	
            (int) hedge_quality_msg->address,
            (int) hedge_quality_msg->quality_percents);
    }

    void waypointCallback(const localization_interfaces::msg::MarvelmindWaypoint::SharedPtr marvelmind_waypoint_msg)
    {
      int n= marvelmind_waypoint_msg->item_index+1;
        RCLCPP_INFO(this->get_logger(), "Waypoint %03d/%03d: Type= %03d,  Param1= %05d, Param2= %05d, Param3= %05d ", 	
            (int) n,
            (int) marvelmind_waypoint_msg->total_items, marvelmind_waypoint_msg->movement_type,
            marvelmind_waypoint_msg->param1, marvelmind_waypoint_msg->param2, marvelmind_waypoint_msg->param3);
    }
};


/**
 * Test subscriber node for getting data from Marvelmind publishers nodes
 */
int main(int argc, char **argv)
{
	
  // initialize ROS node
  rclcpp::init(argc, argv);
  
  //ros::Subscriber subHedge = rosNode.subscribe(HEDGE_POSITION_ADDRESSED_TOPIC_NAME, 1000, hedgePosCallback);
  //ros::Subscriber subHedge_noaddress = rosNode.subscribe(HEDGE_POSITION_TOPIC_NAME, 1000, hedgePos_noaddressCallback);

  rclcpp::spin(std::make_shared<SubscriberTest>());
  rclcpp::shutdown();

  return 0;
}
