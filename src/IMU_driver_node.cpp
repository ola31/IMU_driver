#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include "sensor_msgs/Imu.h"

#include "IMU_driver/IMU_driver.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

IMU imu("can1",0x01,false);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "IMU_driver_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher angle_pub = nh.advertise<std_msgs::Float32>("/angle/x", 1000);
  ros::Publisher gyro_z_pub = nh.advertise<std_msgs::Float32>("/gyro/z", 1000);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

  sensor_msgs::Imu imu_msg;
  tf2::Quaternion myQuaternion;

  imu.initialize_imu();


  //imu.set_Bitrate_250();
  //imu.save_params();
  //imu.software_reset();
  //imu.set_sync_tx_cycle(10);  //10ms = 100hz
  //imu.save_params();
  //imu.software_reset();

  //imu.imu_req();
  imu.set_sync_req(true);
  imu.set_sync_req(true);

  ros::Rate loop_rate(100);  //100hz = 10ms  ,200hz = 5ms ,50hz = 20ms
  while (ros::ok())
  {

    //imu.set_Bitrate_250();
    //imu.imu_read();
    imu.read_sync_data();


    imu_msg.header.stamp    = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = imu.gyro_x;
    imu_msg.angular_velocity.y = imu.gyro_y;
    imu_msg.angular_velocity.z = imu.gyro_z;
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = imu.acc_x;
    imu_msg.linear_acceleration.y = imu.acc_y;
    imu_msg.linear_acceleration.z = imu.acc_z;
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    myQuaternion.setRPY( imu.angle_x, imu.angle_y, imu.angle_z );
    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(myQuaternion);

    //ROS_INFO_STREAM(myQuaternion);
/*
    imu_msg.orientation.w = 0;//imu.quat[0];
    imu_msg.orientation.x = 0;//imu.quat[1];
    imu_msg.orientation.y = 0;//imu.quat[2];
    imu_msg.orientation.z = 1;//imu.quat[3];
    */

   //geometry_msgs::Quaternion quat_msg;
   //quat_msg = myQuaternion.toMsg(myQuaternion);
   //myQuaternion.convert(quat_msg , myQuaternion);
/*
    imu_msg.orientation.w = quat_msg.w;//imu.quat[0];
    imu_msg.orientation.x = quat_msg.x;//imu.quat[1];
    imu_msg.orientation.y = quat_msg.y;//imu.quat[2];
    imu_msg.orientation.z = quat_msg.z;//imu.quat[3];

   */

    //imu_msg.orientation.w = 1;//imu.quat[0];
    //imu_msg.orientation.x = 0;//imu.quat[1];
    //imu_msg.orientation.y = 0;//imu.quat[2];
    //imu_msg.orientation.z = 0;

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;


    std_msgs::Float32 msg;
    msg.data = imu.gyro_z;



    //angle_pub.publish(msg);
    imu_pub.publish(imu_msg);

    gyro_z_pub.publish(msg);
   //ROS_INFO("testing");

    ros::spinOnce();

    //imu.imu_req();
    loop_rate.sleep();
  }

  return 0;
}
