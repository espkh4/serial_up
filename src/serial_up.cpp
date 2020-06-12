#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include "cereal_port/CerealPort.h"
#include "cereal_port/CommStruct.h"
#include <cstdlib>
#define m_pi 3.1415926536
#define REPLY_SIZE 1000000
#define TIMEOUT 1000

int main(int argc, char **argv)
{
    char command ='1';
    char reply[REPLY_SIZE];

	/*Serial Communication*/
	cereal::CerealPort device;	
    try{ device.open("/dev/ttyUSB0",921600); }//Opens serial port
	catch(cereal::Exception& e)
	{
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
	}
	if(device.portOpen())
	{
	ROS_INFO("The serial port is opened @ baudrate %d",device.baudRate());
	}
	

	if(device.write(&command,sizeof(command))==1)
	{
	printf("Command sent\n");
	}
	else
	{
	printf("Error in sending requested command\n");
	} 

  ros::init(argc, argv, "Gyro");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("Gyro_value", 1000);
  ros::Rate loop_rate(1000);
  std_msgs::Float32 msg;

  while (ros::ok())
  {
	try{ device.read(reply, 76,TIMEOUT);}	//Reading
	catch(cereal::TimeoutException& e)	//Catch any possible errors
	{
	ROS_ERROR("Timeout!");
	}

    for (int i=0;i<76;i++)
    {
	//ROS_INFO("Index:%i -> Value:%i \n",i,reply[i]);
    GUI.Complete[i]=reply[i];
    }

   /*  timenow = ros::Time::now().toSec();*/

    ROS_INFO("Wheel 1:%i \n",GUI.INFO.position[0]);

    if(device.write(&command,sizeof(command))==1)
	{
	//printf("Command sent\n");
	}
	else
	{
	printf("Error in sending requested command\n");
	} 

    /// end dong modify
   /*  ROS_INFO("%f   n=%lf   r=%lf\r",tempangle,newangle,gyrocalrate);
    msg.data=newangle; */
    chatter_pub.publish(msg);

	ros::spinOnce();
	loop_rate.sleep();  
	}
  return 0;
}
