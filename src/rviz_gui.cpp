#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

ros::Publisher stop_pub;

void Callback(const sensor_msgs::Joy & msg) {
	if (msg.buttons[2] == 1) 
	{	
		std_msgs::Bool b;
		b.data = false;
   		stop_pub.publish(b);
		ROS_INFO("Started recording");
	} 
	if (msg.buttons[4] == 1) 
	{	
		std_msgs::Bool b;
		b.data = true;
   		stop_pub.publish(b);
		ROS_INFO("Recording stopped");
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rviz_gui");
	ros::NodeHandle node;
    ros::Subscriber rviz_gui_sub;
    rviz_gui_sub = node.subscribe("/rviz_visual_tools_gui", 1, Callback);   
    stop_pub = node.advertise<std_msgs::Bool>("/stop", 1, true);

    ros::spin();
    return 0;
}
