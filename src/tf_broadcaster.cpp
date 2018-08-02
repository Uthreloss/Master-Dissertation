#include "ros/ros.h" //Main headers for ROS System
#include "body_tracker_msgs/BodyTracker.h" //Node message type
#include <sstream>

ros::Subscriber body_subscriber; //Variable to call the node subscribed to the body tracking position publisher

body_tracker_msgs::BodyTracker Body_POS; //Global declaration of the variable that will contain the body position. The variable type is the same as the messages publish by the node astra_body_tracker_node

using namespace std; //To specify that cout and cin correspond to std if no namespace is given
/////////////////////////Method to print position of the hand////////////////////

void positionCallback(const body_tracker_msgs::BodyTracker::ConstPtr & position_message); //Function to update the variable that we use to keep track of the Publisher


int main(int argc, char **argv)
{
	//Initiate new ROS node named "body_data_handling"
	ros::init(argc,argv,"body_data_handling");
	ros::NodeHandle n; //Node reference

	ros::Subscriber body_subscriber = n.subscribe("/body_tracker/position", 10, positionCallback); //Subcriber declaration and configuration (Second argument is queue size)
  ros::spin(); //Loop to call messages as fast as possible
	return 0;

}




////////THIS FUNCTION IS CALLED WHEN A MESSAGE ARRIVES///////
void positionCallback(const body_tracker_msgs::BodyTracker::ConstPtr & position_message) {

	Body_POS.position3d.x= position_message->position3d.x; //The variable that contains publisher info is updated
	Body_POS.position3d.y= position_message->position3d.y;

	Body_POS.position3d.z= position_message->position3d.z;

	//Print the values received////
	 cout<<"POSITION OF YOUR BODY"<<endl;
	 cout<<"X coordinate"<<Body_POS.position3d.x << endl;
	 cout<<"Y coordinate"<<Body_POS.position3d.y << endl;
	 cout<<"Z coordinate"<<Body_POS.position3d.z << endl;

}
