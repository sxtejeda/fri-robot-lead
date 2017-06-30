#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <lead_rqt_plugins/RoomDialog.h>
#include "fri_robot_lead/PersonPresent.h"

//how long the robot waits before checking to see if the user is still present
#define CHECK_TIME 20

//How long the robot will wait before it determines that the user is no longer present
#define USER_TIMEOUT 10
#define RETURN_THRESHOLD 3

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;
const int roomCount = 16;

//Current list of rooms on the third floor - to add a room, simply follow the format of
//{floor number}.{room number}
//Room must exist within the bwi_kr_execution domain in order for the planner to get to it
const std::string rooms[] = {
    "3.400",
    "3.404",
    "3.508",
    "3.414a1",
    "3.414a2",
    "3.414a3",
    "3.416",
    "3.418",
    "3.510",
    "3.512",
    "3.414b1",
    "3.414b2",
    "3.414b3",
    "3.432",
    "3.436",
		"2.300"
}; 
using namespace std;

bool return_to_base, wait_for_person;
ros::Time last_person_detected;

void detectorCallback(const fri_robot_lead::PersonPresent::ConstPtr &msg){
	ROS_INFO_STREAM("Leader.cpp: callback has been called");

	if(msg->personPresent){
		last_person_detected = msg->timeStamp;
		wait_for_person = false;
	}
	else {
		ros::Duration time_since_detection = ros::Time::now() - last_person_detected;
		ROS_INFO_STREAM("Leader.cpp: person not detected. Checking time threshold...");
		ROS_INFO_STREAM("Time since last person detected: " << time_since_detection.toSec() << " seconds.");
		
		if(wait_for_person && time_since_detection.toSec() > RETURN_THRESHOLD)
			return_to_base = true;		
		else if(time_since_detection.toSec() > USER_TIMEOUT)
			wait_for_person = true;
	}
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "leader");
  ros::NodeHandle n;
  ros::NodeHandle privateNode("~");

  return_to_base = false;
	last_person_detected = ros::Time::now();
	ros::Subscriber sub = n.subscribe("/person_present", 10, detectorCallback);

  //Empty message, used to stop the robot
  ros::Publisher move_cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1000);
  actionlib_msgs::GoalID msg;

  //Custom service with dropdown menu containing all the rooms defined in rooms array above
  ros::service::waitForService("/room_dialog");
  ros::ServiceClient client_gui = n.serviceClient<lead_rqt_plugins::RoomDialog>("/room_dialog");

  //Where the robot returns if there is no longer a user present
  bwi_kr_execution::ExecutePlanGoal home;
  bwi_kr_execution::AspRule home_rule;
  bwi_kr_execution::AspFluent home_fluent;
  home_fluent.name = "not facing";
  //Change the line below to change where the 'home base' is
  home_fluent.variables.push_back("d3_414b1");
  home_rule.body.push_back(home_fluent);
  home.aspGoal.push_back(home_rule);


  lead_rqt_plugins::RoomDialog question;
 
  while (ros::ok()) {

    question.request.type = question.request.COMBOBOX_QUESTION;
    question.request.message = "Where would you like to go?";
    question.request.timeout = question.request.NO_TIMEOUT;

    for (int i = 0; i < roomCount; i++) {
      std::string room = rooms[i];
      question.request.options.push_back(room);
    }
    //Moving message for the GUI
    lead_rqt_plugins::RoomDialog moving;
    moving.request.type = moving.request.DISPLAY;
    moving.request.timeout = 0;

   //The actual SimpleActionClient
    Client client("/action_executor/execute_plan", true);
    client.waitForServer();

    string door;

    if (client_gui.call(question)) {
 
      return_to_base = false;
			wait_for_person = false;
      if (question.response.index >= 0) {
        ROS_WARN("RESPONSE RECEIVED");
        switch (question.response.index) {
          case 42:
            ROS_INFO("Sending goal");
            privateNode.param<string>("door", door, question.response.text);
            moving.request.message = "Going to " + question.response.text;
            break;
          default:
            ROS_ERROR("Invalid input for buttons");
            break;
        }
      } else {
        ROS_ERROR("Should not be possible to get here.");
      }

      //Creation of goal
      bwi_kr_execution::ExecutePlanGoal goal;
      bwi_kr_execution::AspRule rule;
      bwi_kr_execution::AspFluent fluent;
      fluent.name = "not facing";
      fluent.variables.push_back(door);
      rule.body.push_back(fluent);
      goal.aspGoal.push_back(rule);

      ROS_INFO("sending goal");
      client.sendGoal(goal);

      client_gui.call(moving);
      ros::Rate wait_rate(10);

			//Protocol for person detection:
			//If USER_TIMEOUT seconds have passed and the robot has not seen anyone, the current guide goal
			//Is cancelled. 
			//Then, the robot will wait RETURN_THRESHOLD seconds for a person to reappear. If no one shows up,
			//the robot will return to the lab.
   	  last_person_detected = ros::Time::now();	
      while (ros::ok() ) {
          ros::spinOnce();
					
					if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
						//TODO: Have the robot ask if the user needs help getting anywhere else. If not, then
						//the robot should return home/enable some kind of roaming function.
						ROS_INFO_STREAM("Leader: Goal succeeded!");
						break;
					}					
					else if(return_to_base) {
						client.sendGoal(home);
						moving.request.message = "Returning to the lab";
						client_gui.call(moving);
						while(ros::ok() && !client.getState().isDone())
							wait_rate.sleep();
						break;
					}
					else if(wait_for_person) {
						ROS_INFO_STREAM("Leader: Person no longer present, cancelling goal");
						client.cancelGoal();
						move_cancel_pub.publish(msg);
					}
					else if(client.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
						ROS_ERROR_STREAM("Leader: goal has been preempted by an external force");
						break;
					}
					else if(client.getState() == actionlib::SimpleClientGoalState::ABORTED){
						ROS_ERROR_STREAM("Leader: goal has been aborted");
					}
      }

    }

    else {
      ROS_ERROR("Failed to call service /room_dialog");
      return 1;
    }

  }

  return 0;
}


