#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <lead_rqt_plugins/RoomDialog.h>
#include "fri_robot_lead/PersonPresent.h"
#include <bwi_msgs/LogicalNavigationAction.h>

//Definitions for the status variable
#define APPROACHING 0
#define REQUESTING 1
#define ELEVATOR 2
#define OTHER 3

//How many seconds the robot will wait before it determines that the user is no longer present
#define USER_TIMEOUT 7

//How many seconds the robot will wait for the user to reappear before it returns to the lab
#define RETURN_THRESHOLD 10

//How many second the robot will wait to determine that a user is once again following it
#define SEEN_THRESHOLD 3

//How many seconds the robot will wait on a door request (not including elevator protocol) 
//before timing out and sending itself back to the lab
#define REQUEST_LIMIT 90

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

bool return_to_base, wait_for_person, potential_person_seen, resume_goal;
ros::Time last_message_time;
ros::Duration last_person_detected, time_seen, wait_time, request_time;
int status;



//The protocol for whether the robot should return to the base:
void detectorCallback(const fri_robot_lead::PersonPresent::ConstPtr &msg){
	//Do not run this method if the robot is engaged in elevator protocol
	if(status == ELEVATOR)
		return;
	
	ros::Time message_time = msg->timeStamp;
	ros::Duration time_elapsed = message_time - last_message_time;
	last_message_time = message_time;
	
	//If the robot has been waiting on a request for more than REQUEST_LIMIT seconds, return to the base
	if(status == REQUESTING){
		request_time += time_elapsed;
		ROS_INFO_STREAM("Leader:: waited for a request for " << request_time.toSec() << " seconds");
		if(request_time.toSec() > REQUEST_LIMIT)
			return_to_base = true;
	}
	//If the robot is moving
	else if(!wait_for_person){
		//If someone is visible, reset last_person_detected
		if(msg->personPresent){
			last_person_detected = ros::Duration(0.0);
		}
		//If no one has been visible for over USER_TIMEOUT seconds, stop moving
		else {
			last_person_detected += time_elapsed;
			ROS_INFO_STREAM("Leader:: person not detected for " << last_person_detected.toSec() << " seconds.");
	
			if(last_person_detected.toSec() > USER_TIMEOUT){
				wait_for_person = true;
				wait_time = ros::Duration(0.0);
			}
		}
	}
	//If the robot has stopped moving
	else {
		if(msg->personPresent){
			//If a person has been seen for over SEEN_THRESHOLD seconds, resume the original goal
			if(potential_person_seen){
				time_seen += time_elapsed;
				ROS_INFO_STREAM("Leader:: person seen for " << time_seen.toSec() << " seconds.");

				if(time_seen.toSec() > SEEN_THRESHOLD){
					wait_for_person = false;
					last_person_detected = ros::Duration(0.0);
					resume_goal = true;
				}
			}
			else {
				potential_person_seen = true;
				time_seen = ros::Duration(0.0);
			}	
		}
		else {
			//If no one has been seen for RETURN_THRESHOLD seconds, return to the base
			wait_time += time_elapsed;
			time_seen = ros::Duration(0.0);
			ROS_INFO_STREAM("Leader:: waited for a person for " << wait_time.toSec() << " seconds");	
			
			if(wait_time.toSec() > RETURN_THRESHOLD){
				return_to_base = true;
			}
		}	
	}

}

void logicalFeedbackCallback(const bwi_msgs::LogicalNavigationActionFeedback::ConstPtr &msg){
	int action = msg->feedback.action;
	int goal_status = msg->status.status;
	if(status == ELEVATOR){
		ROS_INFO_STREAM("Currently running elevator protocol..");
		if(action == bwi_msgs::LogicalNavigationFeedback::APPROACH_DOOR){ 
			std::size_t found = msg->feedback.data.find("elev");
			if(found == std::string::npos) {
				last_person_detected = ros::Duration(0.0);
				ROS_INFO_STREAM("Leader::Elevator protocol completed. Resuming person detection");
				status = APPROACHING;
			}
		}
		else
			return;
	}
	else if(action == bwi_msgs::LogicalNavigationFeedback::APPROACH_DOOR){
		std::size_t found = msg->feedback.data.find("elev");
		if(found != std::string::npos && goal_status == actionlib_msgs::GoalStatus::SUCCEEDED){
			status = ELEVATOR;
			ROS_INFO_STREAM("Leader:: Waiting for elevator, ignoring person detection");
		}
		else{
			status = APPROACHING;
		}
	}	
 else if(action == bwi_msgs::LogicalNavigationFeedback::APPROACH_OBJECT)
		status = APPROACHING;
	else if (action == bwi_msgs::LogicalNavigationFeedback::SENSE_DOOR && status != REQUESTING){
		request_time = ros::Duration(0.0);
		ROS_INFO_STREAM("Leader:: Detected non-approach action. Resetting wait time");
		status = REQUESTING;
	}
	else 
		status = OTHER;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "leader");
  ros::NodeHandle n;
  ros::NodeHandle privateNode("~");

	ros::Subscriber person_sub = n.subscribe("/person_present", 10, detectorCallback);
	ros::Subscriber logical_feedback_sub = n.subscribe("/execute_logical_goal/feedback", 10, logicalFeedbackCallback);


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

      return_to_base = wait_for_person = false;
			status = APPROACHING;
			last_person_detected = ros::Duration(0.0);

      while (ros::ok() ) {
        
				ros::spinOnce();
				
				if(resume_goal){
					wait_rate.sleep();
					client.sendGoal(goal);
					moving.request.message = "Resuming guidance";
					client_gui.call(moving);
					resume_goal = false;
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
					client.cancelGoal();
					move_cancel_pub.publish(msg);
				}
				else if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					//TODO: Have the robot ask if the user needs help getting anywhere else. If not, then
					//the robot should return home/enable some kind of roaming function.
					ROS_INFO_STREAM("Leader: Goal succeeded!");
					break;
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



