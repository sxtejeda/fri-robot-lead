#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <lead_rqt_plugins/RoomDialog.h>

//how long the robot waits before checking to see if the user is still present
#define CHECK_TIME 20

//How long the robot will wait before it determines that the user is no longer present
#define USER_TIMEOUT 10

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;
const int roomCount = 15;

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
    "3.436"
}; 
using namespace std;

int main(int argc, char **argv) {

  bool returning = false;
  ros::init(argc, argv, "leader");
  ros::NodeHandle n;
  ros::NodeHandle privateNode("~");

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

    //Prompt to see if the user is still following
    lead_rqt_plugins::RoomDialog userAlive;
    userAlive.request.type = userAlive.request.CHOICE_QUESTION;
    userAlive.request.message = "Are you still there?";
    userAlive.request.options.push_back("yes");
    userAlive.request.options.push_back("no");
    userAlive.request.timeout = USER_TIMEOUT;

    //The actual SimpleActionClient
    Client client("/action_executor/execute_plan", true);
    client.waitForServer();

    string door;

    if (client_gui.call(question)) {
 
      returning = false;
      if (question.response.index >= 0) {
        ROS_WARN("RESPONSE RECEIVED");
        switch (question.response.index) {
          case 42:
            ROS_WARN("And we're off!");
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

      //check and see if CHECK_TIME seconds have passed before asking the user
      //if he/she is still there.
      ros::Time prev = ros::Time::now();
      ros::Time checkup = prev + ros::Duration(CHECK_TIME);

      while (ros::ok() && !client.getState().isDone() && !returning) {
        wait_rate.sleep();
        ros::Duration elapsed = ros::Time::now() - prev;

        if (elapsed.toSec() > CHECK_TIME) {

          //Stopping the robot
          ros::spinOnce();
          client.cancelAllGoals();
          move_cancel_pub.publish(msg);
          ros::spinOnce();

          if(client_gui.call(userAlive)){
            switch(userAlive.response.index){
              case 0:
                ROS_INFO("User is still present, continue goal");
                client.sendGoal(goal);
                client_gui.call(moving);
                break;
              case 1:
                ROS_INFO("Manual Selection of no");
              default:
                ROS_INFO("User is no longer present");

                //Sending the new goal
                ros::spinOnce();
                client.cancelAllGoals();
                move_cancel_pub.publish(msg);
                client.sendGoal(home);
                ros::spinOnce();

                moving.request.message = "Returning to the lab";
                client_gui.call(moving);
                while(ros::ok() && !client.getState().isDone())
                	wait_rate.sleep();
                returning = true;
                break;
            }
          }
          prev = ros::Time::now();
          checkup = prev + ros::Duration(CHECK_TIME);
        }

      }

      if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Aborted");
      } else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_INFO("Preempted");
      } else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Succeeded!");
      } else
        ROS_INFO("Terminated");

    }

    else {
      ROS_ERROR("Failed to call service /room_dialog");
      return 1;
    }

  }

  return 0;
}


