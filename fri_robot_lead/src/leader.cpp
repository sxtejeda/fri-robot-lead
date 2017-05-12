#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
// #include <bwi_msgs/QuestionDialog.h>
#include <lead_rqt_plugins/RoomDialog.h>

#define WAIT_TIME 20

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;
const int roomCount = 8;
const std::string rooms[] = {
    "1.416",
    "2.302_south",
    "2.700",
    "3.414a1",
    "3.404",
    "3.500",
    "4.414a1",
    "4.600"
}; 
using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "leader");
  ros::NodeHandle n;
  ros::NodeHandle privateNode("~");

  //These are used to actually cancel goals
  ros::Publisher move_cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1000);
  actionlib_msgs::GoalID msg;
  ros::service::waitForService("/room_dialog");
  ros::ServiceClient client_gui = n.serviceClient<lead_rqt_plugins::RoomDialog>("/room_dialog");

  lead_rqt_plugins::RoomDialog question;
  question.request.type = question.request.COMBOBOX_QUESTION;
  question.request.message = "Where would you like to go?";

  for (int i = 0; i < roomCount; i++) {
    std::string room = rooms[i];
    question.request.options.push_back(room);
  }
  while (ros::ok()) {
    
    lead_rqt_plugins::RoomDialog moving;
    lead_rqt_plugins::RoomDialog userAlive;
    userAlive.request.type = userAlive.request.CHOICE_QUESTION;

    Client client("/action_executor/execute_plan", true);
    client.waitForServer();

    //question.request.options.push_back("414");
    //question.request.options.push_back("404");

    question.request.timeout = question.request.NO_TIMEOUT;

    userAlive.request.message = "Are you still there?";
    userAlive.request.options.push_back("yes");
    userAlive.request.options.push_back("no");

    moving.request.type = moving.request.DISPLAY;
    moving.request.timeout = 0;

    string door;

    if (client_gui.call(question)) {
      //move_cancel_pub.publish(msg);
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

      bwi_kr_execution::ExecutePlanGoal goal;

      //temp is made for checking up on the user
      bwi_kr_execution::ExecutePlanGoal temp;
      bwi_kr_execution::AspRule rule;
      bwi_kr_execution::AspFluent fluent;
      fluent.name = "not facing";

      fluent.variables.push_back(door);

      rule.body.push_back(fluent);
      goal.aspGoal.push_back(rule);
      temp.aspGoal.push_back(rule);

      ROS_INFO("sending goal");
      client.sendGoal(goal);

      client_gui.call(moving);
      ros::Rate wait_rate(10);

      //For timing, check and see if WAIT_TIME seconds have passed before asking the user
      //if he/she is still there.
      ros::Time prev = ros::Time::now();
      ros::Time checkup = prev + ros::Duration(WAIT_TIME);

      while (ros::ok() && !client.getState().isDone()) {
        wait_rate.sleep();
        ros::Duration elapsed = ros::Time::now() - prev;

        //Here's an interesting problem. Without this printing out, the timing is extremely
        //inaccurate due to the system not having anything to do. I wonder if there's a
        //better solution for this? Doesn't seem like there is one, and even with the printing
        //to choke the system it really is quite terrible.
        //ROS_INFO_STREAM(elapsed << " seconds have passed");
        if (elapsed.toSec() > WAIT_TIME) {

          ros::spinOnce();
          client.cancelAllGoals();
          move_cancel_pub.publish(msg);
          ros::spinOnce();

          if(client_gui.call(userAlive)){
            switch(userAlive.response.index){
              case 0:
                ROS_INFO("Case 0 selected");
                client.sendGoal(goal);
                client_gui.call(moving);
                break;
              case 1: //yes
                ROS_INFO("Case 1 selected");
                break;
              default:
                ROS_INFO("No response maybe? Not sure...");
                break;
            }
          }
          prev = ros::Time::now();
          checkup = prev + ros::Duration(WAIT_TIME);
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

    question.request.type = question.request.DISPLAY;
    question.request.timeout = 100000000000.000;
    client_gui.call(question);

  }

  return 0;
}


