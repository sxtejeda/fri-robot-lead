#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <bwi_msgs/QuestionDialog.h>

#define WAIT_TIME 20

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;
using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "leader");
  ros::NodeHandle n;
  ros::NodeHandle privateNode("~");

  ros::ServiceClient client_gui = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");

  while (ros::ok()) {
    bwi_msgs::QuestionDialog question;
    bwi_msgs::QuestionDialog moving;
    bwi_msgs::QuestionDialog userAlive;
    question.request.type = question.request.CHOICE_QUESTION;
    userAlive.request.type = userAlive.request.CHOICE_QUESTION;

    Client client("/action_executor/execute_plan", true);
    client.waitForServer();

    question.request.message = "Where would you like to go?";
    question.request.options.push_back("414");
    question.request.options.push_back("404");

    question.request.timeout = question.request.NO_TIMEOUT;

    userAlive.request.message = "Are you still there?";
    userAlive.request.options.push_back("yes");
    userAlive.request.options.push_back("no");

    moving.request.type = moving.request.DISPLAY;
    moving.request.timeout = 0;

    string door;

    if (client_gui.call(question)) {
      if (question.response.index >= 0) {
        switch (question.response.index) {
          case 0:
            privateNode.param<string>("door", door, "d3_414b1");
            moving.request.message = "Going to 414";
            break;
          case 1:
            privateNode.param<string>("door", door, "d3_404");
            moving.request.message = "Going to 404";
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
        ROS_INFO_STREAM(elapsed << " seconds have passed");
        if (elapsed.toSec() > WAIT_TIME) {
          ROS_INFO_STREAM(WAIT_TIME << " seconds have passed");
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

      /*    if(client_gui.call(question)){
        if(question.response.index >= 0){
    switch(question.response.index){
      case 0: {

        moving.request.message = "Going to 414";
        string door;
        privateNode.param<string>("door",door,"d3_414b1");
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
        while(ros::ok() && !client.getState().isDone()){
          wait_rate.sleep();
        }

        if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
          ROS_INFO("Aborted");
        }
        else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
          ROS_INFO("Preempted");
        }

        else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("Succeeded!");
        }
        else
          ROS_INFO("Terminated");

        break;
      }
      case 1:
        question.request.message = "You sure about that?";
        break;
      default:
        ROS_ERROR("Not a valid response");
        return 1;
    }
        }
        else {
    ROS_ERROR("How did you even end up here");
    }
    }*/
    else {
      ROS_ERROR("Failed to call service /question_dialog");
      return 1;
    }

    question.request.type = question.request.DISPLAY;
    question.request.timeout = 100000000000.000;
    client_gui.call(question);

  }

  return 0;
}


