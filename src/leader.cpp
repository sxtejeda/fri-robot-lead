#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <bwi_msgs/QuestionDialog.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;
using namespace std;

int main(int argc, char**argv) {
    ros::init(argc, argv, "leader");
    ros::NodeHandle n;

    Client client("/action_executor/execute_plan",true);
    client.waitForServer();

    ros::ServiceClient client_gui = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");

    while(ros::ok()){
        bwi_msgs::QuestionDialog question;
        bwi_msgs::QuestionDialog userAlive;
        question.request.type = question.request.CHOICE_QUESTION;
        userAlive.request.type = userAlive.request.CHOICE_QUESTION;

        question.request.message = "Where would you like to go?";
        question.request.options.push_back("414");
        question.request.options.push_back("2nd door");

        question.request.timeout = question.request.NO_TIMEOUT;

        userAlive.request.message = "Are you still there?";
        userAlive.request.options.push_back("yes");
        userAlive.request.options.push_back("no");

        if(client_gui.call(question)){
            if(question.response.index >= 0){
                switch(question.response.index){
                    case 0: {
                        question.request.message = "Going to 414";
                        ros::NodeHandle privateNode("~");
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
        }
        else{
            ROS_ERROR("Failed to call service /question_dialog");
            return 1;
        }

        question.request.type = question.request.DISPLAY;
        question.request.timeout = 100000000000.000;
        client_gui.call(question);

    }

    return 0;
}
