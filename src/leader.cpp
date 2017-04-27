#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <bwi_msgs/QuestionDialog.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

int main(int argc, char**argv) {
    ros::init(argc, argv, "leader");
    ros::NodeHandle n;

    Client client("/action_executor/execute_plan",true);
    client.waitForServer();

    ros::ServiceClient client_gui = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
    
    while(ros::ok()){
      bwi_msgs::QuestionDialog question;
      question.request.type = question.request.CHOICE_QUESTION;

      question.request.message = "Is 439 going to steal your soul?";
      question.request.options.push_back("yes");
      question.request.options.push_back("no");

      question.request.timeout = question.request.NO_TIMEOUT;

      if(client_gui.call(question)){
	if(question.response.index >= 0){
	  switch(question.response.index){
	  case 0:
	    question.request.message = "Urite";
	    break;
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
      return 0;
    }

    return 0;
}
