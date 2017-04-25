//#include "bwi_kr_execution/ExecutePlanAction.h"

//#include <bwi_msgs/QuestionDialog.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>



//typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

int main(int argc, char**argv) {
    ros::init(argc, argv, "leader");
  //  ros::NodeHandle n;

//    ros::NodeHandle privateNode("~");

    //Client client("/action_executor/execute_plan",true);
    //client.waitForServer();

    //ros::ServiceClient client_gui = n.serviceClient<bwi_msgs::QuestionDialog>("/question_dialog");
}