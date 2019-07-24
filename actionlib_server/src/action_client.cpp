#include <actionlib_server/MyMsgAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<actionlib_server::MyMsgAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "actionlib_client");
  Client client("action_server", true); //run client in its own thread
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  actionlib_server::MyMsgGoal goal;// send a goal to the action
  //TODO завязать клиент на move_base/simple_goal (msg: geometry_msgs/PoseStamped)
  goal.x = 1;
  goal.y = 1;
  goal.z = 0;
  client.sendGoal(goal);
  //wait for the action to return
  bool finished_before_timeout = client.waitForResult();
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  ROS_INFO("Result is: %d", client.getResult()->status);
  //exit
  return 0;
}
