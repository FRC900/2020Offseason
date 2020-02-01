#include "ros/ros.h"
#include "CAN_Error_Msg.h"
#include "hal/DriverStation.h"
#include "dummy_wpilib_hw.h"

void sendError(CAN_Error_Msg::Request  &req,
               CAN_Error_Msg::Response &res)
{
  res.isError = req.isError;
  res.errorCode = req.errorCode;
  res.isLVCode = req.isLVCode;
  res.details = req.details;
  res.location = req.location;
  res.callStack = req.callStack;
  res.printMsg = req.printMsg;

  HAL_SendError(res.isError,
		        res.errorCode,
				res.isLVCode,
			    res.details,
		        res.location,
		        res.callStack,
		        res.printMsg)
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CAN_Error_Server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("Send_CAN_Error", sendError);
  ROS_INFO("Ready to send error to DS");
  ros::spin();

  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CAN_Error_Client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<Send_CAN_Error>("Send_CAN_Error");
  Send_CAN_Error srv;
  srv.request.isError = atoll(argv[1]);
  srv.request.errorCode = atoll(argv[2]);
  srv.request.isLVCode = atoll(argv[3]);
  srv.request.details = atoll(argv[4]);
  srv.request.location = atoll(argv[5]);
  srv.request.callStack = atoll(argv[6]);
  srv.request.printMsg = atoll(argv[7]);
  if (client.call(srv))
  {
    ROS_INFO("Send ", (long int)srv.response.);
  }
  else
  {
    ROS_ERROR("Failed to call service Send_CAN_Error");
    return 1;
  }

  return 0;
}
