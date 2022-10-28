function ros1msg = geometry_msgs_msg_Pose2D_2To1_Converter(message,ros1msg)
%geometry_msgs_msg_Pose2D_2To1_Converter passes data of ROS 2 message to ROS message.

% Copyright 2019 The MathWorks, Inc.    
ros1msg.X = message.x;
ros1msg.Y = message.y;
ros1msg.Theta = message.theta;
end