function ros1msg = geometry_msgs_msg_Twist_2To1_Converter(message,ros1msg)
%geometry_msgs_msg_Twist_2To1_Converter passes data of ROS 2 message to ROS message.

% Copyright 2019 The MathWorks, Inc.    
ros1msg.Linear.X = message.linear.x;
ros1msg.Linear.Y = message.linear.y;
ros1msg.Linear.Z = message.linear.z;
ros1msg.Angular.X = message.angular.x;
ros1msg.Angular.Y = message.angular.y;
ros1msg.Angular.Z = message.angular.z;
end