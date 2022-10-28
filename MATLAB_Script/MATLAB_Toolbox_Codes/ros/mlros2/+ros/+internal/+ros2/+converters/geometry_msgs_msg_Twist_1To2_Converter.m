function ros2msg = geometry_msgs_msg_Twist_1To2_Converter(message,ros2msg)
%geometry_msgs_msg_Twist_1To2_Converter passes data of ROS message to ROS 2 message.

% Copyright 2019 The MathWorks, Inc.
ros2msg.linear.x = message.Linear.X;
ros2msg.linear.y = message.Linear.Y;
ros2msg.linear.z = message.Linear.Z;
ros2msg.angular.x = message.Angular.X;
ros2msg.angular.y = message.Angular.Y;
ros2msg.angular.z = message.Angular.Z;
end