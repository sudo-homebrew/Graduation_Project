function ros2msg = geometry_msgs_msg_Vector3_1To2_Converter(message,ros2msg)
%geometry_msgs_msg_Vector3_1To2_Converter passes data of ROS message to ROS 2 message.

% Copyright 2019 The MathWorks, Inc.
ros2msg.x = message.X;
ros2msg.y = message.Y;
ros2msg.z = message.Z;
end