function ros1msg = geometry_msgs_msg_Vector3_2To1_Converter(message,ros1msg)
%geometry_msgs_msg_Vector3_2To1_Converter passes data of ROS 2 message to ROS message.

% Copyright 2019 The MathWorks, Inc.    
ros1msg.X = message.x;
ros1msg.Y = message.y;
ros1msg.Z = message.z;
end