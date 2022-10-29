function ros2msg = std_msgs_msg_String_1To2_Converter(message,ros2msg)
%std_msgs_msg_String_1To2_Converter passes data of ROS message to ROS 2 message.

% Copyright 2019 The MathWorks, Inc.
ros2msg.data = message.Data;
end