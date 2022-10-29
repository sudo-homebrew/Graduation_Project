function ros1msg = std_msgs_msg_Float32_2To1_Converter(message,ros1msg)
%std_msgs_msg_Float32_2To1_Converter passes data of ROS 2 message to ROS message.

% Copyright 2019 The MathWorks, Inc.    
ros1msg.Data = message.data;
end