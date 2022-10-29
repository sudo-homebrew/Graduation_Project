function ros1msg = std_msgs_msg_String_2To1_Converter(message,ros1msg)
%std_msgs_msg_String_2To1_Converter passes data of ROS 2 message to ROS message.

% Copyright 2019 The MathWorks, Inc.    
ros1msg.Data = message.data{1};
end