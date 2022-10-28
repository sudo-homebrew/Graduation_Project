classdef GetMessageTopicList
    %GetMessageTopicList Get the list of topic and its message type

%   Copyright 2019 The MathWorks, Inc.
    
    properties
        TopicList = {'/my_topic','/first_topic','/second_topic'}
        MsgList = {'geometry_msgs/Pose','geometry_msgs/Point','geometry_msgs/Quaternion'} 
    end
 
end

