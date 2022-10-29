classdef TopicSelector < ros.internal.dlg.TopicSelector
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2019 The MathWorks, Inc.
    
    methods (Access=protected)
        function setTopicListAndTypes(obj)
            [obj.TopicList, obj.TopicMsgTypes] = ros.ros2.internal.NetworkIntrospection.getTopicNamesTypes();
            if numel(obj.TopicList) < 1
                error(message('ros:slros2:blockmask:NoTopicsAvailableError'));
            end
        end
    end
end
