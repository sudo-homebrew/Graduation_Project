classdef TopicSelector < ros.internal.dlg.TopicSelector
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties
        ROSMaster
    end
    
    methods
        function obj = TopicSelector()
            obj.ROSMaster = ros.slros.internal.sim.ROSMaster();
            obj.ROSMaster.verifyReachable();
        end
    end
    
    methods (Access=protected)
        function setTopicListAndTypes(obj)
            obj.ROSMaster.verifyReachable();
            [obj.TopicList, obj.TopicMsgTypes] = obj.ROSMaster.getTopicNamesTypes();
            if numel(obj.TopicList) < 1
                error(message('ros:slros:topicselector:NoTopicsAvailable', ...
                    obj.ROSMaster.MasterURI));
            end
        end
    end
end
