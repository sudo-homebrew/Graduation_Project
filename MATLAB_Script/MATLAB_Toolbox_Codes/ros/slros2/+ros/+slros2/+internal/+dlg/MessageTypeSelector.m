classdef MessageTypeSelector < ros.internal.dlg.MessageTypeSelector
%This class is for internal use only. It may be removed in the future.

%   Copyright 2014-2019 The MathWorks, Inc.

    methods
        function obj = MessageTypeSelector()
            obj.Title =  message('ros:slros2:blockmask:DialogTitle').getString;
        end
    end
    
    methods (Access=protected)
        function setMessageList(obj)
            obj.MessageList = ros.slros2.internal.block.MessageBlockMask.getMessageList();
            assert(numel(obj.MessageList) > 0, 'Expected non-empty message list');
        end
    end
end
