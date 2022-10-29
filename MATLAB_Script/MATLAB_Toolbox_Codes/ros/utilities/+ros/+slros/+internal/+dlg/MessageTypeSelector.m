classdef MessageTypeSelector < ros.internal.dlg.MessageTypeSelector
%This class is for internal use only. It may be removed in the future.

%   Copyright 2014-2019 The MathWorks, Inc.

    methods (Access=protected)
        function setMessageList(obj)
            obj.MessageList = ros.slros.internal.ROSUtil.getMessageTypesWithoutServices;
            assert(numel(obj.MessageList) > 0, 'Expected non-empty message list');
        end
    end
end
