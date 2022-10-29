classdef ActClientCallbackHandler < handle
%This class is for internal use only. It may be removed in the future.

%ActClientCallbackHandler is helper to process the callbacks. This is
%   entry gate-way for the callbacks from action-client which comes
%   asynchronously from back-end.

%   Copyright 2020 The MathWorks, Inc.

    properties (Transient, Access = ?ros.internal.mixin.ROSInternalAccess)
        %service-client Handle.
        ActClientWeakHandle
    end

    methods
        function onGoalActiveCallbackFcn(obj, msg, info)
            processActivationCallback(obj.ActClientWeakHandle.get, msg, info);
        end
        
        function onFeedbackReceivedCallbackFcn(obj, msg, info)
            processFeedbackCallback(obj.ActClientWeakHandle.get, msg, info);
        end
        
        function onResultReceivedCallbackFcn(obj, msg, info, state)
            processResultCallback(obj.ActClientWeakHandle.get, msg, info, state);
        end
    end
end
