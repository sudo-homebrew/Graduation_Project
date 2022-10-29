classdef (Abstract) ISimpleActionClientParser < handle
%This class is for internal use only. It may be removed in the future.

%ISimpleActionClientParser Abstract interface for simple action client parser
%   Having a common interface allows us to replace the delegate at test
%   time and reduces external dependencies of the system-under-test.
%
%   This design follows the Dependency Lookup / Component Broker pattern
%   described in http://xunitpatterns.com/Dependency%20Lookup.html.
%
%   See also ros.internal.action.ActionDelegateFactory

%   Copyright 2016-2020 The MathWorks, Inc.

    methods (Abstract)
        %parseConstructorInput Parse constructor arguments
        [node, actionNamespace, actionType, dataFormat] = parseConstructorInput(~, defaultActionType, node, actionNamespace, varargin);

        %parseFcnSetterInput Parse argument to callback property setter
        [funcHandle, userData] = parseFcnSetterInput(~, fcnHandle, propName);

        %parseWaitForServerInput Parse the arguments to the waitForServer method
        timeout = parseWaitForServerInput(obj, defaultTimeout, varargin);

        %parseSendGoalInput Validate input to sendGoal method
        parseSendGoalInput(obj, goalMsg, expectedMsgType, dataFormat);

        %parseSendGoalAndWaitInput Parse the arguments to the sendGoalAndWait method
        timeout = parseSendGoalAndWaitInput(obj, defaultTimeout, expectedMsgType, dataFormat, goalMsg, varargin)
    end
end
