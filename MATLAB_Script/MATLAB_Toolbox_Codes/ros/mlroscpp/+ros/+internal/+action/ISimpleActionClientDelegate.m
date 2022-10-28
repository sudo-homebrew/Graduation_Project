classdef (Abstract) ISimpleActionClientDelegate < handle
%This class is for internal use only. It may be removed in the future.

%ISimpleActionClientDelegate Abstract interface for simple action client delegates
%   Having a common interface for all simple action client delegates
%   allows us to replace the delegate at test time and reduces external
%   dependencies of the system-under-test.
%
%   This design follows the Dependency Lookup / Component Broker pattern
%   described in http://xunitpatterns.com/Dependency%20Lookup.html.
%
%   See also ros.internal.action.ActionDelegateFactory

%   Copyright 2016-2020 The MathWorks, Inc.

    methods (Abstract)
        %checkGlobalNode Return global node object if input is empty
        node = checkGlobalNode(obj, node)

        %resolveNameType Resolve a given action name and action type
        [resolvedName, resolvedType] = resolveNameType(obj, node, actionNamespace, actionType, defaultActionType)

        %createActionClient Create simple action client for an action name and type
        actionClient = createActionClient(obj, node, actionNamespace, actionType)

        %createGoalMessage Create a goal message of the specified type
        goalMsg = createGoalMessage(obj, goalMsgType, dataFormat)

        %sendGoal Send a goal message to the action server
        sendGoal(obj, actionClient, goalMsg, clientCallbacks)

        %cancelGoal Cancel an active or pending goal (sent by this client)
        cancelGoal(obj, actionClient)

        %cancelAllGoals Cancel all pending and active goals on the server
        cancelAllGoals(obj, actionClient)

        %shutdown Shut down the action client
        shutdown(obj, actionClient)

        %isServerConnected Check if the action server is connected to the given client
        isConnected = isServerConnected(obj, actionClient)

        %getGoalState Get the state of the currently tracked goal
        [goalState, goalStatusText] = getGoalState(obj, actionClient)

        %waitForServer Wait until the action server is started up, or timeout occurs
        isServerConnected = waitForServer(obj, actionClient, timeout)
    end

end
