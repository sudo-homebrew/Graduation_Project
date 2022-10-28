classdef SimpleActionClientDelegate < ...
        ros.internal.action.ISimpleActionClientDelegate & ...
        handle
    %This class is for internal use only. It may be removed in the future.

    %SimpleActionClientDelegate Action client functionality requiring ROS connectivity
    %   This class encapsulates the majority of rosjava calls related to
    %   the simple action client. Most methods require a valid ROS network
    %   connection.
    %
    %   To facilitate testing, all ROS-specific functionality is delegated
    %   to this class. In unit tests, a mock object can act as a stand-in
    %   for this class.

    %   Copyright 2016-2021 The MathWorks, Inc.

    methods
        function node = checkGlobalNode(~, node)
        %checkGlobalNode Return global node object if input is empty
        %   This ensures standard semantics of formal ROS object (empty
        %   node input ensures that the global node is used).

            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end
        end

        function [resolvedName, resolvedType] = resolveNameType(~, ...
                                                              node, actionNamespace, actionType, defaultActionType)
            %resolveNameType Resolve a given action name and action type
            %   The action name is resolved based on the namespace of the
            %   connected ROS node.
            %   If the action type is not specified by the user, recover it
            %   from an available action.

            resolvedName = node.resolveName(actionNamespace);
            resolvedType = actionType;

            delegateFactory = ros.internal.action.ActionDelegateFactory.getInstance;
            actionIntro = delegateFactory.findDelegate('actionIntrospection', node);
            if strcmp(actionType, defaultActionType)
                % Recover action type, if it is not specified as input. This
                % will throw an error if the namespace is not a valid action namespace
                resolvedType = actionIntro.actionType(resolvedName);
                return;
            end

            % The user specified an action type. Make sure that the
            % user-specified action type does not conflict with the
            % existing action type
            try
                actualType = actionIntro.actionType(resolvedName);
            catch ex
                if ~strcmp(ex.identifier, 'ros:mlros:action:NotActionNamespace')
                    % Rethrow exception if an unexpected exception is seen
                    rethrow(ex);
                end

                % The resolvedType is fine
                return;
            end

            if ~strcmp(actualType, resolvedType)
                error(message('ros:mlros:action:ActionTypeNoMatch', resolvedName, ...
                              actualType, resolvedType));
            end
        end

        function actionClientHndl = createActionClient(~, internalNode, nodeHndl, actionNamespace, actionType, cbHndl, maxJobs)
        %createActionClient Create simple action client for an action name and type
            onGoalActiveCallbackFcn = 'onGoalActiveCallbackFcn';
            onFeedbackReceivedCallbackFcn = 'onFeedbackReceivedCallbackFcn';
            onResultReceivedCallbackFcn = 'onResultReceivedCallbackFcn';

            dllPathsGoal = ros.internal.utilities.getPathOfDependentDlls([actionType 'Goal'],'ros');
            dllPathsFeedback = ros.internal.utilities.getPathOfDependentDlls([actionType 'Feedback'],'ros');
            dllPathsResult = ros.internal.utilities.getPathOfDependentDlls([actionType 'Result'],'ros');
            dllPaths = [dllPathsGoal dllPathsFeedback dllPathsResult];
            try

                %get dll path and class-name of action client
                actInfo = ros.internal.ros.getActionInfo([actionType 'Goal'],actionType,'Goal','action');

                returnCall = addActClient(internalNode, ...
                                          nodeHndl, ...
                                          actionNamespace, ...
                                          actInfo.path, ...
                                          actInfo.cppFactoryClass, ...
                                          onGoalActiveCallbackFcn, ...
                                          onFeedbackReceivedCallbackFcn, ...
                                          onResultReceivedCallbackFcn, ...
                                          dllPaths);

                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlroscpp:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle)
                    error(message('ros:mlroscpp:node:InvalidReturnCallHandleError'))
                end
                actionClientHndl = returnCall.handle;

                % Initialize callback to process requests.
                initActClientCallback(internalNode, ...
                                      returnCall.handle, ...
                                      cbHndl, ...
                                      maxJobs);
                % No need to check reply - should error on failure
            catch ex
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:actionclient:CreateError', actionNamespace, actionType,ex.message));
                throw(newEx.addCustomCause(ex));
            end
        end


        function goalMsg = createGoalMessage(~, goalMsgType, dataFormat)
        %createGoalMessage Create a goal message of the specified type

            goalMsg = rosmessage(goalMsgType, 'DataFormat', dataFormat);
        end

        function sendGoal(~, internalNode, goalMsg, actClientHandle)
        %sendGoal Send a goal message to the action server

            sendGoalToActServer(internalNode, actClientHandle, goalMsg);
        end

        function cancelGoal(~, internalNode, actClientHandle)
        %cancelGoal Cancel an active or pending goal (sent by this client)

            cancelGoal(internalNode, actClientHandle);
        end

        function cancelAllGoals(~, internalNode, actClientHandle)
        %cancelAllGoals Cancel all pending and active goals on the server
        %   The call to cancelAllGoals will not throw any exceptions.

            cancelAllGoal(internalNode, actClientHandle);
        end

        function shutdown(~, internalNode, actClientHandle)
        % Cannot tell server to remove the service client without valid
        % internal node and server handle value

            if ~isempty(internalNode) && ...
                    isvalid(internalNode) && ...
                    ~isempty(actClientHandle)
                try
                    removeActClient(internalNode, actClientHandle);
                catch
                    warning(message('ros:mlros:actionclient:ShutdownError'));
                end
            end
        end

        function isConnected = isServerConnected(~, internalNode, actClientHandle)
        %isServerConnected Check if the action server is connected to the given client
        %   Returns TRUE if the server is connected, or FALSE otherwise.

            try
                if isempty(internalNode) || ...
                        ~isvalid(internalNode) || ...
                        isempty(actClientHandle)
                    isConnected = false;
                else
                    % check if server is connected
                    res = isActServerConnected(internalNode, actClientHandle);
                    isConnected = res.isServerConnected;
                end
            catch ex
                isConnected = false;
                warning(ex);
            end
        end

        function goalStateName = getGoalStateName(~, goalState)
        %getGoalStateName receives the numeric goalState and returns
        %   the name of the goalState

            if goalState < 0 || ...
                    goalState > 7 || ...
                    ~isnumeric(goalState)

                goalStateName = '';
                return;
            end
            goalStateNames = {'pending','active','recalled','rejected',...
                              'preempted','aborted','succeeded','lost'};
            goalStateName = goalStateNames{goalState+1};
        end

        function [goalState, goalStatusText] = getGoalState(obj, internalNode, actClientHandle)
        %getGoalState Get the state of the currently tracked goal
        %   As second output also return the status text associated
        %   with the state.

            goalState = '';
            %TO DO : g2184401 - cpp actionlib does not provide any status
            %text. So it is returned as blank. In mlros also it is always
            %coming as blank.
            goalStatusText = '';

            if ~isempty(internalNode) && ...
                    isvalid(internalNode) && ...
                    ~isempty(actClientHandle)

                % get the state.
                res = getActState(internalNode, actClientHandle);
                goalState = getGoalStateName(obj,res.state);
            end
        end

        function isServerConnected = waitForServer(obj, internalNode, actClientHandle, timeout)
        %waitForServer Wait until the action server is started up, or timeout occurs
        %   Return TRUE or FALSE if the action server is connected at
        %   the end of the function. If the server does not start up
        %   in TIMEOUT seconds, the return will be FALSE.

            isServerConnected = false;
            try
                if ~isempty(internalNode) && ...
                        isvalid(internalNode) && ...
                        ~isempty(actClientHandle)

                    if isequal(timeout,Inf)
                        %For infinite waiting, request backend again and
                        %again with 1sec timeout until user has cancelled
                        %the request.
                        util = ros.internal.Util.getInstance;
                        util.waitUntilTrue( @() waitForServer(obj,internalNode, ...
                                                              actClientHandle, 1), timeout);
                    else
                        waitForActServer(internalNode, actClientHandle, timeout);
                    end
                    isServerConnected = true;
                end
            catch ex
                if ~strcmp(ex.identifier, 'ros:internal:transport:ActClientError') && ...
                        ~strcmp(ex.identifier, 'ros:mlros:util:WaitTimeout')
                    % Rethrow exception if an unexpected exception is seen
                    rethrow(ex);
                end

                % Timeout occurred in waitUntilTrue
                % Continue execution.
            end
        end

        function result = getResult(~, internalNode, actClientHandle)
        %getResult gets the result of last goal sent to action server.

            if ~isempty(internalNode) && ...
                    isvalid(internalNode) && ...
                    ~isempty(actClientHandle)

                % get the result.
                result = getActResult(internalNode, actClientHandle);
            end
        end
    end
end
