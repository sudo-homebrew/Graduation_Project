classdef SimpleActionClient < ros.internal.mixin.ROSInternalAccess & ...
        ros.internal.DataFormatBase & ...
        robotics.core.internal.mixin.Unsaveable & handle
    %SimpleActionClient Create a simple ROS action client
    %   Use SimpleActionClient to connect to an action server and request the
    %   execution of action goals. You can get feedback on the execution
    %   progress and cancel the goal at any time. SimpleActionClient
    %   encapsulates a simple action client and allows you to track a
    %   single goal at a time.
    %
    %   CLIENT = ros.SimpleActionClient(NODE,ACTIONNAME) creates
    %   a CLIENT for the ROS action with name ACTIONNAME. NODE is the
    %   ros.Node object that is used to connect to the ROS
    %   network. If an action with this name is available in the
    %   ROS network, the client determines the action type automatically.
    %   If the action is not available, this function displays an error. CLIENT
    %   is a simple action client and allows you to track a single goal at a
    %   time. ACTIONNAME is a string scalar.
    %
    %   CLIENT = ros.SimpleActionClient(NODE,ACTIONNAME,ACTIONTYPE)
    %   creates an action client for the ROS action with name ACTIONNAME and
    %   type ACTIONTYPE. If no action with this name is available in the ROS network,
    %   it is created. If an action with the same name is available, but its
    %   type does not match ACTIONTYPE, the function displays an error.
    %   ACTIONTYPE is a string scalar.
    %
    %   CLIENT = ros.SimpleActionClient(___,Name,Value) provides additional
    %   options specified by one or more Name,Value pair arguments.
    %
    %      "DataFormat" - Determines format of ROS message to be used by
    %                     the action client, and returned from rosmessage.
    %                     Using structs can be faster than using message
    %                     objects.
    %                     Options are:
    %                        "object" - Message object of the specified type
    %                        "struct" - Message struct with compatible fields
    %                     Default: "object"
    %
    %
    %   SimpleActionClient properties:
    %      ActionName        - (Read-only) Name of action associated with this client
    %      ActionType        - (Read-only) Type of action
    %      DataFormat        - (Read-Only) Action message format required for use
    %      IsServerConnected - (Read-only) Indicates if client is connected to action server
    %      Goal              - (Read-only) The tracked goal
    %      GoalState         - (Read-only) State of the tracked goal
    %      ActivationFcn     - Function executed on goal activation
    %      FeedbackFcn       - Function executed for goal feedback
    %      ResultFcn         - Function executed for goal result
    %
    %   SimpleActionClient methods:
    %      waitForServer   - Wait for action server to start
    %      sendGoal        - Send goal message to action server
    %      sendGoalAndWait - Send goal message and wait for result
    %      cancelGoal      - Cancel last goal this client sent
    %      cancelAllGoals  - Cancel all goals on the server
    %      rosmessage      - Create goal message
    %
    %
    %   Example:
    %
    %      node = ros.Node("/turtlebot_client");
    %
    %      % Create action client for TurtleBot movement
    %      % The turtlebot_move action server needs to be running
    %      % Use struct message format for better performance
    %      turtleActClient = ros.SimpleActionClient(node,"/turtlebot_move",...
    %          "turtlebot_actions/TurtlebotMove","DataFormat","struct")
    %
    %      % Wait for the action server to start up
    %      waitForServer(turtleActClient)
    %
    %      % Request forward movement and wait until the TurtleBot is done
    %      goalMsg = rosmessage(turtleActClient);
    %      goalMsg.ForwardDistance = single(1.0);
    %      resultMsg = sendGoalAndWait(turtleActClient,goalMsg);
    %
    %      % Display the actual distance moved by the robot and the final goal state
    %      resultMsg.ForwardDistance
    %      turtleActClient.GoalState
    %
    %      % Set a custom callback for goal results. Disable feedback.
    %      turtleActClient.FeedbackFcn = [];
    %      turtleActClient.ResultFcn = @(~,res) fprintf("TurtleBot moved %.2f meters forward\n",res.Message.ForwardDistance);
    %
    %      % Send a goal to the server. This call will return immediately.
    %      sendGoal(turtleActClient,goalMsg)
    %
    %   See also ROSACTIONCLIENT, ROSACTION.

    %   Copyright 2016-2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %ActionName - Name of action associated with this client
        ActionName = ''

        %ActionType - Type of action associated with this client
        ActionType = ''

        %IsServerConnected - Indicates if client is connected to action server
        %   See also waitForServer.
        IsServerConnected = false

        %Goal - The tracked goal
        %   The simple action client tracks the execution of this goal. The
        %   client can only track a single goal at a time.
        Goal

        %GoalState - State of the tracked goal
        %   This reflects the state of the goal that is currently tracked
        %   by this action client (last goal message sent to the
        %   action server).
        %   Possible values are: 'pending', 'active', 'recalled', 'rejected',
        %   'preempted', 'aborted', 'succeeded', or 'lost'
        GoalState = ''
    end

    properties (Dependent)
        %ActivationFcn - Function executed on goal activation
        %   This function is called when the last sent goal is accepted by
        %   the action server and it transitions into the 'active' state.
        %
        %   The callback function should accept at least one input
        %   argument, CLIENT, which is the associated SimpleActionClient
        %   object. Additional input arguments can be provided with VARARGIN.
        %
        %   The function signature is as follows:
        %
        %      function activationFcn(CLIENT, VARARGIN)
        %
        ActivationFcn

        %FeedbackFcn - Function executed for goal feedback
        %   This function is called at a regular rate during execution of
        %   the last sent goal. The received feedback message contains
        %   information about the goal progress.
        %
        %   The callback function should accept at least two input
        %   argument. The first argument, CLIENT, is the associated SimpleActionClient
        %   object. The second argument, FBMSG, is the received feedback
        %   message. Additional input arguments can be provided with VARARGIN.
        %
        %   The function signature is as follows:
        %
        %      function feedbackFcn(CLIENT, FBMSG, VARARGIN)
        %
        %   You pass additional arguments to the callback function by including
        %   both the callback function and the arguments as elements of a cell array
        %   when setting the property.
        FeedbackFcn

        %ResultFcn - Function executed for goal result
        %   This function is called when the last sent goal finishes
        %   execution on the server and is in a final state. The received
        %   message contains information about the goal result.
        %
        %   The callback function should accept at least two input
        %   arguments. The first argument, CLIENT, is the associated SimpleActionClient
        %   object. The second argument, RESULT, is a structure with information
        %   about the goal result. Additional input arguments can be provided with VARARGIN.
        %   The RESULT structure contains the following fields:
        %   - Message    - Received result message
        %   - State      - Final goal state
        %   - StatusText - Status text associated with state
        %
        %   The function signature is as follows:
        %
        %      function resultFcn(CLIENT, RESULT, VARARGIN)
        %
        %   You pass additional arguments to the callback function by including
        %   both the callback function and the arguments as elements of a cell array
        %   when setting the property.
        ResultFcn
    end

    properties (Access = private)

        %InternalActivationFcn - Internal storage for the ActivationFcn callback
        InternalActivationFcn

        %InternalFeedbackFcn - Internal storage for the FeedbackFcn callback
        InternalFeedbackFcn

        %InternalResultFcn - Internal storage for the ResultFcn callback
        InternalResultFcn

        %GoalMessageType - The message type of the goal message (sent by client)
        GoalMessageType

        %FeedbackMessageType - The message type of the feedback message (sent by server)
        FeedbackMessageType

        %ResultMessageType - The message type of the result message (sent by server)
        ResultMessageType

        %ActualActivationFcn - Actual activation callback function after
        %parsing the user provided callback function and callback-data.
        ActualActivationFcn

        %ActivationFcnUserData - Callback-data provided by the user with
        %activation callback function.
        ActivationFcnUserData

        %ActualFeedbackFcn - Actual feedback callback function after
        %parsing the user provided callback function and callback-data.
        ActualFeedbackFcn

        %FeedbackFcnUserData - Callback-data provided by the user with
        %feedback callback function.
        FeedbackFcnUserData

        %ActualResultFcn - Actual result callback function after
        %parsing the user provided callback function and callback-data.
        ActualResultFcn

        %ResultFcnUserData - Callback-data provided by the user with
        %result callback function.
        ResultFcnUserData
    end

    properties (Access = ?matlab.unittest.TestCase)
        %Delegate - The delegate for most ROS-related functionality
        Delegate

        %Parser - Helper object for parsing tasks
        Parser
    end

    properties (Transient, Access = {?ros.internal.mixin.ROSInternalAccess,...
                            ?matlab.unittest.TestCase})
        %ActClientCallbackHandler - Helper to handle callbacks
        ActClientCallbackHandler = []

        %InternalNode - Internal representation of the node object
        %   Node required to get subscriber property information
        InternalNode = []

        %ServerNodeHandle - Designation of the node on the server
        %   Node handle required to get subscriber property information
        ServerNodeHandle = []

        %ActClientHandle - Designation of the service-server on the server
        %This is required to get property information.
        ActClientHandle = []

        %FeedbackInfo - Information about feedback message
        FeedbackInfo = struct.empty

        %ResultInfo - Information about result message
        ResultInfo = struct.empty

        %GoalDoneMutex - Condition for if the goal is completed or not.
        GoalDoneMutex = false;

        %IfAnyGoalSent - Flag to check if at least 1 goal has been sent to
        %    action-server or not after the action-client created.
        IfAnyGoalSent = false;

        %MaxConcurrentCallbacks - Number of callbacks allowed in queue.
        %   The concurrent callbacks limits the number of callbacks allowed
        %   on the main MATLAB thread, and is set to the recursion limit
        %   upon construction by default.
        MaxConcurrentCallbacks
    end

    methods
        function obj = SimpleActionClient(node, actionNamespace, varargin)
        %SimpleActionClient Create a ROS action client object
        %   Please see the class documentation
        %   (help ros.SimpleActionClient) for more details.

            narginchk(2,5);

            % Assign the delegates for most action client business logic
            delegateFactory = ros.internal.action.ActionDelegateFactory.getInstance;
            clientDelegate = delegateFactory.findDelegate('simpleClientDelegate');
            parser = delegateFactory.findDelegate('simpleClientParser');

            % Figure out if we need to use the global node or not
            node = clientDelegate.checkGlobalNode(node);

            % Parse the input arguments
            defaultActionType = '';
            [node, actionNamespace, actionType, dataFormat] = ...
                parser.parseConstructorInput(defaultActionType, node, actionNamespace, varargin{:});

            obj.Delegate = clientDelegate;
            obj.Parser = parser;

            [obj.ActionName, obj.ActionType] = obj.Delegate.resolveNameType( ...
                node, actionNamespace, actionType, defaultActionType);
            obj.GoalMessageType = [obj.ActionType 'Goal'];
            obj.FeedbackMessageType = [obj.ActionType 'Feedback'];
            obj.ResultMessageType = [obj.ActionType 'Result'];
            obj.FeedbackInfo = ...
                ros.internal.ros.getActionInfo(obj.FeedbackMessageType, ...
                                               obj.ActionType, ...
                                               'Feedback');
            obj.ResultInfo = ...
                ros.internal.ros.getActionInfo(obj.ResultMessageType, ...
                                               obj.ActionType, ...
                                               'Result');
            obj.setDataFormat(dataFormat)

            % Save the internal node information for later use
            obj.InternalNode = node.InternalNode; %ros.internal.Node
            obj.ServerNodeHandle = node.ServerNodeHandle; %node handle for server
            obj.MaxConcurrentCallbacks = get(0, 'RecursionLimit');

            % Initialize service client callback
            obj.ActClientCallbackHandler = ros.internal.ActClientCallbackHandler;
            obj.ActClientCallbackHandler.ActClientWeakHandle = matlab.internal.WeakHandle(obj);

            % Create CPP action client and initial goal message
            obj.ActClientHandle = obj.Delegate.createActionClient(obj.InternalNode, ...
                                                              obj.ServerNodeHandle, ...
                                                              obj.ActionName, ...
                                                              obj.ActionType, ...
                                                              obj.ActClientCallbackHandler, ...
                                                              obj.MaxConcurrentCallbacks);

            obj.Goal = rosmessage(obj);

            % Set default callback functions
            obj.FeedbackFcn = [];
            obj.ResultFcn = [];
            obj.ActivationFcn = [];

            obj.FeedbackFcn = obj.defaultFeedbackFcn;
            obj.ResultFcn = obj.defaultResultFcn;
            obj.ActivationFcn = obj.defaultActivationFcn;
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);
        end

        function delete(obj)
        %DELETE Shut down action client
        %   DELETE(CLIENT) shuts down the ROS action client object CLIENT.
        %   If the goal that was last sent to the action server is active,
        %   it is not cancelled.

        % Immediately halt callbacks
            obj.FeedbackFcn = [];
            obj.ResultFcn = [];
            obj.ActivationFcn = [];

            if ~isempty(obj.Delegate)
                obj.Delegate.shutdown(obj.InternalNode,obj.ActClientHandle);
            end
            obj.InternalNode = [];
        end

        function goalMsg = rosmessage(obj, varargin)
        %ROSMESSAGE Create goal message based on action type
        %   GOALMSG = ROSMESSAGE(CLIENT) creates and returns a new goal
        %   message GOALMSG. The message type of GOALMSG is determined
        %   by the action type associated with this action client. The
        %   format of MSG is determined by the DataFormat of the action
        %   client.
        %
        %   Example:
        %      % Create action client and goal message
        %      fibActClient = ros.SimpleActionClient(node,"/fibonacci");
        %      msgObj = ROSMESSAGE(fibActClient);
        %
        %      % Improve performance by using struct messages
        %      turtleActClient = ros.SimpleActionClient(node,"/turtlebot_move",...
        %          "DataFormat","struct");
        %      msgStruct = ROSMESSAGE(turtleActClient);

            validateDataFormatROSMessage(obj, varargin{:})

            goalMsg = obj.Delegate.createGoalMessage(obj.GoalMessageType, ...
                                                     obj.DataFormat);
        end

        function status = waitForServer(obj, varargin)
        %WAITFORSERVER Wait for action server to start
        %   WAITFORSERVER(CLIENT) blocks MATLAB from running the current program
        %   until the action server is started up and available to
        %   receive goals. Press Ctrl+C to abort the wait.
        %
        %   WAITFORSERVER(CLIENT,TIMEOUT) specifies a TIMEOUT period, in
        %   seconds. If the server does not start up in the timeout
        %   period, this function displays an error.
        %
        %   STATUS = WAITFORSERVER(____) returns a STATUS indicating
        %   whether the server is available. If the server is not available
        %   within the TIMEOUT, no error will be thrown and STATUS will be
        %   false.

            narginchk(1,2);

            defaultTimeout = Inf;
            timeout = obj.Parser.parseWaitForServerInput(defaultTimeout, varargin{:});
            
            % status - indicating whether this action client has been
            % connected to a valid action server.
            status = obj.Delegate.waitForServer(obj.InternalNode, ...
                                                     obj.ActClientHandle,timeout);
            if ~status && nargout<1
                error(message('ros:mlros:actionclient:WaitServerTimeout', num2str(timeout)));
            end
        end

        function sendGoal(obj, goalMsg)
        %sendGoal Send goal message to action server
        %   sendGoal(CLIENT,GOALMSG) sends a goal message, GOALMSG, to the
        %   action server. This goal is tracked by the action client. The
        %   function does not wait for the goal to be executed and returns
        %   immediately.
        %   If the ActivationFcn, FeedbackFcn, and ResultFcn callbacks
        %   are defined, they are called when the goal is processing on
        %   the action server.
        %   All callbacks associated with a previously sent goal are
        %   disabled, but the previous goal is not cancelled.
        %
        %   See also sendGoalAndWait, ResultFcn, FeedbackFcn, ActivationFcn.

        % Set to default values before sending the goal
            obj.GoalDoneMutex = false;
            obj.IfAnyGoalSent = true;

            try
                if obj.UseObjectMsg
                    goalMsgStruct = toROSStruct(goalMsg);
                else
                    goalMsgStruct = goalMsg;
                end
                obj.Delegate.sendGoal(obj.InternalNode, ...
                                      goalMsgStruct, ...
                                      obj.ActClientHandle);
                obj.Goal = goalMsg; % Only change property if valid message
            catch me
                obj.Parser.parseSendGoalInput(goalMsg, obj.GoalMessageType, obj.DataFormat)
                rethrow(me)
            end
        end

        function cancelGoal(obj)
        %cancelGoal Cancel last goal this client sent
        %   cancelGoal(CLIENT) sends a cancel request for the tracked goal
        %   (last sent to the action server).
        %   If the goal is in the "active" state, the server preempts
        %   its execution. If the goal is in the "pending" state, the goal
        %   is recalled.
        %   If no goal has been sent by this CLIENT or if the tracked goal
        %   has finished execution, this function returns immediately.

            obj.Delegate.cancelGoal(obj.InternalNode, obj.ActClientHandle);
        end

        function cancelAllGoals(obj)
        %cancelAllGoals Cancel all goals on the server
        %   cancelAllGoals(CLIENT) sends a request to the action server
        %   to cancel all currently pending or active goals. This
        %   includes goals from other action clients.

            obj.Delegate.cancelAllGoals(obj.InternalNode, obj.ActClientHandle);
        end

        function [resultMsg, state, statusText] = sendGoalAndWait(obj, goalMsg, varargin)
        %sendGoalAndWait Send goal message and wait for result
        %   RESULTMSG = sendGoalAndWait(CLIENT,GOALMSG) sends a goal message
        %   object, GOALMSG, to the action server and blocks MATLAB
        %   from running the current program until the action server
        %   returns the result, RESULTMSG.
        %   Press Ctrl+C to abort the wait.
        %
        %   RESULTMSG = sendGoalAndWait(CLIENT,GOALMSG,TIMEOUT) specifies
        %   a TIMEOUT period, in seconds. If the server does not return
        %   the result in the timeout period, the function displays an
        %   error.
        %
        %   [RESULTMSG, STATE, STATUSTEXT] = ____ also returns the
        %   final goal state, STATE, and the associated status text,
        %   STATUSTEXT. STATE contains information if the goal
        %   execution succeeded or not.
        %
        %   If the ActivationFcn, FeedbackFcn, and ResultFcn callbacks
        %   are defined, they are called when the goal is processing on
        %   the action server.

            narginchk(2,3)

            % Parse the inputs
            % Do input check up front since method is expected to take time
            defaultTimeout = Inf;
            timeout = obj.Parser.parseSendGoalAndWaitInput(defaultTimeout, obj.GoalMessageType, ...
                                                           obj.DataFormat, goalMsg, varargin{:});

            try
                % Send goal asynchronously. Return right away.
                sendGoal(obj, goalMsg)

                % Wait until the end of the timeout
                util = ros.internal.Util.getInstance;
                util.waitUntilTrue(@() obj.GoalDoneMutex, timeout);

                % Goal is done. Retrieve result and final state information.
                resultMsg = obj.Delegate.getResult(obj.InternalNode, obj.ActClientHandle);
                if obj.UseObjectMsg
                    resultMsg = feval(obj.ResultInfo.msgClassGen, resultMsg);
                end
                [state, statusText] = obj.Delegate.getGoalState(obj.InternalNode, obj.ActClientHandle);
            catch ex
                if ~strcmp(ex.identifier, 'ros:mlros:util:WaitTimeout')
                    % Rethrow exception if an unexpected exception is seen
                    rethrow(ex);
                end

                % Timeout occurred in waitUntilTrue. We have not received a
                % result yet. Cancel the goal.
                cancelGoal(obj);

                % Display error about timeout
                error(message('ros:mlros:actionclient:WaitGoalTimeout', num2str(timeout)));
            end
        end

    end

    %% Custom Getters and Setters for Properties
    methods
        function isConnected = get.IsServerConnected(obj)
            isConnected = obj.Delegate.isServerConnected(obj.InternalNode,...
                                                         obj.ActClientHandle);
        end

        function state = get.GoalState(obj)
            state = '';
            if obj.IfAnyGoalSent
                %If any goal is sent to action-server, get the state. If no
                %goal has been sent, no need to fetch the status from
                %backend.

                state = obj.Delegate.getGoalState(obj.InternalNode,obj.ActClientHandle);
            end
        end

        function feedbackFcn = get.FeedbackFcn(obj)
            feedbackFcn = obj.InternalFeedbackFcn;
        end

        function set.FeedbackFcn(obj, fcnHandle)
        %set.FeedbackFcn Setter for FeedbackFcn callback property

            if isempty(fcnHandle)
                % Allow empty input to disable the callback
                obj.InternalFeedbackFcn = [];
                obj.ActualFeedbackFcn = [];
                obj.FeedbackFcnUserData = [];
                return;
            end

            % Make sure this is a valid function handle and parse any user data
            [funcHandle, userData] = obj.Parser.parseFcnSetterInput(fcnHandle, 'FeedbackFcn');
            obj.ActualFeedbackFcn = funcHandle;
            obj.FeedbackFcnUserData = userData;
            obj.InternalFeedbackFcn = fcnHandle;
        end

        function resultFcn = get.ResultFcn(obj)
            resultFcn = obj.InternalResultFcn;
        end

        function set.ResultFcn(obj, fcnHandle)
        %set.ResultFcn Setter for ResultFcn callback property

            if isempty(fcnHandle)
                % Allow empty input to disable the callback
                obj.InternalResultFcn = [];
                obj.ActualResultFcn = [];
                obj.ResultFcnUserData = [];
                return;
            end

            % Make sure this is a valid function handle and parse any user data
            [funcHandle, userData] = obj.Parser.parseFcnSetterInput(fcnHandle, 'ResultFcn');
            obj.ActualResultFcn = funcHandle;
            obj.ResultFcnUserData = userData;
            obj.InternalResultFcn = fcnHandle;
        end

        function activationFcn = get.ActivationFcn(obj)
            activationFcn = obj.InternalActivationFcn;
        end

        function set.ActivationFcn(obj, fcnHandle)
        %set.ActivationFcn Setter for ActivationFcn callback property

            if isempty(fcnHandle)
                % Allow empty input to disable the callback
                obj.InternalActivationFcn = [];
                obj.ActualActivationFcn = [];
                obj.ActivationFcnUserData = [];
                return;
            end

            % Make sure this is a valid function handle and parse any user data
            [funcHandle, userData] = obj.Parser.parseFcnSetterInput(fcnHandle, 'ActivationFcn');
            obj.ActualActivationFcn = funcHandle;
            obj.ActivationFcnUserData = userData;
            obj.InternalActivationFcn = fcnHandle;
        end

        function processActivationCallback(obj, ~, info)
            if isequal(info.handle, obj.ActClientHandle) && ...
                    ~isempty(obj.ActualActivationFcn)
                feval(obj.ActualActivationFcn, ...
                      obj, ...
                      obj.ActivationFcnUserData{:});
            end
        end

        function processFeedbackCallback(obj, msg, info)
            if isequal(info.handle, obj.ActClientHandle) && ...
                    ~isempty(obj.ActualFeedbackFcn)
                if obj.UseObjectMsg
                    msg = feval(obj.FeedbackInfo.msgClassGen, msg);
                end
                feval(obj.ActualFeedbackFcn, ...
                      obj, ...
                      msg, ...
                      obj.FeedbackFcnUserData{:});
            end
        end

        function processResultCallback(obj, msg, info, state)
            if isequal(info.handle, obj.ActClientHandle)
                res.State = obj.Delegate.getGoalStateName(state.goal_state);
                if obj.UseObjectMsg
                    res.Message = feval(obj.ResultInfo.msgClassGen, msg);
                else
                    res.Message = msg;
                end

                % Goal is done. So update the mutex
                obj.GoalDoneMutex = true;

                if ~isempty(obj.ActualResultFcn)
                    feval(obj.ActualResultFcn, obj, res, obj.ResultFcnUserData{:});
                end
            end
        end
    end

    methods (Static, Access = ?matlab.unittest.TestCase)
        function fcn = defaultActivationFcn
        %defaultActivationFcn Default initialization for ActivationFcn
            fcn = @(~) disp('Goal active');
        end

        function fcn = defaultFeedbackFcn
        %defaultFeedbackFcn Default initialization for FeedbackFcn
            fcn = @(~,msg) disp(['Feedback: ', showDetailsAnyFormat(msg)]);
        end

        function fcn = defaultResultFcn
        %defaultResultFcn Default initialization for ResultFcn
            fcn = @(~,res) disp(['Final state ' res.State ' with result: ' ...
                                showDetailsAnyFormat(res.Message)]);
        end
    end

    methods(Access = private, Static)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.SimpleActionClient';
        end
    end
end

function dispString = showDetailsAnyFormat(msg)
%showDetailsAnyFormat Return display string for message of any DataFormat
%   Intended only for use with default callback functions

    if isa(msg, 'ros.Message')
        msg = toROSStruct(msg);
    end
    dispString = rosShowDetails(msg);
end
