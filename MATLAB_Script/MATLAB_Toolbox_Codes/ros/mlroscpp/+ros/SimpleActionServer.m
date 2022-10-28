classdef SimpleActionServer < ros.internal.mixin.ROSInternalAccess & ...
        ros.internal.DataFormatBase & ...
        robotics.core.internal.mixin.Unsaveable & handle
%SimpleActionServer Create a simple ROS action server
%   Use SimpleActionServer to create a ROS action server to execute goals
%   provided by action clients. The server can provide periodic feedback on
%   execution progress to the clients, and abort goal execution if desired
%   or if a new goal is received. SimpleActionServer encapsulates a ROS
%   simple action server, which can execute a single goal at a time.
%
%   When you create the action server, it registers itself with the ROS
%   master. To get a list of actions that are available on the current ROS
%   network, or to get more information about any particular action, use
%   the rosaction function.
%
%   The action is defined by a type and three messages: one for the goal,
%   one for the feedback, and one for the result. The action server will
%   receive a goal, construct and send feedback during goal execution, and
%   construct and send an appropriate result when goal execution completes.
%   The behavior of the action server is inherently asynchronous, as it
%   only becomes active when an action client connects and sends a goal.
%
%   SERVER = ros.SimpleActionServer(NODE,ACTIONNAME,ACTIONTYPE,ExecuteGoalFcn=CB)
%   creates and returns an action server object. NODE is the ros.Node
%   object that is used to connect to the ROS network. The action will be
%   available on the ROS network through its name ACTIONNAME and has the
%   type ACTIONTYPE. It also specifies the function handle callback, CB,
%   that handles the goal execution, feedback communication, and result
%   creation. CB can be a single function handle or a cell array. The first
%   element of the cell array must be a function handle, or a string
%   containing the name of a function. The remaining elements of the cell
%   array can be arbitrary user data that is passed to the callback
%   function.
%
%   SERVER = ros.SimpleActionServer(___,Name=Value) provides additional
%   options specified by one or more Name-Value pair arguments.
%
%      "DataFormat" - Determines format of ROS message provided to the
%                     ExecuteGoalFcn callback.
%                     Using structs can be faster than using message
%                     objects.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%   The action server callback function requires at least four input
%   arguments and two outputs. The first argument, SRC, is the associated
%   action server object. The second argument, GOAL, is the goal message
%   sent by the action client. The third argument, DEFAULTFEEDBACK, is the
%   default feedback message for that action type. The fourth argument,
%   DEFAULTRESULT, is the default result message for that action type.
%   Use the DEFAULTFEEDBACK message as a starting point for constructing
%   the feedback messages to be passed to sendFeedback within the callback.
%   Use the DEFAULTRESULT message as a starting point for constructing the
%   function output RESULT, which will be sent back to the action client
%   after the function returns. Define the second function output, SUCCESS,
%   as true if the goal was successfully reached, or as false if the goal
%   was aborted or preempted by another goal. Here is a generic structure
%   for the action server callback:
%
%      function [RESULT, SUCCESS] = actionCallback(SRC,GOAL,DEFAULTFEEDBACK,DEFAULTRESULT)
%          % Periodically check if goal is canceled or new goal available
%          if isPreemptRequested(SRC)
%              SUCCESS = false;
%              RESULT = DEFAULTRESULT;
%              % Build the result message in the case of preemption here
%              return
%          else
%              % Periodically send feedback to the client
%              FEEDBACK = DEFAULTFEEDBACK;
%              % Build the feedback message here
%              sendFeedback(SRC, FEEDBACK)
%          end
%          % If the goal is reached
%          SUCCESS = true;
%          RESULT = DEFAULTRESULT;
%          % Build the result message in the case of success here
%      end
%
%   To construct a callback that accepts additional parameters, use a cell
%   array that includes both the function handle callback and the parameters.
%
%   To get a predefined callback framework of short functions, consider
%   using rosActionServerExecuteGoalFcn.
%
%
%   SimpleActionServer properties:
%      ActionName     - (Read-Only) The name of the action
%      ActionType     - (Read-Only) The type of the action
%      DataFormat     - (Read-Only) Message format provided by the server
%      ExecuteGoalFcn - Callback property for goal execution
%
%   SimpleActionServer methods:
%      rosmessage         - Create a new result message
%      getFeedbackMessage - Create a new feedback message
%      isPreemptRequested - Check if the current goal is canceled or
%                           if a new goal is available
%      sendFeedback       - Send feedback message to action client
%
%
%   Example:
%
%      % Create or connect to ROS network and create node
%      core = ros.Core;
%      node = ros.Node("/action_node");
%
%      % Set up action server for calculating Fibonacci sequence
%      % Use struct messages for better performance
%      cb = @fibonacciExecution;
%      server = ros.SimpleActionServer(node,"/fibonacci",...
%          "actionlib_tutorials/Fibonacci",ExecuteGoalFcn=cb,...
%          DataFormat="struct");
%
%      % Create action client and send a goal to the server
%      client = ros.SimpleActionClient(node,"/fibonacci",...
%          "actionlib_tutorials/Fibonacci",DataFormat="struct");
%      goal = rosmessage(client);
%      goal.Order = int32(10);
%      result = sendGoalAndWait(client,goal);
%
%      % Define callback function for action server
%      function [result, success] = fibonacciExecution(src,goal,defaultFeedback,defaultResult)
%          success = true;
%          result = defaultResult;
%          feedback = defaultFeedback;
%          feedback.Sequence = int32([0 1]);
%          for k = 1:goal.Order
%              % Check that the client has not canceled or sent a new goal
%              if isPreemptRequested(src)
%                  success = false;
%                  break
%              end
%
%              % Periodically send feedback to the client
%              feedback.Sequence(end+1) = feedback.Sequence(end-1) + feedback.Sequence(end);
%              sendFeedback(src,feedback)
%
%              % Pause to allow other callbacks (like client feedback) time to complete
%              pause(0.2)
%          end
%
%          if success
%              result.Sequence = feedback.Sequence;
%          end
%      end
%
%
%   See also ROSACTIONSERVER, rosActionServerExecuteGoalFcn, ros.SimpleActionClient, ROSACTION.

%   Copyright 2021 The MathWorks, Inc.

    properties (SetAccess = private)
        %ActionName - Name of action associated with this server
        ActionName = ''

        %ActionType - Type of action associated with this server
        ActionType = ''
    end

    properties
        %ExecuteGoalFcn - Callback property for new goal execution
        ExecuteGoalFcn
    end

    properties (Access = ?ros.internal.mixin.ROSInternalAccess)
        %GoalMessageType - The message type of the goal message (sent by client)
        GoalMessageType

        %FeedbackMessageType - The message type of the feedback message (sent by server)
        FeedbackMessageType

        %ResultMessageType - The message type of the result message (sent by server)
        ResultMessageType

        %ActualExecuteGoalFcn - Actual activation callback function after
        %parsing the user provided callback function and callback-data
        ActualExecuteGoalFcn

        %ExecuteGoalFcnUserData - Callback-data provided by the user with
        %execute-goal callback function
        ExecuteGoalFcnUserData

        %Parser - Helper object for parsing tasks
        Parser
    end

    properties (Transient, Access = ?ros.internal.mixin.ROSInternalAccess)
        %ExecuteGoalCallbackHandler - Helper to handle callbacks
        ExecuteGoalCallbackHandler = []

        %InternalNode - Internal representation of the node object
        %   Node required to get property information
        InternalNode = []

        %ServerNodeHandle - Designation of the node on the server
        %   Node handle required to get property information
        ServerNodeHandle = []

        %ActionServerHandle - Designation of the action server on the server
        %This is required to get property information
        ActionServerHandle = []

        %ActionInfo - includes other information for a given action
        ActionInfo = struct.empty

        %MaxConcurrentCallbacks - Number of callbacks allowed in queue.
        %   The concurrent callbacks limits the number of callbacks allowed
        %   on the main MATLAB thread, and is set to the recursion limit
        %   upon construction by default
        MaxConcurrentCallbacks
    end

    properties (Constant, Access = ?ros.internal.mixin.ROSInternalAccess)
        % Goal status text to send back to client with corresponding method
        GoalStatusPreempted = 'preempted'
        GoalStatusAborted = 'aborted'
        GoalStatusSucceeded = 'succeeded'
    end

    methods
        function obj = SimpleActionServer(node, actionName, actionType, varargin)
        %SimpleActionServer Create a ROS action server object
        %   Please see the class documentation
        %   (help ros.SimpleActionServer) for more details.

        % If no node specified, use the global node.
            if isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Parse the inputs to the constructor
            [actionName, actionType, varargin{:}] = ...
                convertStringsToChars(actionName, actionType, varargin{:});
            parser = getParser(obj);
            parse(parser, node, actionName, actionType, varargin{:})
            node = parser.Results.node;
            resolvedName = resolveName(node, parser.Results.actionName);
            actionType = parser.Results.actionType;
            obj.ExecuteGoalFcn = parser.Results.ExecuteGoalFcn;

            % Make sure that action does not exist yet
            actionIntrospect = ros.internal.action.ActionIntrospection(node);
            listOfActions = actionList(actionIntrospect);
            if ismember(resolvedName, listOfActions)
                error(message('ros:mlroscpp:actionserver:AlreadyExists', resolvedName));
            end

            % Make sure that action type is valid
            actionTypes = rostype.getActionList;
            if ~ismember(actionType, actionTypes)
                error(message('ros:mlroscpp:actionserver:InvalidType', actionType));
            end

            % Set some object properties
            obj.ActionType = actionType;
            obj.GoalMessageType = [actionType 'Goal'];
            obj.FeedbackMessageType = [actionType 'Feedback'];
            obj.ResultMessageType = [actionType 'Result'];
            setDataFormat(obj, parser.Results.DataFormat)

            % Save the internal node information for later use
            obj.InternalNode = node.InternalNode;           % ros.internal.Node
            obj.ServerNodeHandle = node.ServerNodeHandle;   % Out of process server
            obj.MaxConcurrentCallbacks = get(0, 'RecursionLimit');

            % Get action info
            obj.ActionInfo = ros.internal.ros.getActionInfo(obj.GoalMessageType, ...
                                                            obj.ActionType, ...
                                                            'Goal');

            % Create callback handler object
            obj.ExecuteGoalCallbackHandler = ...
                ros.internal.CallbackHandler(matlab.internal.WeakHandle(obj), @executeNextGoal);

            % Store the service server handle in node object
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);

            % Create the action server object
            createActionServer(obj, resolvedName);
        end

        function delete(obj)
        %DELETE Shut down action server
        %   DELETE(SERVER) shuts down the ROS action server object SERVER
        %   and removes its registration from the ROS master

        % Cannot tell server to remove the action client without valid
        % internal node and server handle value
            if ~isempty(obj.InternalNode) && ...
                    isvalid(obj.InternalNode) && ...
                    ~isempty(obj.ActionServerHandle)
                try
                    removeActServer(obj.InternalNode, ...
                                    obj.ActionServerHandle);
                catch
                    warning(message('ros:mlroscpp:actionserver:ShutdownError'));
                end
            end
            obj.InternalNode = [];
        end

        function msg = rosmessage(obj, varargin)
        % ROSMESSAGE Create a new action result message
        %   MSG = ROSMESSAGE(SERVER) creates and returns an empty message MSG.
        %   The message type of MSG is determined by the action type. The
        %   format of MSG is determined by the DataFormat of the action
        %   server. The message is the default result that this server
        %   uses for the final reply to received goals.
        %
        %   Example:
        %      % Create an action server and result message
        %      server1 = ros.SimpleActionServer(node,"/move_group",...
        %         "moveit_msgs/MoveGroup",ExecuteGoalFcn=@executeGoal);
        %      resultObj = ROSMESSAGE(server1);
        %
        %      % Improve performance by using struct messages
        %      server2 = ros.SimpleActionServer(node,"/move_group",...
        %         "moveit_msgs/MoveGroup",ExecuteGoalFcn=@executeGoal,...
        %         DataFormat="struct");
        %      resultStruct = ROSMESSAGE(server2);

            validateDataFormatROSMessage(obj, varargin{:})

            msg = rosmessage(obj.ResultMessageType, 'DataFormat', obj.DataFormat);
        end

        function msg = getFeedbackMessage(obj, varargin)
        % GETFEEDBACKMESSAGE Create a new action feedback message
        %   MSG = GETFEEDBACKMESSAGE(SERVER) creates and returns an empty
        %   message MSG. The message type of MSG is determined by the
        %   action type. The format of MSG is determined by the DataFormat
        %   of the action server. The message is the default feedback that
        %   this server sends to the client while working on a goal.
        %
        %   Example:
        %      % Create an action server and feedback message
        %      server1 = ros.SimpleActionServer(node,"/move_group",...
        %         "moveit_msgs/MoveGroup",ExecuteGoalFcn=@executeGoal);
        %      feedbackObj = getFeedbackMessage(server1);
        %
        %      % Improve performance by using struct messages
        %      server2 = ros.SimpleActionServer(node,"/move_group",...
        %         "moveit_msgs/MoveGroup",ExecuteGoalFcn=@executeGoal,...
        %         DataFormat="struct");
        %      feedbackStruct = getFeedbackMessage(server2);

            validateDataFormatROSMessage(obj, varargin{:})

            msg = rosmessage(obj.FeedbackMessageType, 'DataFormat', obj.DataFormat);
        end

        function set.ExecuteGoalFcn(obj, goalFcn)
        %set.ExecuteGoalFcn Set the callback for newly-received goals
        %
        %   sub.ExecuteGoalFcn = CB sets the callback function that should
        %   be invoked when a new goal is received. Here, CB is either a
        %   scalar function handle or a cell array. You can pass additional
        %   parameters to the callback function by including both the
        %   function handle and the parameters as elements of a cell array
        %   and assign it to CB.
        %
        %   This callback is required for the action server to operate, so
        %   it cannot be empty.
        %
        %   Each callback function must have the following signature
        %   (see the help for ros.SimpleActionServer for a more detailed
        %   explanation):
        %
        %      function [RESULT, SUCCESS] = actionCallback(SRC,GOAL,DEFAULTFEEDBACK,DEFAULTRESULT)
        %          RESULT = DEFAULTRESULT;
        %          SUCCESS = false;
        %          % Execute the goal while providing periodic feedback
        %
        %          % Check if preempt is requested or error occurs and
        %          % handle interrupted goal attempt in that case
        %
        %          % If goal is successfully reached, update the SUCCESS
        %          % and RESULT outputs
        %      end

        % Make sure this is a valid function specifier
        % Error if empty, as execute goal callback is required for action
        % server to operate
            [fcnHandle, userData] = ...
                ros.internal.Parsing.validateFunctionHandle(goalFcn);

            % Set properties used when message is received
            obj.ActualExecuteGoalFcn = fcnHandle; %#ok<MCSUP>
            obj.ExecuteGoalFcnUserData = userData; %#ok<MCSUP>

            obj.ExecuteGoalFcn = goalFcn;
        end

        function sendFeedback(obj, feedbackMsg)
        %sendFeedback Send feedback to action client while goal is executing
        %   sendFeedback(SERVER,FEEDBACK) sends the feedback message
        %   FEEDBACK to the client that sent the currently-executing goal.
        %   The type and format of the message must match the ActionType
        %   and DataFormat of the action server.
        %
        %   See the help for ros.SimpleActionServer for an example of
        %   sendFeedback in use within the ExecuteGoalFcn callback.

            try
                % Convert received message to object if necessary
                if obj.UseObjectMsg
                    feedbackMsgStruct = toROSStruct(feedbackMsg);
                else
                    feedbackMsgStruct = feedbackMsg;
                end

                % Send feedback over the network
                publishFeedback(obj.InternalNode, ...
                                obj.ActionServerHandle, ...
                                feedbackMsgStruct);
            catch ex
                % See if the issue is with the message structure
                validateInputMessage(obj, feedbackMsg, obj.FeedbackMessageType, ...
                                     'SimpleActionServer', 'sendFeedback')

                % Otherwise pass the error back through the callback
                rethrow(ex)
            end
        end

        function status = isPreemptRequested(obj)
        %isPreemptRequested Check if goal has been preempted
        %   STATUS = isPreemptRequested(SERVER) checks whether the
        %   currently-executing goal has been preempted. This can occur if
        %   the client cancels the goal, or if the server receives a new
        %   goal to execute.
        %
        %   See the help for ros.SimpleActionServer for an example of
        %   isPreemptRequested in use within the ExecuteGoalFcn callback.

            responseStruct = isPreemptRequested(obj.InternalNode, ...
                                                obj.ActionServerHandle);
            status = responseStruct.isPreemptRequested;
        end
    end

    methods (Hidden)
        function status = isNewGoalAvailable(obj)
        %isPreemptRequested Check if new goal is waiting for execution
        %   STATUS = isNewGoalAvailable(SERVER) checks whether the server
        %   has received a new goal to execute.

            responseStruct = isNewGoalAvailable(obj.InternalNode, ...
                                                obj.ActionServerHandle);
            status = responseStruct.isNewGoalAvailable;
        end
    end

    methods (Access = ?ros.internal.mixin.ROSInternalAccess)
        function executeNextGoal(obj, goalMsg, varargin)
        %executeNextGoal Take action based on goal from client to execute

        % Default messages to pass to user's callback
            defaultFeedbackMsg = getFeedbackMessage(obj);
            defaultResultMsg = rosmessage(obj);

            % Convert received message to object if necessary
            if obj.UseObjectMsg
                goalMsg = feval(obj.ActionInfo.msgClassGen, goalMsg);
            end

            try
                % Call user-provided function to execute the goal
                [resultMsg, success] = feval(obj.ActualExecuteGoalFcn, ...
                                             obj, ...
                                             goalMsg, ...
                                             defaultFeedbackMsg, ...
                                             defaultResultMsg, ...
                                             obj.ExecuteGoalFcnUserData{:});
            catch ex
                % Abort the goal if something goes wrong due to
                % exception or user mistake in callback
                resultMsg = defaultResultMsg;
                success = false;
                warning(message('ros:mlroscpp:actionserver:UserCallbackError', ex.message))
            end

            try
                % Convert response to struct if needed
                % Keep original for validation if necessary
                if obj.UseObjectMsg
                    resultMsgStruct = toROSStruct(resultMsg);
                else
                    resultMsgStruct = resultMsg;
                end

                % Send the result back over the network
                if success
                    succeedGoal(obj.InternalNode, ...
                                obj.ActionServerHandle, ...
                                obj.GoalStatusSucceeded, ...
                                resultMsgStruct);
                elseif isPreemptRequested(obj)
                    preemptGoal(obj.InternalNode, ...
                                obj.ActionServerHandle, ...
                                obj.GoalStatusPreempted, ...
                                resultMsgStruct);
                else
                    abortGoal(obj.InternalNode, ...
                              obj.ActionServerHandle, ...
                              obj.GoalStatusAborted, ...
                              resultMsgStruct);
                end
            catch ex
                % Abort the goal if the result message cannot be sent
                % Use the default result message to avoid issues
                try
                    validateInputMessage(obj, resultMsg, obj.ResultMessageType, ...
                                         'SimpleActionServer', 'callback')
                catch ex
                    % Exception will be used in warning
                end
                warning(message('ros:mlroscpp:actionserver:SendResponseError', ex.message))
                defaultResultMsgStruct = rosmessage(obj.ResultMessageType, 'DataFormat', 'struct');
                abortGoal(obj.InternalNode, ...
                          obj.ActionServerHandle, ...
                          obj.GoalStatusAborted, ...
                          defaultResultMsgStruct);
            end
        end
    end

    methods (Access = private)
        function createActionServer(obj, name)
        %createActionServer Establish action server on ROS network

        % Preemption will not require a callback
        % The user-provided callback should check if preemption occurred
            newGoalCallback = '';
            executeGoalCallback = obj.ExecuteGoalCallbackHandler.CallbackName;
            preemptGoalCallback = '';

            % Paths for loading libraries
            dllPathsGoal = ros.internal.utilities.getPathOfDependentDlls(obj.GoalMessageType, 'ros');
            dllPathsFeedback = ros.internal.utilities.getPathOfDependentDlls(obj.FeedbackMessageType, 'ros');
            dllPathsResult = ros.internal.utilities.getPathOfDependentDlls(obj.ResultMessageType, 'ros');
            dllPaths = [dllPathsGoal dllPathsFeedback dllPathsResult];

            try
                returnCall = addActServer(obj.InternalNode, ...
                                          obj.ServerNodeHandle, ...
                                          name, ...
                                          obj.ActionInfo.path, ...
                                          obj.ActionInfo.cppFactoryClass, ...
                                          newGoalCallback, ...
                                          executeGoalCallback, ...
                                          preemptGoalCallback,...
                                          dllPaths);

                if isempty(returnCall) || ~isstruct(returnCall)
                    error(message('ros:mlroscpp:node:InvalidReturnCallError'))
                elseif ~isfield(returnCall, 'handle') || ...
                        isempty(returnCall.handle) || ...
                        ~isfield(returnCall, 'actionName') || ...
                        isempty(returnCall.actionName)
                    error(message('ros:mlroscpp:node:InvalidReturnCallHandleError'))
                end
                obj.ActionServerHandle = returnCall.handle;
                obj.ActionName = returnCall.actionName;
                % Initialize callback to process goals
                initActServerCallback(obj.InternalNode, ...
                                      returnCall.handle, ...
                                      obj.ExecuteGoalCallbackHandler, ...
                                      obj.MaxConcurrentCallbacks);
                % No need to check reply - should error on failure
            catch ex
                newEx = MException(message('ros:mlroscpp:actionserver:CreateError', ...
                                           name, obj.ActionType));
                throw(addCause(newEx, ex));
            end
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function parser = getParser(obj)
        %getParser Set up parser for constructor inputs

            parser = inputParser;

            % Node, service name, and service type are required inputs
            addRequired(parser, 'node',  @(x) ...
                        validateattributes(x, {'ros.Node'}, ...
                                           {'scalar'}, ...
                                           'SimpleActionServer', 'node'));
            addRequired(parser, 'actionName',  @(x) ...
                        validateattributes(x, {'char', 'string'}, ...
                                           {'nonempty', 'scalartext'}, ...
                                           'SimpleActionServer', 'actionName'));
            addRequired(parser, 'actionType',  @(x) ...
                        validateattributes(x, {'char', 'string'}, ...
                                           {'nonempty', 'scalartext'}, ...
                                           'SimpleActionServer', 'actionType'));
            % ExecuteGoalFcn input is checked on property set
            % It will error if the default (empty) input is maintained
            addParameter(parser, 'ExecuteGoalFcn', function_handle.empty)
            addDataFormatToParser(obj, parser, 'SimpleActionServer')
        end
    end
end
