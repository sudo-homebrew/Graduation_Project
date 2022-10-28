classdef SimpleActionClient < ros.internal.mixin.ROSInternalAccess & ...
        coder.ExternalDependency
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
%   See also ROSACTIONCLIENT, ROSACTION.

%   Copyright 2021 The MathWorks, Inc.
%#codegen

    properties (Dependent, SetAccess = private)
        %GoalMessage
        GoalMessage

        %FeedbackMessage
        FeedbackMessage

        %ResultMessage
        ResultMessage

        %GoalState - State of the tracked goal
        GoalState
    end

    properties (SetAccess = immutable)
        %ActionName - Name of action associated with this client
        ActionName

        %ActionType - Type of action associated with this client
        ActionType

        %DataFormat - Message format to be used
        DataFormat

        %GoalMessageType - Message type of goal message
        GoalMessageType

        %FeedbackMessageType - Message type of feedback message
        FeedbackMessageType

        %ResultMessageType - Message type of result message
        ResultMessageType

        %ActivationFcn - Function executed on goal activation
        ActivationFcn

        %FeedbackFcn - Function executed for goal feedback
        FeedbackFcn

        %ResultFcn - Function executed for goal result
        ResultFcn
    end

    properties (SetAccess = private)
        %IsServerConnected - Indicates if client is connected to action server
        IsServerConnected

        %Goal - The tracked goal
        Goal
    end

    properties (Access = private)
        GoalMsgStruct
        FeedbackMsgStruct
        ResultMsgStruct
        Mutex
        IsInitialized = false
    end

    properties
        ActionClientHelperPtr
    end

    methods
        function obj = SimpleActionClient(node, actionNamespace, varargin)
        %SimpleActionClient Create a ROS action client object
        %   Please see the class documentation
        %   (help ros.SimpleActionClient) for more details.

        % Set defaults
            defaults = struct( ...
                'DataFormat', 'object', ...
                'ActivationFcn',@(~) fprintf('Goal active\n'), ...
                'FeedbackFcn',@(~,msg) [], ...
                'ResultFcn',@(~,res) fprintf('Result message received\n'));
            coder.inline('never');
            coder.extrinsic('ros.codertarget.internal.getCodegenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo');
            coder.extrinsic('ros.codertarget.internal.ROSMATLABCgenInfo.getInstance');
            coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');

            % Ensure actionType is not empty
            coder.internal.assert(nargin>2 && contains(varargin{1},'/'),'ros:mlroscpp:codegen:MissingMessageType',actionNamespace,'SimpleActionClient');

            % A node cannot create another node in codegen
            if ~isempty(node)
                coder.internal.assert(false,'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % Action name and type must be specified for codegen
            actname = convertStringsToChars(actionNamespace);
            validateattributes(actname,{'char'},{'nonempty'},...
                               'SimpleActionClient','actionName');
            acttype = convertStringsToChars(varargin{1});
            validateattributes(acttype,{'char'},{'nonempty'},...
                               'SimpleActionClient','actionType');

            % Enables message struct type generation for all required
            % action message cpp definitions
            rosmessage([acttype 'Action'],'DataFormat','struct');

            % Parse NV pairs including all callback functions
            nvPairs = struct( ...
                'ActivationFcn', uint32(0), ...
                'FeedbackFcn', uint32(0), ...
                'ResultFcn', uint32(0), ...
                'DataFormat', uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{2:end});
            activationFcn = coder.internal.getParameterValue(pStruct.ActivationFcn, ...
                                                             defaults.ActivationFcn, varargin{2:end});
            feedbackFcn = coder.internal.getParameterValue(pStruct.FeedbackFcn, ...
                                                           defaults.FeedbackFcn, varargin{2:end});
            resultFcn = coder.internal.getParameterValue(pStruct.ResultFcn, ...
                                                         defaults.ResultFcn, varargin{2:end});

            dataFormat = coder.internal.getParameterValue(pStruct.DataFormat, ...
                                                          defaults.DataFormat, varargin{2:end});
            validateStringParameter(dataFormat,{'object','struct'},'SimpleActionClient','DataFormat');
            coder.internal.assert(strcmp(dataFormat,'struct'), ...
                                  'ros:mlroscpp:codegen:InvalidDataFormat','SimpleActionClient');
            coder.internal.assert(isa(activationFcn,'function_handle'), ...
                                  'ros:mlroscpp:codegen:InvalidCallback','ActivationFcn','SimpleActionClient');
            coder.internal.assert(isa(feedbackFcn,'function_handle'), ...
                                  'ros:mlroscpp:codegen:InvalidCallback','FeedbackFcn','SimpleActionClient');
            coder.internal.assert(isa(resultFcn,'function_handle'), ...
                                  'ros:mlroscpp:codegen:InvalidCallback','ResultFcn','SimpleActionClient');

            % Store input arguments
            obj.ActionName = actname;
            obj.ActionType = acttype;
            obj.IsServerConnected = false;
            obj.GoalMessageType = [acttype 'Goal'];
            obj.FeedbackMessageType = [acttype 'Feedback'];
            obj.ResultMessageType = [acttype 'Result'];
            obj.DataFormat = dataFormat;
            obj.ActivationFcn = activationFcn;
            obj.FeedbackFcn = feedbackFcn;
            obj.ResultFcn = resultFcn;

            % Get and register code generation information
            cgActionInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,actname,[acttype 'Action'],'actclient');

            cgGoalInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,actname,[acttype 'Goal'],'actclient');
            goalMsgStructGenFcn = str2func(cgGoalInfo.MsgStructGen);
            obj.GoalMsgStruct = goalMsgStructGenFcn();

            cgFeedbackInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,actname,[acttype 'Feedback'],'actclient');
            feedbackMsgStructGenFcn = str2func(cgFeedbackInfo.MsgStructGen);
            obj.FeedbackMsgStruct = feedbackMsgStructGenFcn();

            cgResultInfo = coder.const(@ros.codertarget.internal.getCodegenInfo,actname,[acttype 'Result'],'actclient');
            resultMsgStructGenFcn = str2func(cgResultInfo.MsgStructGen);
            obj.ResultMsgStruct = resultMsgStructGenFcn();

            % Create pointer to MATLABActClient object
            coder.ceval('auto goalStructPtr= ', coder.wref(obj.GoalMsgStruct));
            coder.ceval('auto feedbackStructPtr= ', coder.wref(obj.FeedbackMsgStruct));
            coder.ceval('auto resultStructPtr= ', coder.wref(obj.ResultMsgStruct));

            TemplateTypeStr = ['MATLABActClient<',cgActionInfo.MsgClass, ...
                               ',' cgGoalInfo.MsgClass ',' cgFeedbackInfo.MsgClass ',' cgResultInfo.MsgClass ...
                               ',' cgGoalInfo.MsgStructGen '_T,' cgFeedbackInfo.MsgStructGen '_T,' ...
                               cgResultInfo.MsgStructGen '_T>'];

            obj.ActionClientHelperPtr = coder.opaque(['std::unique_ptr<', TemplateTypeStr, '>'], 'HeaderFile', 'mlroscpp_actclient.h');
            if ros.internal.codegen.isCppPreserveClasses
                % Create SimpleActionClient by passing in class method as
                % callback
                obj.ActionClientHelperPtr = coder.ceval(['std::unique_ptr<', TemplateTypeStr, ...
                                                         '>(new ', TemplateTypeStr, '([this](){this->activationCallback();},[this](){this->feedbackCallback();},', ...
                                                         '[this](){this->resultCallback();},', ...
                                                         'goalStructPtr,feedbackStructPtr,resultStructPtr));//']);
            else
                % Create SimpleActionClient by passing in static function
                % as callback
                obj.ActionClientHelperPtr = coder.ceval(['std::unique_ptr<', TemplateTypeStr, ...
                                                         '>(new ', TemplateTypeStr, '([obj](){SimpleActionClient_activationCallback(obj);},[obj](){SimpleActionClient_feedbackCallback(obj);},', ...
                                                         '[obj](){SimpleActionClient_resultCallback(obj);},', ...
                                                         'goalStructPtr,feedbackStructPtr,resultStructPtr));//']);
            end
            coder.ceval('MATLABActClient_createActClient',obj.ActionClientHelperPtr,coder.rref(obj.ActionName),...
                        size(obj.ActionName,2));

            % Ensure callback is not optimized away by making an explicict
            % call here
            obj.activationCallback();
            obj.feedbackCallback();
            obj.resultCallback();
            obj.IsInitialized = true;
        end

        function goalMsg = rosmessage(obj,varargin)
        %ROSMESSAGE Create goal message based on action type
        %   GOALMSG = ROSMESSAGE(CLIENT) creates and returns a new goal
        %   message GOALMSG. The message type of GOALMSG is determined
        %   by the action type associated with this action client. The
        %   format of MSG is determined by the DataFormat of the action
        %   client.

            goalMsg = rosmessage(obj.GoalMessageType, 'DataFormat', 'struct');
        end

        function status = waitForServer(obj, varargin)
        %WAITFORSERVER Wait for action server to start
        %   STATUS = WAITFORSERVER(CLIENT) blocks MATLAB from running the current program
        %   until the action server is started up and available to
        %   receive goals. Press Ctrl+C to abort the wait.
        %
        %   STATUS = WAITFORSERVER(CLIENT,TIMEOUT) specifies a TIMEOUT period, in
        %   seconds. If the server does not start up in the timeout
        %   period, this function displays an error.

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.ActionClientHelperPtr);

            status = true;
            coder.internal.narginchk(1,2,nargin);
            waitTimeout = inf;
            if nargin > 1
                validateattributes(varargin{1},{'numeric'},...
                                   {'scalar','nonempty','real','positive'},'waitForServer','Timeout');
                waitTimeout = varargin{1};
                % Address syntax: waitForServer(client,inf)
                % Since MATLAB Interpretation mode does not allow "0" as input
                % timeout, "0" will be passed to C++ class representing
                % infinite case.
                if isinf(waitTimeout)
                    waitTimeout = 0;
                end
            end
            coder.ceval('MATLABActClient_waitForServer',obj.ActionClientHelperPtr,waitTimeout, coder.ref(status));
            if status
                obj.IsServerConnected = true;
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

            coder.ceval('MATLABActClient_lock',obj.ActionClientHelperPtr);
            obj.GoalMsgStruct = goalMsg;
            coder.ceval('MATLABActClient_unlock',obj.ActionClientHelperPtr);
            coder.ceval('MATLABActClient_sendGoal',obj.ActionClientHelperPtr,goalMsg);
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

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.ActionClientHelperPtr);
            coder.ceval('MATLABActClient_cancelGoal',obj.ActionClientHelperPtr);
        end

        function cancelAllGoals(obj)
        %cancelAllGoals Cancel all goals on the server
        %   cancelAllGoals(CLIENT) sends a request to the action server
        %   to cancel all currently pending or active goals. This
        %   includes goals from other action clients.

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.ActionClientHelperPtr);
            coder.ceval('MATLABActClient_cancelAllGoals',obj.ActionClientHelperPtr);
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

            coder.inline('never');

            % Warning if no status output
            if nargout<2
                coder.internal.compileWarning('ros:mlroscpp:codegen:MissingStatusOutput','sendGoalAndWait');
            end

            isDone = false;
            coder.ceval('MATLABActClient_lock',obj.ActionClientHelperPtr);
            obj.GoalMsgStruct = goalMsg;
            coder.ceval('MATLABActClient_unlock',obj.ActionClientHelperPtr);
            coder.internal.narginchk(2,3,nargin);
            waitTimeout = inf;
            if nargin > 2
                validateattributes(varargin{1},{'numeric'},...
                                   {'scalar','nonempty','real','positive'},'waitForServer','Timeout');
                waitTimeout = varargin{1};
            end
            % Address syntax: sendGoalAndWait(client, goalmsg, inf)
            % Since MATLAB Interpretation mode does not allow "0" as input
            % timeout, "0" will be passed to C++ class representing
            % infinite case.
            if isinf(waitTimeout)
                waitTimeout = 0;
            end
            coder.ceval('MATLABActClient_sendGoalAndWait',obj.ActionClientHelperPtr,goalMsg,waitTimeout,coder.ref(isDone));
            if ~isDone
                coder.internal.error('ros:mlros:actionclient:WaitGoalTimeout', num2str(waitTimeout));
            end
            coder.ceval('MATLABActClient_lock',obj.ActionClientHelperPtr);
            resultMsg = obj.ResultMsgStruct;
            coder.ceval('MATLABActClient_unlock',obj.ActionClientHelperPtr);

            % Retrieve state and status text information
            stateLen = int32(0);
            stateLen = coder.ceval('MATLABActClient_getStateLength', obj.ActionClientHelperPtr);
            state = char(zeros(1,stateLen));

            statusTextLen = int32(0);
            statusTextLen = coder.ceval('MATLABActClient_getStatusTextLength', obj.ActionClientHelperPtr);

            if isequal(statusTextLen,0)
                statusText = char(zeros(1,1));
            else
                statusText = char(zeros(1,statusTextLen));
            end
            coder.ceval('MATLABActClient_getStatusInfo', obj.ActionClientHelperPtr, coder.ref(state), coder.ref(statusText));
        end

        function activationCallback(obj)
        %ACTIVATIONCALLBACK Activation callback function of this client

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.ActionClientHelperPtr);
            goalMsg = obj.GoalMessage;
            coder.ceval('//',goalMsg);
            if obj.IsInitialized
                obj.ActivationFcn();
            end
        end

        function feedbackCallback(obj)
        %FEEDBACKCALLBACK Feedback callback function of this client

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.ActionClientHelperPtr);
            latestFeedbackMsg = obj.FeedbackMessage;
            if obj.IsInitialized
                obj.FeedbackFcn(obj,latestFeedbackMsg);
            end
        end

        function resultCallback(obj)
        %RESULTCALLBACK Result callback function of this client

            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.ActionClientHelperPtr);
            latestResultMsg = obj.ResultMessage;
            if obj.IsInitialized
                obj.ResultFcn(obj,latestResultMsg);
            end
        end

        function msg = get.Goal(obj)
            coder.inline('never');
            coder.ceval('MATLABActClient_lock',obj.ActionClientHelperPtr);
            msg = obj.GoalMsgStruct;
            coder.ceval('MATLABActClient_unlock',obj.ActionClientHelperPtr);
        end

        function goalState = get.GoalState(obj)
            coder.inline('never');
            ros.internal.codegen.doNotOptimize(obj.ActionClientHelperPtr);

            % Update the goal state of the current goal
            coder.ceval('MATLABActClient_updateGoalState',obj.ActionClientHelperPtr);
            % Retrieve state and status text information
            stateLen = int32(0);
            stateLen = coder.ceval('MATLABActClient_getStateLength',obj.ActionClientHelperPtr);
            goalState = char(zeros(1,stateLen));
            statusTextLen = int32(0);
            statusTextLen = coder.ceval('MATLABActClient_getStatusTextLength',obj.ActionClientHelperPtr);
            if isequal(statusTextLen,0)
                statusText = char(zeros(1,1));
            else
                statusText = char(zeros(1,statusTextLen));
            end
            coder.ceval('MATLABActClient_getStatusInfo', obj.ActionClientHelperPtr, coder.ref(goalState), coder.ref(statusText));
        end

        function msg = get.GoalMessage(obj)
            coder.ceval('MATLABActClient_lock',obj.ActionClientHelperPtr);
            msg = obj.GoalMsgStruct;
            coder.ceval('MATLABActClient_unlock',obj.ActionClientHelperPtr);
        end

        function msg = get.FeedbackMessage(obj)
            coder.ceval('MATLABActClient_lock',obj.ActionClientHelperPtr);
            msg = obj.FeedbackMsgStruct;
            coder.ceval('MATLABActClient_unlock',obj.ActionClientHelperPtr);
        end

        function msg = get.ResultMessage(obj)
            coder.ceval('MATLABActClient_lock',obj.ActionClientHelperPtr);
            msg = obj.ResultMsgStruct;
            coder.ceval('MATLABActClient_unlock',obj.ActionClientHelperPtr);
        end
    end

    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'GoalMessageType'};
        end

        function ret = getDescriptiveName(~)
            ret = 'ROS ActionClient';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo,bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_actclient.h',srcFolder);
            end
        end
    end

    methods (Static, Access = ?ros.internal.mixin.ROSInternalAccess)
        function props = getImmutableProps()
            props = {'ActionType','ActionName','DataFormat',...
                     'GoalMessageType','FeedbackMessageType','ResultMessageType'};
        end
    end
end

function validateStringParameter(value, options, funcName, varName)
% Separate function to suppress output and just validate
    validatestring(value, options, funcName, varName);
end
