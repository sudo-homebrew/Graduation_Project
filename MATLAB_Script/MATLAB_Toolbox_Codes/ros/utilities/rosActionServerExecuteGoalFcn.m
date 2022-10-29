function callbackHandle = rosActionServerExecuteGoalFcn(varargin)
%rosActionServerExecuteGoalFcn Return a function handle for action server callback
%   CB = rosActionServerExecuteGoalFcn returns a function handle for use
%   as the ExecuteGoalFcn for a ROS action server. With no input specifying
%   behavior, when used in the action server the callback will immediately
%   indicate the goal was reached and return the default result message.
%
%   CB = rosActionServerExecuteGoalFcn(___,Name=Value) provides additional
%   options specified by one or more Name=Value pair arguments.
%
%      "IsGoalReachedFcn"          - Callback function executed to
%                                    determine if the goal reached.
%                                    Default: Returns true
%      "StepExecutionFcn"          - Callback function executed every
%                                    iteration to progress towards the goal.
%                                    If empty, no step function will be
%                                    called.
%                                    Default: Empty
%      "CreateFeedbackFcn"         - Callback function executed every
%                                    iteration to construct a feedback
%                                    message to send to the action client.
%                                    If empty, no feedback will be sent.
%                                    Default: Returns default feedback message
%      "CreatePreemptedResultFcn"  - Callback function executed if the goal
%                                    is preempted to construct the result
%                                    message to send to the action client.
%                                    Default: Returns default result message
%      "CreateSuccessfulResultFcn" - Callback function executed if the goal
%                                    is successfully reached to construct
%                                    the result message to send to the
%                                    action client.
%                                    Default: Returns default result message
%      "StepDelay"                 - Number of seconds to pause each
%                                    iteration. Providing a non-zero value
%                                    is recommended to allow execution of
%                                    other callbacks, such as allowing the
%                                    ROS action client to react to received
%                                    feedback.
%                                    Default: 0.01 seconds
%      "UserData"                  - Data for later use or modification.
%                                    This data will be stored in an object
%                                    passed into the other provided
%                                    callbacks, and is a means to share
%                                    data or resources between them.
%                                    Default: []
%
%   Each specified callback function requires two input arguments, the
%   shared object storing UserData as the first, and an appropriate ROS
%   message as the second. Most callbacks must provide appropriate output.
%   Details are explained in the function signatures below.
%
%      function ATGOAL = isGoalReached(SHAREDOBJ,GOAL)
%          % Set output ATGOAL to true if the current goal, GOAL, is reached
%          % Set the output to false to continue goal execution
%          % Use the data or resources in SHAREDOBJ.UserData to determine
%          % the current state
%      end
%
%      function stepExecution(SHAREDOBJ,GOAL)
%          % Take action to make progress towards the current goal, GOAL
%          % Use the data or resources in SHAREDOBJ.UserData as required
%      end
%
%      function FEEDBACK = createFeedback(SHAREDOBJ,DEFAULTFEEDBACK)
%          FEEDBACK = DEFAULTFEEDBACK;
%          % Build the feedback message using the data or resources in
%          % SHAREDOBJ.UserData as required
%      end
%
%      function RESULT = createPreemptedResult(SHAREDOBJ,DEFAULTRESULT)
%          RESULT = DEFAULTRESULT;
%          % Build the result message reflecting the incomplete goal
%          % execution due to preemption
%          % Use the data or resources in SHAREDOBJ.UserData as required
%      end
%
%      function RESULT = createSuccessfulResult(SHAREDOBJ,DEFAULTRESULT)
%          RESULT = DEFAULTRESULT;
%          % Build the successful result message here
%          % Use the data or resources in SHAREDOBJ.UserData as required
%      end
%
%   These callback function inputs accept only function handles, not cell
%   arrays. Additional required data can either be set in the UserData
%   input, or an anonymous function handle can be passed to provide
%   additional parameters.
%
%
%   Example:
%
%      % Create or connect to ROS network
%      rosinit
%
%      % Set up action server callback for calculating Fibonacci sequence
%      fibSequence = int32([0 1]);  % Store in UserData
%      isGoalReached = @(sharedObj,goal) numel(sharedObj.UserData) > goal.Order;
%      cb = rosActionServerExecuteGoalFcn(IsGoalReachedFcn=isGoalReached,...
%          StepExecutionFcn=@nextFibNumber,...
%          CreateFeedbackFcn=@assignUserDataToMessage,...
%          CreateSuccessfulResultFcn=@assignUserDataToMessage,...
%          StepDelay=0.2,...
%          UserData=fibSequence);
%
%      % Create action server, using struct messages for better performance
%      server = rosactionserver("/fibonacci","actionlib_tutorials/Fibonacci",...
%          ExecuteGoalFcn=cb,DataFormat="struct");
%
%      % Create action client and send a goal to the server
%      client = rosactionclient("/fibonacci","actionlib_tutorials/Fibonacci",...
%          DataFormat="struct");
%      goal = rosmessage(client);
%      goal.Order = int32(10);
%      result = sendGoalAndWait(client,goal);
%
%      % Define step execution callback
%      function nextFibNumber(sharedObj,~)
%          sharedObj.UserData(end+1) = sharedObj.UserData(end-1) + sharedObj.UserData(end);
%      end
%
%      % Define helper function to assign current sequence to message field
%      function msg = assignUserDataToMessage(sharedObj,msg)
%          % Works because in Fibonacci action, both the feedback and the
%          % result messages use the fieldname "Sequence"
%          msg.Sequence = sharedObj.UserData;
%      end
%
%   See also ROSACTIONSERVER, ros.SimpleActionServer.

%   Copyright 2021 The MathWorks, Inc.

% Set up defaults
    defaultIsGoalReachedFcn = @(~, ~) true;
    defaultStepExecutionFcn = function_handle.empty;
    defaultCreateFeedbackFcn = @(~, feedbackMsg) feedbackMsg;
    defaultCreatePreemptedResultFcn = @(~, resultMsg) resultMsg;
    defaultCreateSuccessfulResultFcn = @(~, resultMsg) resultMsg;
    defaultStepDelay = 0.01;
    defaultUserData = [];

    % Set up parser
    parser = inputParser;
    addParameter(parser, 'IsGoalReachedFcn', defaultIsGoalReachedFcn, ...
                 @(x) validateattributes(x, {'function_handle'}, ...
                                         {'scalar'}, ...
                                         'rosActionServerExecuteGoalFcn', ...
                                         'IsGoalReachedFcn'));
    addParameter(parser, 'StepExecutionFcn', defaultStepExecutionFcn, ...
                 @(x) validateEmptyOrScalarFcnHandle(x, 'StepExecutionFcn'));
    addParameter(parser, 'CreateFeedbackFcn', defaultCreateFeedbackFcn, ...
                 @(x) validateEmptyOrScalarFcnHandle(x, 'CreateFeedbackFcn'));
    addParameter(parser, 'CreatePreemptedResultFcn', defaultCreatePreemptedResultFcn, ...
                 @(x) validateattributes(x, {'function_handle'}, ...
                                         {'scalar'}, ...
                                         'rosActionServerExecuteGoalFcn', ...
                                         'CreatePreemptedResultFcn'));
    addParameter(parser, 'CreateSuccessfulResultFcn', defaultCreateSuccessfulResultFcn, ...
                 @(x) validateattributes(x, {'function_handle'}, ...
                                         {'scalar'}, ...
                                         'rosActionServerExecuteGoalFcn', ...
                                         'CreateSuccessfulResultFcn'));
    addParameter(parser, 'StepDelay', defaultStepDelay, ...
                 @(x) validateattributes(x, {'numeric'}, ...
                                         {'scalar', 'nonnegative', 'finite'}, ...
                                         'rosActionServerExecuteGoalFcn', ...
                                         'StepDelay'));
    addParameter(parser, 'UserData', defaultUserData);

    % Parse the input parameters
    parse(parser, varargin{:})

    % Create the shareable handle object for containing user-specified data
    shareObj = ros.internal.DataContainer(parser.Results.UserData);

    % Create the function handle to be used as the ExecuteGoalFcn
    callbackHandle = ...
        @(actionServer, goal, defaultFeedback, defaultResult) ...
        basicExecuteGoalFcn(actionServer, ...
                            goal, ...
                            defaultFeedback, ...
                            defaultResult, ...
                            parser.Results.IsGoalReachedFcn, ...
                            parser.Results.StepExecutionFcn, ...
                            parser.Results.CreateFeedbackFcn, ...
                            parser.Results.CreatePreemptedResultFcn, ...
                            parser.Results.CreateSuccessfulResultFcn, ...
                            parser.Results.StepDelay, ...
                            shareObj);
end

function validateEmptyOrScalarFcnHandle(h, varName)
%validateEmptyOrScalarFcnHandle Validation for functions that can be empty
%   Ensure that the input is either empty, signifying that the function
%   handle will not be used, or a scalar function handle. Any class of
%   empty input is valid, and treated the same.

    if ~isempty(h)
        validateattributes(h, {'function_handle'}, {'scalar'}, ...
                           'rosActionServerExecuteGoalFcn', varName)
    end
end

function [result, success] = basicExecuteGoalFcn(actionServer, ...
                                                 goal, ...
                                                 defaultFeedback, ...
                                                 defaultResult, ...
                                                 isGoalReached, ...
                                                 stepExecution, ...
                                                 createFeedback, ...
                                                 createPreemptedResult, ...
                                                 createSuccessfulResult, ...
                                                 stepDelay, ...
                                                 shareObj)
%basicExecuteGoalFcn ExecuteGoalFcn framework for ROS action server
%   Callback for ROS action server in a recommended framework, calling
%   user-specified functionality for specific checks or actions that need
%   to be taken at different points in the callback.
%
%   Please see the main function documentation for details
%   (help rosActionServerExecuteGoalFcn)

% Set up for main loop
    try
        tfAtGoal = isGoalReached(shareObj, goal);
    catch ex
        error(message('ros:utilities:actionservercallback:BasicCallbackUserFcnError', 'IsGoalReachedFcn', ex.message))
    end

    % Primary control loop
    while ~tfAtGoal
        if ~isPreemptRequested(actionServer)
            % Take primary action step
            if ~isempty(stepExecution)
                try
                    stepExecution(shareObj, goal)
                catch ex
                    error(message('ros:utilities:actionservercallback:BasicCallbackUserFcnError', 'StepExecutionFcn', ex.message))
                end
            end

            try
                tfAtGoal = isGoalReached(shareObj, goal);
            catch ex
                error(message('ros:utilities:actionservercallback:BasicCallbackUserFcnError', 'IsGoalReachedFcn', ex.message))
            end

            % If goal not reached, handle periodic tasks
            if ~tfAtGoal
                % Send feedback at each goal execution step
                if ~isempty(createFeedback)
                    try
                        feedbackMsg = createFeedback(shareObj, defaultFeedback);
                    catch ex
                        error(message('ros:utilities:actionservercallback:BasicCallbackUserFcnError', 'CreateFeedbackFcn', ex.message))
                    end
                    sendFeedback(actionServer, feedbackMsg);
                end

                % Pause between execution steps to clear the callback queue
                if stepDelay > 0
                    pause(stepDelay)
                end
            end
        else
            % Goal execution has been preempted
            try
                result = createPreemptedResult(shareObj, defaultResult);
            catch ex
                error(message('ros:utilities:actionservercallback:BasicCallbackUserFcnError', 'CreatePreemptedResultFcn', ex.message))
            end
            success = false;
            return
        end
    end

    % Successfully reached goal
    try
        result = createSuccessfulResult(shareObj, defaultResult);
    catch ex
        error(message('ros:utilities:actionservercallback:BasicCallbackUserFcnError', 'CreateSuccessfulResultFcn', ex.message))
    end
    success = true;

end
