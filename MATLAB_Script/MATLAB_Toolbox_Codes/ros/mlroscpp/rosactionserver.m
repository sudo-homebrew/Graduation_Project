function server = rosactionserver(actionName, actionType, varargin)
%ROSACTIONSERVER Create a ROS action server
%   SERVER = ROSACTIONSERVER(ACTIONNAME,ACTIONTYPE,ExecuteGoalFcn=CB)
%   creates and returns an action server object. The action will be
%   available on the ROS network through its name ACTIONNAME and has the
%   type ACTIONTYPE. It also specifies the function handle callback, CB,
%   that handles the goal execution, feedback communication, and result
%   creation. CB can be a single function handle or a cell array. The first
%   element of the cell array must be a function handle, or a string
%   containing the name of a function. The remaining elements of the cell
%   array can be arbitrary user data that is passed to the callback
%   function.
%
%   SERVER = ROSACTIONSERVER(___,Name=Value) provides additional options
%   specified by one or more Name-Value pair arguments.
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
%   Example:
%
%      % Create or connect to ROS network
%      rosinit
%
%      % Set up action server for calculating Fibonacci sequence
%      % Use struct messages for better performance
%      cb = @fibonacciExecution;
%      server = ROSACTIONSERVER("/fibonacci","actionlib_tutorials/Fibonacci",...
%          ExecuteGoalFcn=cb,DataFormat="struct");
%
%      % Create action client and send a goal to the server
%      client = rosactionclient("/fibonacci","actionlib_tutorials/Fibonacci",...
%          DataFormat="struct");
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
%   See also ros.SimpleActionServer, rosActionServerExecuteGoalFcn, ROSACTIONCLIENT, ROSACTION.

%   Copyright 2021 The MathWorks, Inc.

    server = ros.SimpleActionServer([], actionName, actionType, varargin{:});
end
