function [client, goalMsg] = rosactionclient(actionName, varargin)
%ROSACTIONCLIENT Create a ROS action client
%   CLIENT = ROSACTIONCLIENT(ACTIONNAME) creates a CLIENT for the ROS
%   action with name ACTIONNAME. If an action with this name is available
%   in the ROS network, the client determines the action type automatically.
%   If the action is not available, this function displays an error. CLIENT
%   is a simple action client and allows you to track a single goal at a
%   time. ACTIONNAME is a string scalar.
%
%   CLIENT = ROSACTIONCLIENT(ACTIONNAME,ACTIONTYPE) creates an action
%   client for the ROS action with name ACTIONNAME and type ACTIONTYPE. If
%   no action with this name is available in the ROS network, it is
%   created. If an action with the same name is available, but its
%   type does not match ACTIONTYPE, the function displays an error.
%   ACTIONTYPE is a string scalar.
%
%   CLIENT = ROSACTIONCLIENT(___,Name,Value) provides additional
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
%   [CLIENT,GOALMSG] = ROSACTIONCLIENT(___) returns a goal message GOALMSG
%   that you can send with the action client CLIENT. The goal message is
%   initialized with default values. The format of GOALMSG is determined by
%   the DataFormat of the action client.
%
%   Use ROSACTIONCLIENT to connect to an action server and request the
%   execution of action goals. You can get feedback on the execution
%   progress and cancel the goal at any time.
%
%
%   Example:
%
%      % Create action client for TurtleBot movement
%      % The turtlebot_move action server needs to be running
%      % Use struct message format for better performance
%      turtleActClient = ROSACTIONCLIENT("/turtlebot_move",...
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
%   See also ros.SimpleActionClient, ROSACTION.

%   Copyright 2016-2021 The MathWorks, Inc.
%#codegen

    if isempty(coder.target)
        try
            % Create and return action client object using the global node
            client = ros.SimpleActionClient([], actionName, varargin{:});

            % Create goal message, if requested by user
            if nargout > 1
                goalMsg = rosmessage(client);
            end
        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        coder.internal.narginchk(1,10,nargin);
        client = ros.SimpleActionClient([], actionName, varargin{:});

        % Assign output message,  if requested
        if nargout > 1
            goalMsg = rosmessage(client);
        end
    end
end
