function output = rosaction( operation, varargin )
%ROSACTION Display or return information about ROS actions
%   ACTIONLIST = ROSACTION('list') returns a list of available ROS actions
%   in the ROS network as a string array ACTIONLIST.
%   If the output argument ACTIONLIST is not defined, the list is printed
%   to the command line.
%   Simplified form: ROSACTION list
%
%   ACTIONINFO = ROSACTION('info', 'ACTIONNAME') returns the action type,
%   the action server, and action clients for a specific action with name
%   ACTIONNAME. ACTIONINFO is a structure containing the following fields:
%     - ActionType: Type of the action
%     - GoalMessageType: Message type of goal messages sent by the action client
%     - FeedbackMessageType: Message type of feedback messages sent by the action server
%     - ResultMessageType: Message type of result messages sent by the action server
%     - ActionServer: Structure with information about the action server
%     - ActionClients: Structure array with information about all registered action clients
%
%   The structure for 'ActionServer' and 'ActionClients' has
%   the following fields:
%     - NodeName: Name of the node containing the action server or client
%     - URI: The URI of the node containing the action server or client
%
%   If the action name does not exist, an error is displayed.
%   Simplified form: ROSACTION info ACTIONNAME
%
%   ACTIONTYPE = ROSACTION('type', 'ACTIONNAME') returns the action type
%   for a specific action with name ACTIONNAME. ACTIONTYPE is a string.
%   Simplified form: ROSACTION type ACTIONNAME
%
%
%   Example:
%      % List all actions
%      ROSACTION list
%
%      % Get information about the /turtlebot_move action
%      % You need to be connected to the TurtleBot
%      ROSACTION info /turtlebot_move
%
%      % Get type of the /fibonacci action
%      ROSACTION type /fibonacci

%   Copyright 2016-2020 The MathWorks, Inc.

    try
        supportedOperations = {'list', 'info', 'type'};
        if nargin == 0
            % User did not specify any arguments. Give user a list of supported operations.
            error(message('ros:mlros:common:OperationNotSpecified', ...
                          ros.internal.Parsing.printStringList(supportedOperations)));
        end

        % Parse the specified operation
        [validOperation, actionName] = parseInputs(supportedOperations, operation, varargin{:});

        if nargout == 0
            rosactionImpl(validOperation, actionName, varargin{:});
        else
            output = rosactionImpl(validOperation, actionName, varargin{:});
        end
    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end

end

function [validOperation, actionName] = parseInputs(supportedOperations, operation, varargin)
%parseInputs Parse the inputs to the rosaction function
%   Return the validated operation and the action name (if the user
%   specified one).

    actionName = '';
    validOperation = validatestring(operation, supportedOperations, 'rosaction', 'operation');

    switch validOperation
      case 'list'
        % Syntax: rosaction list
        % No additional arguments are allowed
        narginchk(2,2);

      case 'info'
        % Syntax: rosaction info 'ACTIONNAME'
        % Only one additional argument (action name) is allowed
        narginchk(3,3);
        actionName = varargin{1};
        validateActionName(actionName);

      case 'type'
        % Syntax: rosaction type 'ACTIONNAME'
        % Only one additional argument (action name) is allowed
        narginchk(3,3);
        actionName = varargin{1};
        validateActionName(actionName);
    end

end

function validateActionName(actionName)

% Convert strings to chars to ensure empty inputs produce appropriate
% error messages. Note that this error will also identify "" as an
% empty input.
    actionName = convertStringsToChars(actionName);
    validateattributes(actionName, {'char','string'}, {'nonempty', 'scalartext'}, 'rosaction', 'actionName');
end

function output = rosactionImpl(validOperation, actionName, varargin)
%rosactionImpl Actual implementation of rosaction functionality.

% Create action introspection object
    actionIntro = ros.internal.action.ActionIntrospection([]);

    % We can assume that the operation is already parsed and valid
    switch validOperation
      case 'list'
        % Display a sorted list of actions that are registered on the master.
        actionNames = actionIntro.actionList;

        % If output argument specified, return the list
        if nargout == 1
            output = actionNames;
            return;
        end

        % Otherwise, display the sorted list on the console
        for name = actionNames
            disp(char(name));
        end

      case 'info'
        % Get action information
        actionInfo = actionIntro.actionInfo(actionName);

        % If output argument specified, return the information structure
        if nargout == 1
            output = actionInfo;
            return;
        end

        % Otherwise, display the action information on the console
        actionInfoString = actionIntro.infoStructToString( actionInfo );
        disp(actionInfoString);

      case 'type'
        % Get the type of an action
        actionType = actionIntro.actionType(actionName);

        % If output argument specified, return the type string
        if nargout == 1
            output = actionType;
            return;
        end

        % Otherwise, display the action type on the console
        disp(actionType);
    end

end
