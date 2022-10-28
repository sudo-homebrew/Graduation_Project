classdef SimpleActionClientParser < ...
        ros.internal.action.ISimpleActionClientParser & ...
        ros.internal.DataFormatBase & ...
        handle
    %This class is for internal use only. It may be removed in the future.

    %SimpleActionClientParser Handles all parsing needs of the SimpleActionClient
    %   To facilitate testing, the parsing functionality is encapsulated in
    %   this class.

    %   Copyright 2016-2020 The MathWorks, Inc.

    methods
        function [node, actionNamespace, actionType, dataFormat] = parseConstructorInput(obj, defaultActionType, node, actionNamespace, varargin)
        %parseConstructorInput Parse constructor arguments
        %   There are multiple valid syntaxes:
        %   - SimpleActionClient(NODE,"ACTIONNAME")
        %   - SimpleActionClient(NODE,"ACTIONNAME","ACTIONTYPE")
        %   - SimpleActionClient(____,"Name",Value)

        % Convert all strings to character vectors
            [actionNamespace, varargin{:}] = convertStringsToChars(actionNamespace, varargin{:});

            % Parse name-value pairs separately from optional input
            % Note that it is possible that an empty string is a valid
            % namespace (especially once resolved through node namespace)
            ordinalParser = inputParser;
            addRequired(ordinalParser, 'node', ...
                        @(x) validateattributes(x, ...
                                                {'ros.Node'}, ...
                                                {'scalar'}, ...
                                                'SimpleActionClient', ...
                                                'node'));
            addRequired(ordinalParser, 'actionName', ...
                        @(x) validateattributes(x, ...
                                                {'char', 'string'}, ...
                                                {'scalartext'}, ...
                                                'SimpleActionClient', ...
                                                'actionName'));
            addOptional(ordinalParser, 'actionType', defaultActionType, ...
                        @(x) validateattributes(x, ...
                                                {'char', 'string'}, ...
                                                {'nonempty', 'scalartext'}, ...
                                                'SimpleActionClient', ...
                                                'actionType'));
            paramParser = inputParser;
            addDataFormatToParser(obj, paramParser, 'SimpleActionClient')

            % Parse all arguments
            nvPairsStart = ros.internal.Parsing.findNameValueIndex(...
                varargin, paramParser.Parameters);
            if isempty(nvPairsStart)
                % Additional arguments past type assume to be parameters
                nvPairsStart = min(2, numel(varargin)+1);
            end
            parse(ordinalParser, node, actionNamespace, varargin{1:nvPairsStart-1})
            parse(paramParser, varargin{nvPairsStart:end})

            % Extract only necessary return arguments
            % Cannot rely on DataFormat property in this class due to it
            % being a singleton, shared with all action client instances
            actionNamespace = ordinalParser.Results.actionName;
            actionType = ordinalParser.Results.actionType;
            dataFormat = paramParser.Results.DataFormat;
        end

        function [funcHandle, userData] = parseFcnSetterInput(~, fcnHandle, propName)
        %parseFcnSetterInput Parse argument to callback property setter
        %   The input can either be:
        %   - a scalar function handle or
        %   - a cell array containing either a function handle or string as its
        %     first element and arbitrary user data. The string needs to
        %     be a function name.

            [funcHandle, userData] = ros.internal.Parsing.validateFunctionHandle(...
                fcnHandle, 'SimpleActionClient', propName);
        end

        function timeout = parseWaitForServerInput(obj, defaultTimeout, varargin)
        %parseWaitForServerInput Parse the arguments to the waitForServer method
        %   There are two valid syntaxes:
        %   - waitForServer(CLIENT)
        %   - waitForServer(CLIENT, TIMEOUT)

            timeout = obj.parseTimeout('waitForServer', defaultTimeout, varargin{:});
        end

        function parseSendGoalInput(obj, goalMsg, expectedMsgType, dataFormat)
        %parseSendGoalInput Validate input to sendGoal method
        %   There is only one valid syntax:
        %   - sendGoal(CLIENT, GOALMSG)

            methodName = 'sendGoal';
            setDataFormat(obj, dataFormat);
            obj.validateInputMessage(goalMsg, expectedMsgType, 'SimpleActionClient', methodName)
        end

        function timeout = parseSendGoalAndWaitInput(obj, defaultTimeout, expectedMsgType, dataFormat, goalMsg, varargin)
        %parseSendGoalAndWaitInput Parse the arguments to the sendGoalAndWait method
        %   There are two valid syntaxes:
        %   - sendGoalAndWait(CLIENT, GOALMSG)
        %   - sendGoalAndWait(CLIENT, GOALMSG, TIMEOUT)

            methodName = 'sendGoalAndWait';
            setDataFormat(obj, dataFormat);
            obj.validateInputMessage(goalMsg, expectedMsgType, 'SimpleActionClient', methodName)
            timeout = obj.parseTimeout(methodName, defaultTimeout, varargin{:});
        end

    end

    methods (Access = protected)
        function timeout = parseTimeout(~, methodName, defaultTimeout, varargin)
        %parseTimeout Parse an optional numeric timeout value

            if isempty(varargin)
                % Use default timeout if the user did not specify
                timeout = defaultTimeout;
                return;
            end

            % Otherwise, parse the timeout input
            % Note that infinity is explicitly allowed as timeout value.
            timeoutInput = varargin{1};
            validateattributes(timeoutInput, {'numeric'}, ...
                               {'scalar', 'real', 'positive', 'nonnan'}, methodName, 'timeout');
            timeout = double(timeoutInput);
        end
    end

end
