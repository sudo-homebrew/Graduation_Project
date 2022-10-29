function varargout = rostime(varargin)
%ROSTIME Access ROS time functionality
%   T = ROSTIME returns a default ROS time object. The properties for seconds
%   and nanoseconds are set to 0.
%
%   T = ROSTIME(TOTALSECS) initializes the time values for seconds and
%   nanoseconds based on the TOTALSECS input. TOTALSECS represents the time
%   in seconds as a floating-point number.
%
%   T = ROSTIME(SECS, NSECS) initializes the time values based on the SECS
%   (seconds) and NSECS (nanoseconds) inputs. Both inputs have to be
%   integers. Large values for NSECS are wrapped automatically and the
%   remainders are added to the value for SECS.
%
%   T = ROSTIME("now") returns the current ROS time in T. If the
%   "/use_sim_time" ROS parameter is set to true, this returns the
%   simulation time published on the "/clock" topic. Otherwise, the
%   function returns your computer's system time. T is a
%   ros.msg.Time object. If no output argument is given, the
%   current time (in seconds) is printed to the screen.
%
%   [T, ISSIMTIME] = ROSTIME("now") also returns a boolean ISSIMTIME
%   that indicates if T is simulation time (true) or system time (false).
%
%   T = ROSTIME("now","system") always returns your machine's system
%   time, even if ROS publishes simulation time on the "/clock" topic.
%   If no output argument is given, the system time (in seconds) is
%   printed to the screen.
%
%   T = ROSTIME(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "DataFormat" - Determines format of the returned time message.
%                     The "struct" format is typically faster to use, but
%                     does not validate message field data when set, and
%                     cannot be used with arithmetic operations.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%   The system time in ROS follows the Unix / POSIX time standard. POSIX
%   time is defined as the time that has elapsed since 00:00:00 Coordinated
%   Universal Time (UTC), 1 January 1970, not counting leap seconds.
%
%   ROSTIME can be used to timestamp messages or to measure time in the
%   ROS network.
%
%
%   Example:
%
%      % Create time object from seconds and nanoseconds
%      t1 = ROSTIME(1500,200000)
%
%      % Create time object for total seconds
%      t2 = ROSTIME(500.14671)
%
%      % Add 3 seconds to the time and calculate the duration between the
%      % two times
%      t2 = t2 + 3
%      d = t1 - t2
%
%      % Show the current ROS time
%      ROSTIME now
%
%      % Return the current time in a time structure
%      tStruct = ROSTIME("now","DataFormat","struct")
%
%      % Timestamp message data with current system time
%      % Use struct message format for better performance
%      point = rosmessage("geometry_msgs/PointStamped","DataFormat","struct");
%      point.Header.Stamp = ROSTIME("now","system","DataFormat","struct");
%      point.Point.X = 5;
%
%   See also ROSDURATION.

%   Copyright 2014-2021 The MathWorks, Inc.
%#codegen
    coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');
    coder.internal.narginchk(0,4,nargin)

    %% Parse inputs
    if nargin > 0
        % There is at least one argument. Decide which parser to use, based on
        % the data type.
        validateattributes(varargin{1}, {'numeric','char','string'}, {}, 'rostime');
        if isnumeric(varargin{1})
            % varargin{1} is seconds value
            sec = varargin{1};
            [nsec,dataFormat] = parseInputsNumeric(varargin{2:end});
            isNumericInput = true;
        else
            % varargin{1} must be 'now' or 'DataFormat'
            if strcmpi(varargin{1},'now')
                % rostime(<operation>,...
                operation = varargin{1};
                validatestring(operation,{'now'}, 'rostime', 'operation');
                [provider,dataFormat] = parseInputsText(varargin{2:end});
                isNumericInput = false;
            else
                % rostime('DataFormat',...
                validatestring(varargin{1},{'DataFormat'}, 'rostime', 'DataFormat');
                [provider,dataFormat] = parseInputsText(varargin{:});
                isNumericInput = true;
                sec = 0;
                nsec = 0;
            end
        end
    else
        % No input case is treated as sec = 0, nsec = 0
        isNumericInput = true;
        sec = 0;
        nsec = 0;
        dataFormat = 'object';
    end

    if isempty(coder.target)
        %% Interpreted mode
        try
            % Construct time message based on input type
            if isNumericInput
                % Parse second and optional nanosecond input and return time object
                [sec, nsec] = validateTime(sec, nsec);
                if strcmpi(dataFormat, 'struct')
                    msg = ros.internal.ros.messages.ros.time;
                    msg.Sec = uint32(sec);
                    msg.Nsec = uint32(nsec);
                else
                    msg = ros.msg.Time(sec, nsec);
                end
                varargout{1} = msg;
                return
            else
                % rostime('now',...
                isSystemTime = validateOperation(operation, provider);
                if nargout == 0
                    rostimeImpl(isSystemTime,dataFormat);
                else
                    [varargout{1:nargout}] = rostimeImpl(isSystemTime,dataFormat);
                end
            end
        catch ex
            % Save stack traces and exception causes internally, but do not
            % print them to the console
            rosex = ros.internal.ROSException.fromException(ex);
            throwAsCaller(rosex);
        end
    else
        %% Codegen
        coder.internal.assert(strcmp(dataFormat, 'struct'),...
                              'ros:mlroscpp:codegen:InvalidDataFormat', 'rostime');

        %% Generate message structure
        msgStructGenFcnName = coder.const(@ros.codertarget.internal.getEmptyCodegenMsg,'ros/Time','ros');
        msgStructGenFcn = str2func(msgStructGenFcnName);
        msgStruct = msgStructGenFcn();
        if nargin > 0
            % There is at least one argument. Decide which based on
            % the data type.
            if isNumericInput
                %% rostime(1,...)
                [sec, nsecVal] = validateTime(sec, nsec);
                msgStruct.Sec = uint32(sec);
                msgStruct.Nsec = uint32(nsecVal);
            else
                %% rostime('now',...)
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                coder.updateBuildInfo('addIncludePaths',srcFolder);
                coder.updateBuildInfo('addIncludeFiles','mlroscpp_time.h',srcFolder);
                coder.cinclude('mlroscpp_time.h');
                isSystemTime = validateOperation(operation, provider);
                isSimTime = coder.nullcopy(false);
                isSimTime = coder.ceval('time2struct',coder.wref(msgStruct),isSystemTime);
            end
        end
        if nargout > 0
            varargout{1} = msgStruct;
        end
        if nargout > 1
            varargout{2} = isSimTime;
        end
    end
end

%% Local functions
function [provider,dataFormat] = parseInputsText(varargin)
% Parse parameters for rostime('now',...) case
    opArgs = {'provider'};
    % Define the parameter names.
    NVPairNames = {'DataFormat'};
    % Select parsing options.
    pOpts = struct( ...
        'CaseSensitivity',false, ...
        'PartialMatching','unique', ...
        'StructExpand',false, ...
        'IgnoreNulls',true, ...
        'SupportOverrides',false);
    pStruct = coder.internal.parseInputs(opArgs,NVPairNames,pOpts,varargin{:});
    provider = coder.internal.getParameterValue(pStruct.provider,'',varargin{:});
    dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'object',varargin{:});
    dataFormat = validatestring(dataFormat, {'object', 'struct'}, 'rostime', 'DataFormat');
end

function [nsec,dataFormat] = parseInputsNumeric(varargin)
% Parse parameters for rostime(1.5,...) case
    opArgs = {'nsec'};
    % Define the parameter names.
    NVPairNames = {'DataFormat'};
    % Select parsing options.
    pOpts = struct( ...
        'CaseSensitivity',false, ...
        'PartialMatching','unique', ...
        'StructExpand',false, ...
        'IgnoreNulls',true, ...
        'SupportOverrides',false);
    pStruct = coder.internal.parseInputs(opArgs,NVPairNames,pOpts,varargin{:});
    nsec = coder.internal.getParameterValue(pStruct.nsec,[],varargin{:});
    % if nsec == [], test for scalar fails
    if ~(isequal(class(nsec),'double') && isempty(nsec))
        validateattributes(nsec,{'numeric'},{'scalar'},'rostime','nsecs');
    end
    dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'object',varargin{:});
    dataFormat = validatestring(dataFormat, {'object', 'struct'}, 'rostime', 'DataFormat');
end

function [sec, nsec] = validateTime(sec,nsec)
%parseNumericInputs Parse the numeric arguments provided by the user
%   Valid syntaxes:
%   - ROSTIME(TOTALSECS)
%   - ROSTIME(SECS, NSECS)

% We already ascertained that there are only 1 or 2 inputs
    if isempty(nsec)
        % Syntax: ROSTIME(TOTALSECS)
        % Parse the floating-point seconds.
        [sec, nsec] = ros.internal.Parsing.validateTotalUnsignedSeconds(sec, 'rostime', 'totalSecs');
    else
        % Syntax: ROSTIME(SECS, NSECS)
        % Parse the integer seconds and nanoseconds.
        sec = ros.internal.Parsing.validateUnsignedSeconds(sec, 'rostime', 'sec');

        % Normalize the input to create a valid time object.
        % Take the overflow from nanoseconds larger than 1e9 into account.
        [nsec, secOverflow] = ros.internal.Parsing.validateSignedNanoseconds(nsec, true, 'rostime', 'nsecs');
        sec = sec + secOverflow;

        % The resulting time needs to be within the valid time limits.
        % Otherwise, an overflow occurred and we should throw an error.
        coder.internal.assert(ros.internal.Parsing.isValidUnsignedSecsNsecs(sec, nsec),...
                              'ros:mlros:time:ResultTimeInvalid');
    end
end

function isSystemTime = validateOperation(operation, provider)
%parseOperationInputs Parse the string arguments provided by the user
%   Valid syntaxes:
%   - ROSTIME('now')
%   - ROSTIME('now', 'system')

% We already ascertained that there is at least one input
    isSystemTime = false;
    validatestring(operation, {'now'}, 'rostime', 'operation');
    if ~isempty(provider)
        % Syntax: ROSTIME('now', 'system')
        validatestring(provider, {'system'}, 'rostime', 'provider');

        % If the validation passes, we know that the user wants system time
        isSystemTime = true;
    end
end

function varargout = rostimeImpl(isSystemTime,dataFormat)
%rostimeImpl Actual implementation of rostime functionality
%   This retrieves either the system or current ROS time.

% The operation should be parsed and valid
    if isSystemTime
        % Return or print system time
        currentTime = getCurrentSystemTime;
    else
        % Let ROS decide what time to return (system or simulation time)
        [currentTime, isSimTime] = getCurrentROSTime;
        varargout{2} = isSimTime;
    end
    if nargout == 0
        printTime(currentTime)
        return
    end
    if strcmp(dataFormat,'struct')
        msg = ros.internal.ros.messages.ros.time;
        msg.Sec = uint32(currentTime.Sec);
        msg.Nsec = uint32(currentTime.Nsec);
        varargout{1} = msg;
    else
        varargout{1} = currentTime;
    end
end

function [currentTime, isSimTime] = getCurrentROSTime
%getCurrentROSTime Retrieve the current ROS time

% Return the current time
    timeObj = ros.internal.Time;
    currentTime = timeObj.CurrentTime;
    isSimTime = timeObj.IsSimulationTime;
end

function sysTime = getCurrentSystemTime
%getCurrentSystemTime Retrieve Unix system time as ROS time structure
% Construct time object
    timeObj = ros.internal.Time;
    sysTime = timeObj.CurrentSystemTime;
end

function printTime(currentTime)
%printTime Print time (in seconds) to the console

    if isempty(currentTime)
        secs = 0;
    else
        secs = double(currentTime.Sec) + double(currentTime.Nsec) / 1e9;
    end

    disp([num2str(secs) ' ' message('ros:mlros:time:CurrentTimeSeconds').getString]);
end
