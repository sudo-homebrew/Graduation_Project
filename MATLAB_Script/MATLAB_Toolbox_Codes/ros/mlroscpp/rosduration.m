function duration = rosduration(varargin)
%ROSDURATION Create a ROS duration object
%   D = ROSDURATION returns a default ROS duration object. The properties
%   for seconds and nanoseconds are set to 0.
%
%   D = ROSDURATION(TOTALSECS) initializes the time values for seconds and
%   nanoseconds based on the TOTALSECS input. TOTALSECS represents the time
%   in seconds as a floating-point number.
%
%   D = ROSDURATION(SECS, NSECS) initializes the time values based on the SECS
%   (seconds) and NSECS (nanoseconds) inputs. Both inputs have to be
%   integers. Large values for NSECS are wrapped automatically and the
%   remainders are added to the value for SECS.
%
%   D = ROSDURATION(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as
%   Name1,Value1,...,NameN,ValueN:
%
%      "DataFormat" - Determines format of the returned duration message.
%                     The "struct" format is typically faster to use, but
%                     does not validate message field data when set, and
%                     cannot be used with arithmetic operations.
%                     Options are:
%                        "object" - Message object of the specified type
%                        "struct" - Message struct with compatible fields
%                     Default: "object"
%
%
%   Example:
%
%      % Create duration object from seconds and nanoseconds
%      d1 = ROSDURATION(200,100000)
%
%      % Create duration object for total seconds
%      d2 = ROSDURATION(500.14671)
%
%      % Add 3 seconds to the second duration and then multiply the result by 10
%      d3 = (d2 + 3) * 10
%
%      % Add the duration to a time. The result is another time.
%      t2 = rostime('now','system') + d3
%
%      % Create a duration struct and assign it in a message struct
%      dStruct = ROSDURATION(500.14671,"DataFormat","struct");
%      msg = rosmessage("std_msgs/Duration","DataFormat","struct");
%      msg.Data = dStruct;
%
%   See also ROSTIME.

%   Copyright 2016-2020 The MathWorks, Inc.
%#codegen
    coder.extrinsic('ros.codertarget.internal.getEmptyCodegenMsg');
    coder.internal.narginchk(0,4,nargin)
    if nargin > 0
        % There is at least one argument. Decide which parser to use, based on
        % the data type.
        if isnumeric(varargin{1})
            sec = varargin{1};
            [nsec,dataFormat] = parseInputs(varargin{2:end});
        else
            % First input must be numeric or scalar text
            validateattributes(varargin{1},{'string','char'},{'scalartext'},...
                               'rosduration','NV-pair');
            sec = 0;
            [nsec,dataFormat] = parseInputs(varargin{:});
        end
    else
        sec = 0;
        nsec = 0;
        dataFormat = 'object';
    end

    if isempty(coder.target)
        %% Interpreted mode
        try
            % Parse second and optional nanosecond input and return time object
            [sec, nsec] = validateDuration(sec, nsec);
            if strcmpi(dataFormat, 'struct')
                duration = ros.internal.ros.messages.ros.duration;
                duration.Sec = int32(sec);
                duration.Nsec = int32(nsec);
            else
                duration = ros.msg.Duration(sec, nsec);
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
                              'ros:mlroscpp:codegen:InvalidDataFormat', 'rosduration');
        msgStructGenFcnName = coder.const(@ros.codertarget.internal.getEmptyCodegenMsg,'ros/Duration','ros');
        msgStructGenFcn = str2func(msgStructGenFcnName);
        duration = msgStructGenFcn();
        if nargin > 0
            [sec, nsec] = validateDuration(sec, nsec);
            duration.Sec = int32(sec);
            duration.Nsec = int32(nsec);
        end
    end
end

%% Input parser
function [nsec,dataFormat] = parseInputs(varargin)
    %% Parse input parameters
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
    dataFormat = coder.internal.getParameterValue(pStruct.DataFormat,'object',varargin{:});
    dataFormat = validatestring(dataFormat, {'object', 'struct'}, 'rostime', 'DataFormat');
end

function [sec,nsec] = validateDuration(sec,nsec)
%parseInputs Parse the numeric arguments provided by the user
%   Valid syntaxes:
%   - ROSDURATION(TOTALSECS)
%   - ROSDURATION(SECS, NSECS)

% We already ascertained that there are only 1 or 2 inputs
    if isempty(nsec)
        % Syntax: ROSDURATION(TOTALSECS)
        % Parse the floating-point seconds.
        validateattributes(nsec,{'numeric'},{},'rosduration','nsecs')
        [sec, nsec] = ros.internal.Parsing.validateTotalSignedSeconds(sec, 'rosduration', 'totalSecs');
    else
        % Syntax: ROSDURATION(SECS, NSECS)
        % Parse the integer seconds and nanoseconds.
        sec = ros.internal.Parsing.validateSignedSeconds(sec, 'rosduration', 'secs');

        % Normalize the input to create a valid duration object.
        % Take the overflow from nanoseconds larger than 1e9 into account.
        [nsec, secOverflow] = ros.internal.Parsing.validateSignedNanoseconds(nsec, true, 'rosduration', 'nsecs');
        sec = sec + secOverflow;

        % The resulting time needs to be within the valid time limits.
        % Otherwise, an overflow occurred and we should throw an error.
        coder.internal.assert(ros.internal.Parsing.isValidSignedSecsNsecs(sec, nsec),...
                              'ros:mlros:duration:ResultDurationInvalid');
    end
end
