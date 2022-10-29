classdef Parsing
%This class is for internal use only. It may be removed in the future.

%Parsing Utility functions for parsing arguments
%   This class is a collection of static functions that are helpful in
%   parsing arguments.
%
%   Parsing methods:
%       findNameValueIndex     - Returns index of first name-value occurrence
%       validateFunctionHandle - Check if the input is a function handle
%       validateFilePath       - Validate a relative or absolute file path
%       validateFolderPath     - Validate a relative or absolute folder path
%       matchString            - Match a string within a cell array
%       endsWith               - Determines whether a string ends with a substring
%       printStringList        - Create a comma-separated list of strings in a cell array

%   Copyright 2020-2021 The MathWorks, Inc.
%#codegen

    properties(Constant, Hidden)
        %MaxTimeNumeric - Maximum value that is allowed for a scalar numeric time
        %   Note that the seconds representation in ROS is limited to
        %   uint32 and that the maximum value for the nanoseconds is 1e9 - 1.
        %   Limit MaxTimeNumeric to intmax('uint32').
        %   All numeric times <= MaxTimeNumeric and >= 0 are valid.
        MaxTimeNumeric = double(intmax('uint32'))

        %MaxDurationNumeric - Maximum value that is allowed for a scalar duration
        %   Note that durations in ROS are signed int32 values.
        %   All numeric durations <= MaxDurationNumeric and >=
        %   MinDurationNumeric are valid.
        MaxDurationNumeric = double(intmax('int32'))

        %MinDurationNumeric - Minimum value that is allowed for a scalar duration
        %   Note that durations in ROS are signed int32 values.
        %   All numeric durations <= MaxDurationNumeric and >=
        %   MinDurationNumeric are valid.
        MinDurationNumeric = double(intmin('int32'))
    end

    methods (Access = public, Static)

        function nvIndex = findNameValueIndex(args, validNVNames)
        %findNameValueIndex Returns index of first name-value occurrence
        %
        %   nvIndex = ros.internal.Parsing.findNameValueIndex(args, validNVNames) returns
        %   the index of the occurrence NVINDEX of one of the valid
        %   name-value names VALIDNVNAMES in the argument list ARGS. If
        %   the name cannot be found, this function will return [].
        %   Partial and case-insensitive matching is supported.
        %
        %   Examples:
        %       % Here IDX will be 3
        %       idx = ros.internal.Parsing.findNameValueIndex(...
        %          {'192.168.1.1', 13111, 'NodeName', 'test'},...
        %          {'NodeName'})
        %
        %       % IDX will be [], because name cannot be found
        %       idx = ros.internal.Parsing.findNameValueIndex(...
        %          {'192.168.1.1', 13111},...
        %          {'NodeName'})

            nvIndex = [];

            for i = 1:length(args)
                arg = args{i};

                % Confirm that this is a string argument
                if ~ischar(arg)
                    continue;
                end

                % Try to match it to the valid names, partial and
                % case-insensitive matching is supported by validatestring
                try
                    validatestring(arg, validNVNames);
                catch
                    continue;
                end

                % Make sure there is at least one more argument more
                % (verify that this could be a valid name-value pair)
                if i == length(args)
                    continue;
                end

                % If this is still executed, the validation was successful.
                % Return the argument index.
                nvIndex = i;
                break;
            end
        end

        function [fHandle, userData] = validateFunctionHandle(funcHandle, funcName, argName)
        %validateFunctionHandle Check if the input is a function handle
        %
        %   [FHANDLE, USERDATA] = validateFunctionHandle(FUNCHANDLE)
        %   validates the input FUNCHANDLE.
        %   FUNCHANDLE can either be a scalar function handle or a cell
        %   array containing either a function handle or string as its
        %   first element and arbitrary user data. The string needs to
        %   be a function name.
        %   A scalar function handle (or string) is returned in FHANDLE
        %   and USERDATA is a cell array of parsed user data. If no
        %   user data is part of the FUNCHANDLE input, USERDATA will be
        %   an empty cell array.
        %   Otherwise, throw an error.
        %
        %   Examples:
        %       % Parse a function handle or a cell array
        %       [h, userData] = ros.internal.Parsing.validateFunctionHandle(@function1)
        %       [h, userData] = ros.internal.Parsing.validateFunctionHandle(...
        %          {@function2, 'data1', 'data2'})
        %       [h, userData] = ros.internal.Parsing.validateFunctionHandle(...
        %          {'function3', 'data1', 'data2'})
        %
        %       % This will throw an error
        %       [h, userData] = ros.internal.Parsing.validateFunctionHandle(5)

        % By default there is no user data
            userData = {};

            if nargin < 2
                funcName = '';
                argName = '';
            end

            % Input must be a function handle or a cell array. Don't allow empty inputs.
            validateattributes(funcHandle, {'function_handle', 'cell'}, {'nonempty'}, funcName, argName);

            if isa(funcHandle, 'function_handle')
                % Check if input is a scalar function handle. Assign and return.
                validateattributes(funcHandle, {'function_handle'}, {'scalar'}, funcName, argName);
                fHandle = funcHandle;
                return;
            end

            % Input is a cell array
            % First element of cell can either be a function handle or
            % text denoting a function name.
            validateattributes(funcHandle{1}, {'function_handle', 'char', 'string'}, {'nonempty'}, funcName, argName);

            if ischar(funcHandle{1}) || isstring(funcHandle{1})
                % Verify that the function name is not empty
                robotics.internal.validation.validateString(funcHandle{1}, false, funcName, argName)
            else
                % Verify that function handle is scalar
                validateattributes(funcHandle{1}, {'function_handle'}, {'scalar'}, funcName, argName)
            end

            fHandle = funcHandle{1};
            userData = funcHandle(2:end);

            % Always return empty cell array consistently
            if isempty(userData)
                userData = {};
            end
        end
    end

    %% Validations related to Time and Duration
    methods (Static)
        function validSecs = validateUnsignedSeconds(secs, funcName, varName)
        %validateUnsignedSeconds Validate that input represents valid (unsigned) seconds
        %   This validation is only applicable for integer seconds. See
        %   validateTotalUnsignedSeconds() for parsing floating-point second
        %   representations.
        %   These signed seconds can then be used in the Sec field of Time
        %   objects.

            validateattributes(secs, {'numeric'}, {'nonempty', 'scalar', 'real', 'nonnegative', ...
                                                   'nonnan', 'finite', 'integer', '<=', ros.internal.Parsing.MaxTimeNumeric}, ...
                               funcName, varName);

            % Cast output to double
            validSecs = double(secs);
        end

        function validSecs = validateSignedSeconds(secs, funcName, varName)
        %validateSignedSeconds Validate that input represents valid (signed) seconds
        %   This validation is only applicable for integer seconds. See
        %   validateTotalSignedSeconds() for parsing floating-point second
        %   representations.
        %   These signed seconds can then be used in the Sec field of
        %   Duration objects.

            validateattributes(secs, {'numeric'}, {'nonempty', 'scalar', 'real', ...
                                                   'nonnan', 'finite', 'integer', '<=', ros.internal.Parsing.MaxDurationNumeric, ...
                                                   '>=', ros.internal.Parsing.MinDurationNumeric}, ...
                               funcName, varName);

            % Cast output to double
            validSecs = double(secs);
        end

        function [validSecs, validNsecs] = validateTotalUnsignedSeconds(totalSecs, funcName, varName)
        %validateTotalUnsignedSeconds Validate that input represents valid (unsigned) floating-point seconds
        %   This validation is only applicable for floating-point (total) seconds.
        %   See validateUnsignedSeconds() for parsing integer second
        %   representations.

            [validSecs, validNsecs] = numericToUnsignedSecsNsecs(totalSecs, funcName, varName);
        end

        function [validSecs, validNsecs] = validateTotalSignedSeconds(totalSecs, funcName, varName)
        %validateTotalSignedSeconds Validate that input represents valid (signed) floating-point seconds
        %   This validation is only applicable for floating-point (total) seconds.
        %   See validateSignedSeconds() for parsing integer second
        %   representations.

            [validSecs, validNsecs] = numericToSignedSecsNsecs(totalSecs, funcName, varName);
        end

        function [validNsecs, additionalSecs] = validateUnsignedNanoseconds(nsecs, allowOverflow, funcName, varName)
        %validateUnsignedNanoseconds Validate that input represents valid (unsigned) nanoseconds
        %   These nanoseconds can then be used in the Nsec field of Time
        %   objects.
        %   ALLOWOVERFLOW determines if nanoseconds that are greater or
        %   equal to 1e9 are allowed to overflow into additional
        %   seconds (returned in ADDITIONALSECS).

            additionalSecs = 0;

            if allowOverflow
                maxValue = ros.internal.Parsing.MaxTimeNumeric;
            else
                % The nanosecond integer needs to be less than 1e9
                maxValue = 999999999;
            end

            validateattributes(nsecs, {'numeric'}, ...
                               {'nonempty', 'scalar', 'real', ...
                                'nonnegative', 'nonnan', 'finite', ...
                                'integer', '<=', maxValue}, ...
                               funcName, varName);

            validNsecs = double(nsecs);

            % Calculate overflow into seconds, if required
            if allowOverflow
                [additionalSecs, validNsecs] = ros.internal.Parsing.normalizeSecsNsecs(0, validNsecs);
            end
        end

        function [validNsecs, additionalSecs] = validateSignedNanoseconds(nsecs, allowOverflow, funcName, varName)
        %validateSignedNanoseconds Validate that input represents valid (signed) nanoseconds
        %   These nanoseconds can then be used in the Nsec field of
        %   Duration objects.
        %   ALLOWOVERFLOW determines if nanoseconds whose absolute value
        %   is greater or equal to 1e9 are allowed to overflow into additional
        %   seconds (returned in ADDITIONALSECS).

            if allowOverflow
                maxValue = ros.internal.Parsing.MaxDurationNumeric;
                minValue = ros.internal.Parsing.MinDurationNumeric;
            else
                % The nanosecond integer needs to be less than 1e9
                maxValue = 999999999;
                minValue = -999999999;
            end

            validateattributes(nsecs, {'numeric'}, ...
                               {'nonempty', 'scalar', 'real', 'nonnan', ...
                                'finite', 'integer', '<=', maxValue, ...
                                '>=', minValue}, ...
                               funcName, varName);

            validNsecs = double(nsecs);

            % Calculate overflow into seconds, if required
            [additionalSecs, validNsecs] = ros.internal.Parsing.normalizeSecsNsecs(0, validNsecs);
        end

        function validTime = validateROSTime(time, funcName, varName)
        %validateROSTime Validate any user input that represents a ROS time
        %
        %   VALIDTIME = validateROSTime(TIME, FUNCNAME, VARNAME)
        %   validates that a user input TIME represents a ROS time.
        %   TIME is valid if it is a scalar numeric value, a scalar Time
        %   object, or a scalar time struct.
        %   The output VALIDTIME will always be a valid ros.msg.Time
        %   object.
        %   Use FUNCNAME and VARNAME to customize the error messages
        %   printed by validateattributes.

        % Allow numeric or Time object input
            validateattributes(time, {'numeric', 'ros.msg.Time', 'struct'}, ...
                               {'nonempty', 'scalar'}, funcName, varName);

            % Restrict the possible values of numeric inputs.
            % Note that only non-negative times are allowed in ROS.
            if isnumeric(time)
                % Convert numeric to a valid ROS time object
                [sec,nsec] = numericToUnsignedSecsNsecs(time,funcName,varName);
                validTime = ros.msg.Time(sec,nsec);
            elseif isstruct(time)
                % Convert struct to a valid ROS time object
                if ~isfield(time,'Sec') || ~isfield(time,'Nsec')
                    error(message('ros:mlroscpp:time:InvalidStructureInput'));
                end
                validTime = ros.msg.Time(struct('sec',time.Sec,'nsec',time.Nsec));
            else
                % Return handle to input Time object
                validTime = time;
            end
        end

        function validTime = validateROSTimeStruct(time, funcName, varName)
        %validateROSTimeStruct Validate user input that represents a ROS time
        %
        %   VALIDTIME = validateROSTimeStruct(TIME, FUNCNAME, VARNAME)
        %   validates that a user input TIME represents a ROS time. TIME is
        %   valid if it is a scalar numeric value or a scalar time struct.
        %   Note that it is invalid if it is a scalar Time object.
        %   The output VALIDTIME will always be a valid rostime struct.
        %   Use FUNCNAME and VARNAME to customize the error messages
        %   printed by validateattributes.

        % Allow numeric or rostime struct
            validateattributes(time, {'numeric','struct'}, ...
                               {'nonempty','scalar'}, funcName, varName);

            % Restrict the possible values of numeric inputs.
            % Note that only non-negative times are allowed in ROS.
            if isnumeric(time)
                % Convert numeric to a valid ROS time struct
                [sec, nsec] = numericToUnsignedSecsNsecs(time,funcName,varName);
                validTime = rostime(sec,nsec,'DataFormat','struct');
            else
                % Ensure input struct contains expected fields and return
                % directly
                coder.internal.assert(isfield(time,'Sec') && isfield(time,'Nsec'),'ros:mlroscpp:time:InvalidStructureInput');
                validTime = time;
            end
        end

        function validDuration = validateROSDuration(dur, funcName, varName)
        %validateROSDuration Validate any user input that represents a ROS duration
        %
        %   VALIDDURATION = validateROSDuration(DUR, FUNCNAME, VARNAME)
        %   validates that a user input DUR represents a ROS duration.
        %   DUR is valid if it is a scalar numeric value, a scalar Duration
        %   object, or a scalar duration struct.
        %   The output VALIDDURATION will always be a valid ros.msg.Duration
        %   object.
        %   Use FUNCNAME and VARNAME to customize the error messages
        %   printed by validateattributes.

        % Allow numeric or Duration object input
            validateattributes(dur, {'numeric', 'ros.msg.Duration', 'struct'}, ...
                               {'nonempty', 'scalar'}, funcName, varName);

            % Restrict the possible values of numeric inputs.
            % Note that only non-negative durations are allowed in ROS.
            if isnumeric(dur)
                % Convert numeric to a valid ROS duration object
                [secs, nsecs] = numericToSignedSecsNsecs(dur, funcName, varName);
                validDuration = ros.msg.Duration(struct('sec', secs, 'nsec', nsecs));
            elseif isstruct(dur)
                % Convert struct to a valid ROS duration object
                if ~isfield(dur,'Sec') || ~isfield(dur,'Nsec')
                    error(message('ros:mlroscpp:duration:InvalidStructureInput'));
                end
                validDuration = ros.msg.Duration(struct('sec', dur.Sec, 'nsec', dur.Nsec));
            else
                % Return handle to input Duration object
                validDuration = dur;
            end
        end

        function isValid = isValidUnsignedSecsNsecs(secs, nsecs)
        %isValidUnsignedSecsNsecs Check if input is a valid unsigned seconds value
        %   This function assumes that the inputs are numeric scalar
        %   values. The seconds and nanoseconds will be normalized
        %   before they are checked against the numeric limits.

            isValid = false;

            % Normalize the inputs to account for overflows
            [normalSecs, ~] = ros.internal.Parsing.normalizeSecsNsecs(secs, nsecs);

            % Seconds value needs to be positive and smaller than the
            % maximum limit to be valid
            if normalSecs >= 0 && normalSecs <= ros.internal.Parsing.MaxTimeNumeric
                isValid = true;
            end
        end

        function isValid = isValidSignedSecsNsecs(secs, nsecs)
        %isValidSignedSeconds Check if input is a valid signed seconds value
        %   This function assumes that the input is a numeric scalar
        %   value.

            isValid = false;

            % Normalize the inputs to account for overflows
            [normalSecs, ~] = ros.internal.Parsing.normalizeSecsNsecs(secs, nsecs);

            % Seconds value needs to be larger than the minimum and smaller than the
            % maximum limit to be valid
            if normalSecs >= ros.internal.Parsing.MinDurationNumeric && ...
                    normalSecs <= ros.internal.Parsing.MaxDurationNumeric
                isValid = true;
            end
        end

        function [normalSecs, normalNsecs] = normalizeSecsNsecs(secs, nsecs)
        %normalizeSecsNsecs Normalize the given seconds and nanoseconds input
        %   This code deals with nanosecond numbers that are larger than 1e9 or
        %   negative. It ensures that nanosecond values are always positive and in
        %   the range of 0 <= nanoseconds <= 1e9 - 1. Any nanosecond values outside
        %   of this range will result in overflow into the seconds.
        %
        %   This implementation follows the code implemented in ROS C++:
        %   http://docs.ros.org/diamondback/api/rostime/html/time_8h_source.html#l00118

            secs = double(secs);
            nsecs = double(nsecs);

            while (nsecs >= 1e9)
                nsecs = nsecs - 1e9;
                secs = secs + 1;
            end

            while (nsecs < 0)
                nsecs = nsecs + 1e9;
                secs = secs - 1;
            end

            normalSecs = secs;
            normalNsecs = nsecs;
        end

        function timeout = validateTimeout(timeout, funcName, argName)
        %validateTimeout Parse a numeric timeout value

        % Note that infinity is explicitly allowed as timeout value.
            validateattributes(timeout, {'numeric'}, ...
                               {'scalar', 'real', 'positive', 'nonnan'}, ...
                               funcName, argName);
            timeout = double(timeout);
        end
    end

    %% Validations for string parsing and handling
    methods (Static)
        function [matchedString, index] = matchString(string, strings, partialMatching)
        %matchString Match a string within a cell array
        %   This function always does case-insensitive matching,
        %   but the user can specify if partial matches should also be supported.
        %
        %   [MATCHEDSTRING, INDEX] = ros.internal.Parsing.matchString(STRING, STRINGS, PARTIAL MATCHING)
        %   will try to match the STRING in the string cell array
        %   STRINGS. Set PARTIALMATCHING to true or false, depending if
        %   partial matching should be supported. If a match is found
        %   the result is returned in MATCHEDSTRING, along with its
        %   INDEX in the STRINGS cell array.
        %
        %   Examples:
        %      % Returns a = '/test' and b = 1
        %      [a,b] = ros.internal.Parsing.matchString('/test', ...
        %         {'/test', '/rosout', '/default'}, false);
        %
        %      % Returns a = '/rosout' and b = 2
        %      [a,b] = ros.internal.Parsing.matchString('/r', ...
        %         {'/test', '/rosout', '/default'}, true);

            if partialMatching
                try
                    matchedString = validatestring(string, strings);
                    index = find(strcmp(strings, matchedString));
                catch
                    error(message('ros:mlros:util:NoStringMatch', string));
                end
            else
                % Only do case-insensitive matching
                foundStrings = strcmpi(string, strings);
                if any(foundStrings)
                    matchedString = string;
                    index = find(foundStrings);
                else
                    error(message('ros:mlros:util:NoStringMatch', string));
                end
            end
        end

        function [status, splitStr] = endsWith(str, substr)
        %endsWith Determines whether a string ends with a substring
        %
        %   [STATUS, 'SPLITSTR'] = ros.internal.Parsing.endsWith('STRING', 'SUBSTRING')
        %   checks if the input STRING ends with the given SUBSTRING.
        %   If there is a case-insensitive match, the STATUS will be
        %   set to TRUE, otherwise STATUS will be FALSE. In case of a
        %   match, SPLITSTR will contain the part of the STRING without
        %   the SUBSTRING
        %
        %   Examples:
        %       % Returns status = true and splitStr = 'test'
        %       [status,splitStr] =
        %       ros.internal.Parsing.endsWith('testTest', 'Test');
        %
        %       % Returns status = true and splitStr = 'case'
        %       [status,splitStr] =
        %       ros.internal.Parsing.endsWith('caseTest', 'test');
        %
        %       % Returns status = false and splitStr = ''
        %       [status,splitStr] =
        %       ros.internal.Parsing.endsWith('something', 'test');

            splitStr = '';

            % A string always ends with an empty substring
            if isempty(substr)
                status = true;
                return;
            end

            strLen = length(str);
            substrLen = length(substr);

            % If the substring is longer than the actual string, the return
            % is always false.
            if strLen < substrLen
                status = false;
                return;
            end

            % Determine ending substring
            status = strcmpi(str(strLen-substrLen+1:strLen), substr);
            if status
                splitStr = str(1:strLen-substrLen);
            end
        end

        function absPath = validateFilePath(filePath)
        %validateFilePath Validate a relative or absolute file path
        %   This function ensures that a given file path refers to an
        %   existing file and will change any relative path to an
        %   absolute one.
        %
        %   ABSPATH = ros.internal.Parsing.validateFilePath('FILEPATH')
        %   checks if the input FILEPATH refers to a valid file. If it
        %   does, return the absolute path to this file in ABSPATH.
        %   If the file does not exist, or if it refers to a folder
        %   instead, this function will throw an error.
        %
        %   Examples:
        %       % Returns absolute path for file test.txt
        %       absPath =
        %       ros.internal.Parsing.validateFilePath('test.txt');

            absPath = robotics.internal.validation.validateFilePath(filePath);
        end

        function absPath = validateFolderPath(folderPath)
        %validateFolderPath Validate a relative or absolute folder path
        %   This function ensures that a given folder path refers to an
        %   existing folder and will change any relative path to an
        %   absolute one.
        %
        %   ABSPATH = ros.internal.Parsing.validateFolderPath('FOLDERPATH')
        %   checks if the input FOLDERPATH refers to a valid folder. If it
        %   does, return the absolute path to this folder in ABSPATH.
        %   If the folder does not exist, or if it refers to a file
        %   instead, this function will throw an error.
        %
        %   Examples:
        %       % Returns absolute path for folder test
        %       absPath =
        %       ros.internal.Parsing.validateFolderPath('test');

        % Validate inputs
            validateattributes(folderPath, {'char'}, {'nonempty'}, ...
                               'validateFolderPath', 'folderPath');

            % Check if folder exists and throw an error if it does not
            [status, fileStruct] = fileattrib(folderPath);
            if ~status
                error(message('ros:mlros:common:FolderNotExist', folderPath));
            end

            % Throw an error if this is not a directory
            if ~fileStruct.directory
                error(message('ros:mlros:common:FolderPathNotDir', folderPath));
            end

            % Recover absolute path to folder
            absPath = fileStruct.Name;
        end

        function s = printStringList( stringList )
        %printStringList Create a comma-separated list of strings in a cell array
        %
        %   Example:
        %       % Returns string - 'a', 'b', 'c'
        %       ros.internal.Parsing.printStringList({'a','b','c'})

            if isempty(stringList)
                s = '';
                return;
            end

            s = sprintf( '''%s'', ', stringList{:});
            if length(s) > 1
                % Remove the trailing comma
                s(end-1:end) = [];
            end
        end

    end
end

function [secs,nsecs] = numericToUnsignedSecsNsecs(time, funcName, varName)
%numericToUnsignedSecsNsecs Convert numeric value to unsigned seconds and nanoseconds

% The maximum value for seconds is MaxTimeNumeric, so allow any fractional
% part < 1 beyond that for nanoseconds.
    validateattributes(time, {'numeric'}, ...
                       {'nonempty', 'scalar', 'real', 'nonnegative', ...
                        'nonnan', 'finite', ...
                        '<', ros.internal.Parsing.MaxTimeNumeric + 1}, ...
                       funcName, varName)

    % We use the same conversion to seconds and nanoseconds that ROS C++ uses:
    % See http://docs.ros.org/jade/api/rostime/html/time_8h_source.html
    time = double(time);

    secs = floor(time);
    nsecs = round((time - secs) * 1e9);

    % Normalize the second and nanosecond values
    [secs,nsecs] = ros.internal.Parsing.normalizeSecsNsecs(secs,nsecs);
end

function [secs,nsecs] = numericToSignedSecsNsecs(time, funcName, varName)
%numericToSignedSecsNsecs Convert numeric value to signed seconds and nanoseconds

% The maximum value for seconds is MaxDurationNumeric, so allow any fractional
% part < 1 beyond that for nanoseconds.
% The minimum value for seconds is MinDurationNumeric. Any negative
% fractional part beyond that would result in an invalid duration on
% normalization.
    validateattributes(time, {'numeric'}, ...
                       {'nonempty', 'scalar', 'real', 'nonnan', 'finite', ...
                        '<', ros.internal.Parsing.MaxDurationNumeric + 1, ...
                        '>=', ros.internal.Parsing.MinDurationNumeric}, ...
                       funcName, varName)

    % We use the same conversion to seconds and nanoseconds that ROS C++ uses:
    % See http://docs.ros.org/jade/api/rostime/html/duration_8h_source.html
    time = double(time);

    % Take floor towards 0, so negative numbers are handled correctly
    secs = fix(time);
    nsecs = round((time - secs) * 1e9);

    % Normalize the second and nanosecond values
    [secs,nsecs] = ros.internal.Parsing.normalizeSecsNsecs(secs,nsecs);
end
