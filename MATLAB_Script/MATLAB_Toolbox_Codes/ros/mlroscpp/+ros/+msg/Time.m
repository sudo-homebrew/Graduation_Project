classdef Time < ros.msg.internal.TimeEntity
%Time ROS Time representation
%
%   Time properties:
%       Sec  - Second component of time
%       Nsec - Nanosecond component of time
%
%   Time methods:
%       seconds - Scalar number representing the time in seconds
%
%   See also rostime, ros.msg.Duration.

%   Copyright 2014-2020 The MathWorks, Inc.

    properties (Constant, Access = protected)
        %ClassName - Name of concrete TimeEntity class
        ClassName = 'Time'
    end

    properties
        %Sec - Second component of time
        Sec

        %Nsec - Nanosecond component of time
        Nsec
    end

    methods
        function obj = Time(varargin)
        %Time Constructor
            narginchk(0,2);
            if nargin == 0
                % No input case. Set sec:nsec to zero
                sec = 0;
                nsec = 0;
            elseif nargin == 1
                % One input case. Input can be a
                % struct('sec',sec,'nsec',nsec) or an array of
                % struct('sec',sec,'nsec',nsec) or a totalSeconds value of
                % type double

                % Construct appropriate empty array
                if isempty(varargin{1})
                    obj = ros.msg.Time.empty(0,1);
                    return;
                elseif isstruct(varargin{1})
                    % struct inputs are handled here
                    validateattributes(varargin{1},{'struct'},{'nonempty'},obj.ClassName,'time');
                    timeInput = varargin{1};

                    % Construct array of objects
                    % Preallocate array, ensuring unique handle objects
                    objArr(numel(timeInput),1) = ros.msg.Time;
                    for k = 1:numel(timeInput)
                        elem = timeInput(k);

                        if ~isfield(elem,'sec') || ~isfield(elem,'nsec')
                            error(message('ros:mlroscpp:time:InvalidStructureInput'));
                        end
                        if ~obj.Parser.isValidUnsignedSecsNsecs(elem.sec, elem.nsec)
                            error(message('ros:mlroscpp:time:TimeOutOfRange'));
                        end
                        [sec,nsec] = obj.Parser.normalizeSecsNsecs(elem.sec, elem.nsec);
                        objArr(k).Sec = sec;
                        objArr(k).Nsec = nsec;
                    end
                    obj = objArr;
                    return
                else
                    % Input must be a double scalar representing time in
                    % seconds
                    [sec,nsec] = obj.Parser.validateTotalUnsignedSeconds(...
                        varargin{1}, obj.ClassName, 'totalSecs');
                end
            elseif nargin == 2
                % Two input case. Input is a tuple representing (seconds:nano-seconds)
                if ~obj.Parser.isValidUnsignedSecsNsecs(varargin{1}, varargin{2})
                    error(message('ros:mlroscpp:time:InvalidTimeTuple'));
                end
                [sec,nsec] = obj.Parser.normalizeSecsNsecs(double(varargin{1}), double(varargin{2}));
            end
            obj.Sec = sec;
            obj.Nsec = nsec;
        end
    end


    %% Arithmetic and Relational Operations
    methods
        function set.Sec(obj, sec)
        %set.Sec Setter for seconds value

            validSecs = ros.internal.Parsing.validateUnsignedSeconds(sec,...
                                                              obj.ClassName, 'Sec');
            obj.Sec = validSecs;
        end

        function set.Nsec(obj, nsec)
        %set.Nsec Setter for nanoseconds value

        % For the property setter, strictly enforce the nanosecond
        % limit and do not allow overflow into seconds.
            validNsecs = ros.internal.Parsing.validateUnsignedNanoseconds(...
                nsec, false, obj.ClassName, 'Nsec');
            obj.Nsec = validNsecs;
        end

        function isLessThan = lt(a, b)
        %LT Overload the '<' operator for the time object
        %   ISLESSTHAN = LT(TIME, TIMEOBJ) returns TRUE if TIME represents a
        %   time that is less than TIMEOBJ. TIMEOBJ is a time object.
        %
        %   ISLESSTHAN = LT(TIME, TIMENUM) returns TRUE if TIME represents a
        %   time that is less than TIMENUM. TIMENUM is a scalar floating-point number
        %   representing a valid time.
        %
        %   ISLESSTHAN = LT(TIMENUM, TIME) returns TRUE if TIMENUM represents a
        %   time that is less than TIME.
            [obj, compTime, objPos] = determineTimeObj(a, b, 'lt');

            % Compare time object to another time object
            validTime = ros.internal.Parsing.validateROSTime(compTime, 'lt', '');
            if objPos == 1
                % Syntax: LT(TIME, TIMEOBJ)
                %         LT(TIME, TIMENUM)
                isLessThan = seconds(obj) < seconds(validTime);
            else
                % Syntax: LT(TIMENUM, TIME)
                isLessThan = seconds(validTime) < seconds(obj);
            end
        end

        function isEqual = eq(a,b)
        %EQ Overload the '==' operator for the time object
        %   ISEQUAL = EQ(TIME, TIMEOBJ) returns TRUE if TIME represents
        %   the same time as TIMEOBJ. TIMEOBJ is a time object.
        %
        %   ISEQUAL = EQ(TIME, TIMENUM) returns TRUE if TIME represents
        %   the same time as TIMENUM. TIMENUM is a
        %   scalar floating-point number representing a valid time.
        %
        %   ISEQUAL = EQ(TIMENUM, TIME) returns TRUE if TIMENUM represents
        %   the same time as TIME.
            [obj, compTime] = determineTimeObj(a, b, 'eq');
            validTime = obj.Parser.validateROSTime(compTime, 'eq', '');
            isEqual = (obj.Sec == validTime.Sec) && (obj.Nsec == validTime.Nsec);
        end

        function newTime = plus(a, b)
        %PLUS Overload the '+' operator for the time object
        %   NEWTIME = PLUS(TIME, DUR) calculates the result of TIME + DUR.
        %   DUR is a duration object. The resulting time is returned in NEWTIME.
        %
        %   NEWTIME = PLUS(TIME, DURNUM) calculates the result of TIME +
        %   DURNUM. DURNUM is a scalar floating-point number
        %   representing a valid duration.
        %
        %   NEWTIME = PLUS(DURNUM, TIME) calculates the result of DURNUM +
        %   TIME.

        % We know that one of the inputs (a or b) is the Time object.
        % The other input is the duration that should be added.
            [obj, dur] = determineTimeObj(a, b, 'plus');

            % Convert input to valid duration object
            validDuration = obj.Parser.validateROSDuration(dur, 'plus', '');

            % The resulting time needs to be within the valid time limits.
            % Otherwise, an overflow occurred and we should throw an error.
            if ~obj.Parser.isValidUnsignedSecsNsecs(obj.Sec + validDuration.Sec, obj.Nsec + validDuration.Nsec)
                error(message('ros:mlroscpp:time:PlusInvalid', ...
                              t2string(validDuration), ...
                              t2string(obj)));
            end

            % Add duration to time and return result
            [sec, nsec] = obj.Parser.normalizeSecsNsecs(...
                obj.Sec + validDuration.Sec, obj.Nsec + validDuration.Nsec);
            newTime = ros.msg.Time(struct('sec',sec,'nsec',nsec));
        end

        function newTimeEntity = minus(a, b)
        %MINUS Overload the '-' operator for the time object
        %   NEWTIME = MINUS(TIME, DUR) calculates the result of
        %   TIME - DUR. DUR is a duration object.
        %   The resulting time is returned in NEWTIME.
        %
        %   NEWTIME = MINUS(TIME, DURNUM) calculates the result of TIME -
        %   DURNUM. DURNUM is a scalar floating-point number
        %   representing a valid duration.
        %
        %   DUR = MINUS(TIME, TIMEOBJ) calculates the result of
        %   TIME - TIMEOBJ. TIMEOBJ is a time object. The resulting
        %   duration is returned in DUR.
        %
        %   DUR = MINUS(TIMENUM, TIME) calculates the result of TIMENUM -
        %   TIME. TIMENUM is a scalar floating-point number
        %   representing a valid time.

        % We know that one of the inputs (a or b) is the Time object.
            [obj, timeEntity, objPos] = determineTimeObj(a, b, 'minus');

            % Basic input data type validation
            validateattributes(timeEntity, {'ros.msg.Time', 'ros.msg.Duration', ...
                                'numeric', 'struct'}, {}, 'minus');

            if objPos == 1
                % Syntax: DUR = MINUS(TIME, TIMEOBJ)
                %         NEWTIME = MINUS(TIME, DUR)
                %         NEWTIME = MINUS(TIME, DURNUM)
                % The second input could be either a Time object, a Duration object,
                % or a numeric scalar.
                if isa(timeEntity, 'ros.msg.Time')
                    % Syntax: DUR = MINUS(TIME, TIMEOBJ)
                    % Time minus Time is a Duration

                    %validTime = obj.Parser.validateROSTime(timeEntity, 'minus', '');
                    % The resulting duration needs to be within the valid limits.
                    if ~obj.Parser.isValidSignedSecsNsecs(obj.Sec - timeEntity.Sec,...
                                                          obj.Nsec - timeEntity.Nsec)
                        error(message('ros:mlroscpp:time:MinusDurInvalid', ...
                                      t2string(timeEntity), ...
                                      t2string(obj)));
                    end

                    % Subtract time from time and return resulting duration
                    newTimeEntity = ros.msg.Duration(obj.Sec - timeEntity.Sec,...
                                                     obj.Nsec - timeEntity.Nsec);
                else
                    % Syntax: NEWTIME = MINUS(TIME, DUR)
                    %         NEWTIME = MINUS(TIME, DURNUM)
                    % Time minus Duration is a Time. All numeric scalars
                    % and valid structs are interpreted as a duration

                    % Convert input to valid duration object
                    validDuration = obj.Parser.validateROSDuration(timeEntity, 'minus', '');

                    % The resulting time needs to be within the valid time limits.
                    % Otherwise, an overflow occurred and we should throw an error.
                    if ~obj.Parser.isValidUnsignedSecsNsecs(obj.Sec - validDuration.Sec, obj.Nsec - validDuration.Nsec)
                        error(message('ros:mlroscpp:time:MinusTimeInvalid', ...
                                      t2string(validDuration), ...
                                      t2string(obj)));
                    end

                    % Subtract duration from time and return resulting time
                    [sec, nsec] = obj.Parser.normalizeSecsNsecs(...
                        obj.Sec - validDuration.Sec,...
                        obj.Nsec - validDuration.Nsec);
                    newTimeEntity = ros.msg.Time(struct('sec',sec,'nsec',nsec));
                end
            else
                % Syntax: DUR = MINUS(TIMENUM, TIME)
                % The first input is always interpreted as a floating-point
                % value representing a valid time value.

                validTime = obj.Parser.validateROSTime(timeEntity, 'minus', '');
                % The resulting duration needs to be within the valid limits.
                if ~obj.Parser.isValidSignedSecsNsecs(validTime.Sec - obj.Sec, validTime.Nsec - obj.Nsec)
                    error(message('ros:mlroscpp:time:MinusDurInvalid', ...
                                  t2string(obj), ...
                                  t2string(validTime)));
                end

                % Subtract time from time and return resulting duration
                [sec, nsec] = obj.Parser.normalizeSecsNsecs(...
                    validTime.Sec - obj.Sec,...
                    validTime.Nsec - obj.Nsec);
                newTimeEntity = ros.msg.Duration(struct('sec',sec,'nsec',nsec));
            end
        end
    end

    methods (Access = {?ros.Message, ?ros.msg.internal.TimeEntity})
        function strObj = toROSStruct(obj)
        %toROSStruct Convert the object to internal ROS struct

        % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end

            strObj = struct.empty(numel(obj),0);
            for ii = 1:numel(obj)
                strObj(ii).Sec = uint32(obj(ii).Sec);
                strObj(ii).Nsec = uint32(obj(ii).Nsec);
            end
        end
    end

    methods (Access = ?ros.msg.internal.TimeEntity)
        function reload(obj, strObj)
        %reload Called by loadobj to assign properties
        %   To provide backwards compatibility to time objects that
        %   might have been stored in rosbags and MAT files,
        %   check if they use the old property names 'Secs' and 'Nsecs'.
        %   Also, re-interpret any negative numbers as positive.

            if isfield(strObj, 'Secs')
                sec = typecast(int32(strObj.Secs), 'uint32');
                nsec = typecast(int32(strObj.Nsecs), 'uint32');
            else
                sec = typecast(int32(strObj.Sec), 'uint32');
                nsec = typecast(int32(strObj.Nsec), 'uint32');
            end

            [sec,nsec] = obj.Parser.normalizeSecsNsecs(double(sec), double(nsec));

            % Cap the seconds value
            maxSecValue = obj.Parser.MaxTimeNumeric;
            if sec > maxSecValue
                sec = maxSecValue;
            end

            obj.Sec = sec;
            obj.Nsec = nsec;
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.msg.internal.TimeEntity, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of time from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.Time.empty(0,1);
                return
            end

            % Create an empty object
            obj = ros.msg.Time;
            obj.reload(strObj);
        end
    end
end

function [timeObj, otherInput, objPos] = determineTimeObj(a,b,fcnName)
%determineTimeObj Determine which input is the time object
%   During invocation of the arithmetic (+,-,...) and relational (>,<,...)
%   operators, the time object might either be passed as first or second
%   argument. This function determines which one it is.
%
%   TIMEOBJ is the Time object among A and B and OTHERINPUT is the
%   additional input to the arithmetic or relational operation.
%   OBJPOS indicates which argument (1-based number) contained the time
%   object.

% We know that one of the inputs (a or b) is the Time object.
% Determine which one it is.
    if isa(a, 'ros.msg.Time')
        timeObj = a;
        otherInput = b;
        objPos = 1;
    else
        timeObj = b;
        otherInput = a;
        objPos = 2;
    end

    % The arithmetic operation is only allowed on scalar and non-empty duration
    % objects.
    validateattributes(timeObj, {'ros.msg.Time'}, {'nonempty','scalar'}, fcnName);
end
