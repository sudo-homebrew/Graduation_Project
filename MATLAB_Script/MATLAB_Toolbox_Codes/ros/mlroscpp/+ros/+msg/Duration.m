classdef Duration < ros.msg.internal.TimeEntity
%Duration ROS duration representation
%
%   Duration properties:
%       Sec  - Second component of duration
%       Nsec - Nanosecond component of duration
%
%   Duration methods:
%       seconds - Scalar number representing the duration in seconds
%
%   See also ros.msg.Time.

%   Copyright 2014-2020 The MathWorks, Inc.

    properties (Constant, Access = protected)
        %ClassName - Name of concrete TimeEntity class
        ClassName = 'Duration'
    end

    properties
        %Sec - Second component of time
        Sec

        %Nsec - Nanosecond component of time
        Nsec
    end

    methods
        function obj = Duration(varargin)
        %Duration Constructor. ]
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
                    obj = ros.msg.Duration.empty(0,1);
                    return;
                elseif isstruct(varargin{1})
                    validateattributes(varargin{1},{'struct'},{'nonempty'},obj.ClassName,'time');
                    durationInput = varargin{1};

                    % Construct array of objects if necessary
                    % Preallocate array, ensuring unique handle objects
                    objArr(numel(durationInput),1) = ros.msg.Duration;
                    for k = 1:numel(durationInput)
                        elem = durationInput(k);

                        if ~isfield(elem,'sec') || ~isfield(elem,'nsec')
                            error(message('ros:mlroscpp:duration:InvalidStructureInput'));
                        end
                        if ~obj.Parser.isValidSignedSecsNsecs(elem.sec, elem.nsec)
                            error(message('ros:mlroscpp:duration:DurationOutOfRange'));
                        end
                        [sec,nsec] = obj.Parser.normalizeSecsNsecs(elem.sec,elem.nsec);
                        objArr(k,1).Sec = sec;
                        objArr(k,1).Nsec = nsec;
                    end
                    obj = objArr;
                    return
                else
                    % Input must be a double scalar representing time in
                    % seconds
                    [sec,nsec] = obj.Parser.validateTotalSignedSeconds(...
                        varargin{1}, obj.ClassName, 'totalSecs');
                end
            elseif nargin == 2
                % Two input case. Input is a tuple representing (seconds:nano-seconds)
                if ~obj.Parser.isValidSignedSecsNsecs(varargin{1}, varargin{2})
                    error(message('ros:mlroscpp:duration:InvalidTimeTuple'));
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

            validSecs = obj.Parser.validateSignedSeconds(sec, obj.ClassName, 'Sec');
            obj.Sec = validSecs;
        end

        function set.Nsec(obj, nsec)
        %set.Nsec Setter for nanoseconds value

        % For the property setter, strictly enforce the nanosecond
        % limit and do not allow overflow into seconds.
            validNsecs = obj.Parser.validateUnsignedNanoseconds(nsec, false, obj.ClassName, 'Nsec');
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
            [obj, compTime, objPos] = determineDurationObj(a, b, 'lt');

            % Compare time object to another time object
            validTime = obj.Parser.validateROSDuration(compTime, 'lt', '');
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
            [obj, compTime] = determineDurationObj(a, b, 'eq');
            validTime = obj.Parser.validateROSDuration(compTime, 'eq', '');
            isEqual = (obj.Sec == validTime.Sec) && (obj.Nsec == validTime.Nsec);
        end

        function newTimeEntity = plus(a,b)
        %PLUS Overload the '+' operator for the duration object
        %   NEWDUR = PLUS(DUR, DUROBJ) calculates the result of DUR +
        %   DUROBJ. DUROBJ is a duration object. The resulting duration
        %   is returned in NEWDUR.
        %
        %   NEWDUR = PLUS(DUR, DURNUM) calculates the result of DUR +
        %   DURNUM. DURNUM is a scalar floating-point number
        %   representing a valid duration.
        %
        %   NEWDUR = PLUS(DURNUM, DUR) calculates the result of DURNUM +
        %   DUR.
        %
        %   NEWTIME = PLUS(DUR, TIME) calculates the result of DUR +
        %   TIME. TIME is a time object. The resulting time is returned
        %   in NEWTIME.

        % We know that one of the inputs (a or b) is the Duration object.
            [obj, timeEntity, objPos] = determineDurationObj(a, b, 'plus');

            % Basic input data type validation
            validateattributes(timeEntity, {'ros.msg.Time', 'ros.msg.Duration', ...
                                'numeric', 'struct'}, {}, 'plus');

            if objPos == 1 && isa(timeEntity, 'ros.msg.Time')
                % Syntax: NEWTIME = PLUS(DUR, TIME)

                validTime = obj.Parser.validateROSTime(timeEntity, 'plus', '');

                % The resulting time needs to be within the valid limits.
                if ~obj.Parser.isValidUnsignedSecsNsecs(obj.Sec + validTime.Sec, obj.Nsec + validTime.Nsec)
                    error(message('ros:mlroscpp:duration:PlusTimeInvalid', ...
                                  t2string(validTime), t2string(obj)));
                end

                % Add duration to time and return resulting time
                newTimeEntity = ros.msg.Time(obj.Sec + validTime.Sec, ...
                                             obj.Nsec + validTime.Nsec);
            else
                % Syntax: NEWDUR = PLUS(DUR, DUROBJ)
                %         NEWDUR = PLUS(DUR, DURNUM)
                %         NEWDUR = PLUS(DURNUM, DUR)

                % Duration plus Duration is another Duration.
                % All numeric scalars and valid structs are interpreted as
                % a duration

                % Convert input to valid duration object
                validDuration = obj.Parser.validateROSDuration(timeEntity, 'plus', '');

                % The resulting duration needs to be within the valid limits.
                if ~obj.Parser.isValidSignedSecsNsecs(obj.Sec + validDuration.Sec, obj.Nsec + validDuration.Nsec)
                    error(message('ros:mlroscpp:duration:PlusDurInvalid', ...
                                  t2string(validDuration), t2string(obj)));
                end

                % Add duration to duration and return resulting duration
                newTimeEntity = ros.msg.Duration(obj.Sec + validDuration.Sec, ...
                                                 obj.Nsec + validDuration.Nsec);
            end
        end

        function newDur = minus(a, b)
        %MINUS Overload the '-' operator for the duration object
        %   NEWDUR = MINUS(DUR, DUROBJ) calculates the result of DUR - DUROBJ.
        %   DUR and DUROBJ are both duration objects. The resulting
        %   duration is returned in NEWDUR.
        %
        %   NEWDUR = MINUS(DUR, DURNUM) calculates the result of DUR -
        %   DURNUM. DURNUM is a scalar floating-point number
        %   representing a valid duration.
        %
        %   NEWDUR = MINUS(DURNUM, DUR) calculates the result of DURNUM -
        %   DUR.

        % We know that one of the inputs (a or b) is the Duration object.
        % The other input is the duration that should be subtracted.
            [obj, dur, objPos] = determineDurationObj(a, b, 'minus');

            % Convert input to valid duration object
            validDuration = obj.Parser.validateROSDuration(dur, 'minus', '');

            if objPos == 1
                % Syntax: NEWDUR = MINUS(DUR, DUROBJ)
                %         NEWDUR = MINUS(DUR, DURNUM)

                minuend = obj;
                subtrahend = validDuration;
            else
                % Syntax: NEWDUR = MINUS(DURNUM, DUR)

                minuend = validDuration;
                subtrahend = obj;
            end

            % The resulting duration needs to be within the valid limits.
            if ~obj.Parser.isValidSignedSecsNsecs(minuend.Sec - subtrahend.Sec, minuend.Nsec - subtrahend.Nsec)
                error(message('ros:mlroscpp:duration:MinusDurInvalid', ...
                              t2string(subtrahend), t2string(minuend)));
            end

            % Subtract duration from duration and return resulting duration
            newDur = ros.msg.Duration(minuend.Sec - subtrahend.Sec, ...
                                      minuend.Nsec - subtrahend.Nsec);
        end

        function newDur = mtimes(a,b)
        %MTIMES Overload the '*' operator for the duration object
        %   NEWDUR = MTIMES(DUR, DUROBJ) calculates the result of DUR *
        %   DUROBJ. Both DUR and DUROBJ are duration objects.
        %
        %   NEWDUR = MTIMES(DUR, DURNUM) calculates the result of DUR *
        %   DURNUM. DURNUM is a scalar floating-point number
        %   representing a valid duration.
        %
        %   NEWDUR = MTIMES(DURNUM, DUR) calculates the result of DURNUM *
        %   DUR.

        % We know that one of the inputs (a or b) is the Duration object.
            [obj, factor] = determineDurationObj(a, b, 'mtimes');
            validDurationFactor = obj.Parser.validateROSDuration(factor, 'mtimes', '');

            newDurSeconds = obj.seconds * validDurationFactor.seconds;

            % The resulting duration needs to be within the valid limits.
            if ~obj.Parser.isValidSignedSecsNsecs(newDurSeconds, 0)
                error(message('ros:mlroscpp:duration:TimesDurInvalid', ...
                              t2string(obj), t2string(validDurationFactor)));
            end
            newDur = rosduration(newDurSeconds);
        end

        function newDur = mrdivide(a,b)
        %MRDIVIDE Overload the '/' operator for the duration object
        %   NEWDUR = MRDIVIDE(DUR, DUROBJ) calculates the result of DUR /
        %   DUROBJ. Both DUR and DUROBJ are duration objects.
        %
        %   NEWDUR = MRDIVIDE(DUR, DURNUM) calculates the result of DUR /
        %   DURNUM. DURNUM is a scalar floating-point number
        %   representing a valid duration.
        %
        %   NEWDUR = MRDIVIDE(DURNUM, DUR) calculates the result of DURNUM /
        %   DUR.

        % We know that one of the inputs (a or b) is the Duration object.
            [obj, divInput, objPos] = determineDurationObj(a, b, 'mrdivide');
            validDurationDiv = obj.Parser.validateROSDuration(divInput, 'mrdivide', '');

            if objPos == 1
                dividend = obj;
                divisor = validDurationDiv;
            else
                dividend = validDurationDiv;
                divisor = obj;
            end

            % If divisor is 0 and dividend non-zero, the result is Inf. If
            % dividend and divisor are 0, the result is NaN. In both cases,
            % the check below detects the problem.
            newDurSeconds = dividend.seconds / divisor.seconds;

            % The resulting duration needs to be within the valid limits.
            if ~obj.Parser.isValidSignedSecsNsecs(newDurSeconds, 0)
                error(message('ros:mlroscpp:duration:DivideDurInvalid', ...
                              t2string(dividend), t2string(divisor)));
            end

            newDur = rosduration(newDurSeconds);
        end

        function newDur = uminus(obj)
        %UMINUS Overload the unary minus operator for the duration object
        %   MINUSDUR = UMINUS(DUR) negates the value of the duration
        %   object DUR and returns the result in the duration object
        %   MINUSDUR.

        % Note that the rosduration function automatically
        % normalizes any negative nanosecond values.
            newDur = rosduration(-obj.Sec, -obj.Nsec);
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
                strObj(ii).Sec = int32(obj(ii).Sec);
                strObj(ii).Nsec = int32(obj(ii).Nsec);
            end
        end
    end

    methods (Access = ?ros.msg.internal.TimeEntity)
        function reload(obj, strObj)
        %reload Called by loadobj to assign properties
        %   To provide backwards compatibility to duration objects
        %   that might have been stored in rosbags and MAT files,
        %   check if they use the old property names 'Secs' and 'Nsecs'.
        %   Also, re-interpret any negative numbers as positive.

            if isfield(strObj, 'Secs')
                sec = int32(strObj.Secs);
                nsec = int32(strObj.Nsecs);
            else
                sec = int32(strObj.Sec);
                nsec = int32(strObj.Nsec);
            end

            [sec,nsec] = obj.Parser.normalizeSecsNsecs(double(sec), double(nsec));

            % Cap the seconds value to its minimum and maximum limits
            maxSecValue = obj.Parser.MaxDurationNumeric;
            minSecValue = obj.Parser.MinDurationNumeric;
            if sec > maxSecValue
                sec = maxSecValue;
            end
            if sec < minSecValue
                sec = minSecValue;
            end

            obj.Sec = sec;
            obj.Nsec = nsec;
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.msg.internal.TimeEntity, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of duration from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.Duration.empty(0,1);
                return
            end

            % Create an empty object
            obj = ros.msg.Duration;
            obj.reload(strObj);
        end
    end
end

function [durObj, otherInput, objPos] = determineDurationObj(a,b,fcnName)
%determineDurationObj Determine which input is the duration object
%   During invocation of the arithmetic (+,-,...) and relational (>,<,...)
%   operators, the duration object might either be passed as first or second
%   argument. This function determines which one it is.
%
%   DUROBJ is the Duration object among A and B and OTHERINPUT is the
%   additional input to the arithmetic or relational operation.
%   OBJPOS indicates which argument (1-based number) contained the duration
%   object.

% We know that one of the inputs (a or b) is the Duration object.
% Determine which one it is.
    if isa(a, 'ros.msg.Duration')
        durObj = a;
        otherInput = b;
        objPos = 1;
    else
        durObj = b;
        otherInput = a;
        objPos = 2;
    end

    % The arithmetic operation is only allowed on scalar and non-empty duration
    % objects.
    validateattributes(durObj, {'ros.msg.Duration'}, {'nonempty','scalar'}, fcnName);
end
