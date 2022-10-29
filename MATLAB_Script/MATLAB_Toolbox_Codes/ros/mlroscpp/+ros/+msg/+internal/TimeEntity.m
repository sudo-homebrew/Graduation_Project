classdef (Abstract) TimeEntity < matlab.mixin.CustomDisplay & ...
        matlab.mixin.internal.CompactDisplay & ...
        matlab.mixin.Copyable
    %This class is for internal use only. It may be removed in the future.

    %TimeEntity Base class for ROS representation of time and duration.
    %
    %   TimeEntity properties:
    %       Sec  - Second component of time
    %       Nsec - Nanosecond component of time
    %
    %   See also ros.msg.Time, ros.msg.Duration.

    %   Copyright 2014-2020 The MathWorks, Inc.

    properties (Access = {?ros.msg.internal.TimeEntity, ?matlab.unittest.TestCase})
        %Parser - The parser for time and duration entities
        Parser = ros.internal.Parsing
    end

    properties (Abstract)
        %Sec - Second component of time
        Sec

        %Nsec - Nanosecond component of time
        Nsec
    end

    properties (Constant, Hidden)
        PropertyList = {'Sec', 'Nsec'} % List of non-constant message properties
        ROSPropertyList = {'sec', 'nsec'} % List of non-constant ROS message properties
        PropertyMessageTypes = {'', ''} % Types of contained nested messages
    end

    properties (Abstract, Constant, Access = protected)
        %ClassName - Name of concrete TimeEntity class
        ClassName
    end

    methods
        function secs = seconds(obj)
        %SECONDS Scalar number representing the time or duration in seconds
        %   SECS = SECONDS(TIME) calculates the scalar number SECS (in
        %   seconds) that represents the same value as the time object
        %   TIME.
        %
        %   SECS = SECONDS(DURATION) calculates the scalar number SECS (in
        %   seconds) that represents the same value as the duration object
        %   DURATION.

            secs = obj.Sec + 1e-9 * obj.Nsec;
        end
    end

    %% Abstract relational operators
    % Subclasses must implement lt and eq operators. All other relational
    % operators can be expressed in terms of lt and eq.
    methods (Abstract)
        isLessThan = lt(a, b);
        isEqual = eq(a,b);
    end

    %% Relational operators
    methods
        function isGreaterThan = gt(a, b)
        %GT Overload the '>' operator for the time object
        %   ISGREATERTHAN = GT(TIME, TIMEOBJ) returns TRUE if TIME represents a
        %   time that is greater than TIMEOBJ. TIMEOBJ is a time object.
        %
        %   ISGREATERTHAN = GT(TIME, TIMENUM) returns TRUE if TIME represents a
        %   time that is greater than TIMENUM. TIMENUM is a scalar floating-point number
        %   representing a valid time.
        %
        %   ISGREATERTHAN = GT(TIMENUM, TIME) returns TRUE if TIMENUM represents a
        %   time that is greater than TIME.
            isGreaterThan = ~le(a,b);
        end

        function isLessThanOrEqual = le(a, b)
        %LE Overload the '<=' operator for the time object
        %   ISLESSOREQUAL = LE(TIME, TIMEOBJ) returns TRUE if TIME represents a
        %   time that is less than or equal to TIMEOBJ. TIMEOBJ is a time object.
        %
        %   ISLESSOREQUAL = LE(TIME, TIMENUM) returns TRUE if TIME represents a
        %   time that is less than or equal to TIMENUM. TIMENUM is a
        %   scalar floating-point number representing a valid time.
        %
        %   ISLESSOREQUAL = LE(TIMENUM, TIME) returns TRUE if TIMENUM represents a
        %   time that is less than or equal to TIME.
            isLessThanOrEqual = lt(a,b) || eq(a,b);
        end

        function isGreaterOrEqual = ge(a, b)
        %GE Overload the '>=' operator for the time object
        %   ISGREATEROREQUAL = GE(TIME, TIMEOBJ) returns TRUE if TIME represents a
        %   time that is greater than or equal to TIMEOBJ. TIMEOBJ is a time object.
        %
        %   ISGREATEROREQUAL = GE(TIME, TIMENUM) returns TRUE if TIME represents a
        %   time that is greater than or equal to TIMENUM. TIMENUM is a
        %   scalar floating-point number representing a valid time.
        %
        %   ISGREATEROREQUAL = GE(TIMENUM, TIME) returns TRUE if TIMENUM represents a
        %   time that is greater than or equal to TIME.
            isGreaterOrEqual = ~lt(a,b);
        end

        function isNotEqual = ne(a,b)
        %NE Overload the '~=' operator for the time object
        %   ISNOTEQUAL = NE(TIME, TIMEOBJ) returns TRUE if TIME represents
        %   a different time from TIMEOBJ. TIMEOBJ is a time object.
        %
        %   ISNOTEQUAL = NE(TIME, TIMENUM) returns TRUE if TIME represents
        %   a different time from TIMENUM. TIMENUM is a
        %   scalar floating-point number representing a valid time.
        %
        %   ISNOTEQUAL = NE(TIMENUM, TIME) returns TRUE if TIMENUM represents
        %   a different time from TIME.
            isNotEqual = ~eq(a,b);
        end
    end

    methods (Hidden)
        function s = toStruct(obj)
        %toStruct Convert the message object to a struct

        % Fast execution for scalar objects
            if isscalar(obj)
                s = obj.saveobj;
                return
            end

            % Special handling for empty objects / arrays
            if isempty(obj)
                s = struct.empty(0,1);
                return
            end

            % Special handling for object array inputs
            % Pre-allocate structure array for performance.
            s = repmat(obj(1).saveobj, size(obj));
            for k = 2:numel(obj)
                s(k) = obj(k).saveobj;
            end
        end

        function fromStruct(obj, s)
        %fromStruct Reload the object from a struct representation
            obj.reload(s);
        end
    end

    methods (Access = {?ros.Message, ?ros.msg.internal.TimeEntity})
        function strObj = saveobj(obj)
        %saveobj Implements saving of message to MAT file

        % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end

            strObj.Sec = obj.Sec;
            strObj.Nsec = obj.Nsec;
        end

        function valid = compareTypeStruct(~, strObj)
        %compareTypeStruct Check scalar internal ROS struct has
        %expected structure

            valid = isfield(strObj, 'Sec') && isfield(strObj, 'Nsec');
        end
    end

    methods (Access = protected)
        function cpObj = copyElement(obj)
        %copyElement Implements deep copy behavior for time entity
        % Call default copy method for shallow copy
            cpObj = copyElement@matlab.mixin.Copyable(obj);

            % Iterate over all properties and copy them recursively
            cpObj.Sec = obj.Sec;
            cpObj.Nsec = obj.Nsec;
        end

        function delete(~)
        %delete Protected delete call to prevent explicit invocations
        %   This will avoid problems due to deleted handle
        %   objects in copying or saving to a MAT file.
        end
    end

    % Methods inherited from CustomDisplay
    methods (Access = protected)
        function header = getHeader(obj)
        %getHeader Returns a custom header for ROS time structure

            if ~isscalar(obj)
                % For an object array, extract its size
                headerStr = matlab.mixin.CustomDisplay.getClassNameForHeader(obj);
                headerStr = ['  ' num2str(size(obj,1)) 'x' num2str(size(obj,2)) ...
                             ' ROS ', headerStr,' array with properties:'];
                header = sprintf('%s\n',headerStr);
            else
                % This is the custom display for scalar objects
                headerStr = matlab.mixin.CustomDisplay.getClassNameForHeader(obj);
                headerStr = ['  ROS ', headerStr,' with properties:'];
                header = sprintf('%s\n',headerStr);
            end
        end

        % Convert a TimeEntity to string representation
        % TimeEntity -> "<TimeStruct.sec>:<TimeStruct.nsec>"
        function str = t2string(obj)
            str = string(obj.Sec)+":"+string(obj.Nsec);
        end
    end
end
