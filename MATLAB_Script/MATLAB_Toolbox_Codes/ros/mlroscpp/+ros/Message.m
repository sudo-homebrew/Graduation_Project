classdef (Abstract) Message < matlab.mixin.Copyable & ...
        matlab.mixin.CustomDisplay & ...
        matlab.mixin.internal.CompactDisplay & ...
        matlab.mixin.Heterogeneous
    %Message Base class for all ROS messages in MATLAB
    %   All message implementations use ros.Message as a superclass
    %   and will inherit a set of properties and methods.
    %
    %   Message properties:
    %      MessageType   - The ROS message type of this object
    %
    %   Message methods:
    %      definition    - Retrieves the message definition
    %      showdetails   - Displays all message contents
    %
    %   See also ROSMESSAGE.

    %   The superclasses of the Message class are used as follows:
    %   Copyable       - Adds the ability to deep-copy message objects.
    %   CustomDisplay  - Allows customization of object display.
    %   CompactDisplay - Removes full class namespaces from object display.
    %   Heterogeneous  - Allow heterogeneous message arrays and support
    %                    good default initialization of object array
    %                    elements.

    %   Copyright 2014-2020 The MathWorks, Inc.

    properties(Abstract, Constant)
        %MessageType - The ROS message type of this object
        MessageType
    end

    properties(Abstract, Hidden, Constant)
        %MD5Checksum - Recursive message definition checksum
        MD5Checksum

        %PropertyList - List of non-constant message properties
        PropertyList

        %ROSPropertyList - List of non-constant ROS message properties
        ROSPropertyList

        %PropertyMessageTypes - Types of contained nested messages
        PropertyMessageTypes
    end

    methods
        function obj = Message(msgStruct)
        % Construct the message, with optional ROS message structure

        % Support default constructor
            if nargin == 0
                msgStruct = ros.internal.getEmptyMessage(obj.MessageType, 'ros');
            end
            objType = class(obj);

            % Construct appropriate empty array
            if isempty(msgStruct)
                obj = obj.empty(0,1);
                return
            end

            % Check for correct input class
            % Only check first element of struct array, since all fields at
            % first level will be consistent, and nested messages will be
            % checked subsequently on creation
            if ~ros.Message.compareTypeStruct(objType, msgStruct(1))
                error(message('ros:mlroscpp:message:NoTypeMatch', ...
                              obj.MessageType));
            end

            % Make scalar construction fast
            fromStruct(obj, msgStruct(1))

            % Continue to construct array of objects if necessary
            for k = 2:numel(msgStruct)
                obj(k,1) = feval(objType, msgStruct(k)); %#ok<AGROW>
            end
        end

        function varargout = definition(obj)
        %DEFINITION Retrieves the message type definition
        %   DEFINITION(OBJ) prints the ROS definition of the message
        %   type associated with message object OBJ to the command window.
        %
        %   DEF = DEFINITION(OBJ) returns the ROS definition as a
        %   string.
        %
        %   This function will retrieve the ROS definition of the
        %   message type of this object. The definition is
        %   structured according to the ROS message description
        %   language.

            typeDefinition = '';
            if ~isempty(obj.MessageType)
                typeDefinition = rosmsg('show', obj.MessageType);
            end

            % Return the definition string, if requested by user
            if nargout > 0
                varargout{1} = typeDefinition;
                return
            end

            % Otherwise, print to console
            disp(typeDefinition)
        end


        function varargout = showdetails(obj)
        %SHOWDETAILS Displays all message contents
        %   SHOWDETAILS(OBJ) recursively prints all data contents of
        %   message object OBJ to the command window.
        %
        %   DET = SHOWDETAILS(OBJ) returns the detailed data view as a
        %   string.
        %
        %   This function allows you to display the full data contents of
        %   a ROS message. This can be very useful in interactive
        %   exploration and debugging, as it allows you to get a quick
        %   summary of the message data.

            detailedDisplay = ros.msg.internal.MessageDisplay.printData(obj);

            % Return the detailed data string, if requested by user
            if nargout > 0
                varargout{1} = detailedDisplay;
                return
            end

            % Otherwise, print to console
            disp(detailedDisplay)
        end
    end

    methods (Hidden)
        function s = toROSStruct(obj)
        %toROSStruct Convert the message object to internal ROS struct
        %   STRUCTMSG = toROSStruct(OBJ) returns the message data in
        %   OBJ as a structure STRUCTMSG, using the internal ROS
        %   fieldnames. Converting the message to a struct requires
        %   reading all the data contained within a message, which
        %   might take some time if the message structure is complex.
        %   OBJ can be a scalar message object or an object array where
        %   all message objects have the same message type. An error
        %   will be displayed if OBJ is a heterogeneous array
        %   (containing message objects with different message types),
        %   since this data structure cannot be represented as a
        %   struct.

        % Fast execution for scalar objects
            if isscalar(obj)
                s = toScalarROSStruct(obj);
                return
            end

            % Special handling for empty objects / arrays
            if isempty(obj)
                s = struct.empty(0,1);
                return
            end

            % Special handling for object array inputs
            % Pre-allocate structure array for performance.
            s = repmat(toScalarROSStruct(obj(1)), size(obj));
            for k = 2:numel(obj)
                s(k) = toScalarROSStruct(obj(k));
            end
        end

        function s = toStruct(obj)
        %toStruct Convert the message object to a struct
        %   STRUCTMSG = toStruct(OBJ) returns the message data in OBJ as a
        %   structure STRUCTMSG. This can be useful if a non object-oriented
        %   representation of the message is desired. Converting the message to a
        %   struct requires reading all the data contained within a
        %   message, which might take some time if the message structure
        %   is complex.
        %   OBJ can be a scalar message object or an object array where
        %   all message objects have the same message type. An error
        %   will be displayed if OBJ is a heterogeneous array
        %   (containing message objects with different message types),
        %   since this data structure cannot be represented as a struct.

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

        function fromStruct(obj, sObj)
        %fromStruct Reload the object from a struct representation
        %   fromStruct(OBJ, STRUCTOBJ) assigns all message properties
        %   in OBJ based on the data contained in the struct STRUCTOBJ.
        %   STRUCTOBJ needs to have the same nested structure as the
        %   message object.
        %   This function use of the same mechanism that is used when
        %   loading a message from a MAT file.
        %
        %   STRUCTOBJ can be a scalar struct or a struct array of
        %   messages. If STRUCTOBJ is a struct array, OBJ needs to be
        %   an object array with the same number of elements.
        %   An error will be displayed if OBJ is a heterogeneous object array
        %   (containing message objects with different message types),
        %   since this data structure cannot be represented as a struct.

        % Fast execution for scalar objects
            if isscalar(obj)
                obj.reload(sObj)
                return
            end

            % Special handling for empty objects / arrays
            if isempty(obj)
                return
            end

            % Special handling for structure array inputs
            % Ensure that structure and object array have the same size and
            % then assign data element by element.
            if numel(obj) ~= numel(sObj)
                error(message('ros:mlroscpp:message:FromStructArrayInputSize', ...
                              num2str(numel(obj)), num2str(numel(sObj))));
            end

            for k = 1:numel(sObj)
                obj(k).reload(sObj(k));
            end
        end

        function fromROSStruct(obj, sObj)
        %fromROSStruct Reload the object from an internal ROS struct
        %   fromROSStruct(OBJ, STRUCTOBJ) assigns all message
        %   properties in OBJ based on the data contained in the struct
        %   STRUCTOBJ. STRUCTOBJ needs to have the same nested
        %   structure a the message object, excepting the fieldnames.
        %
        %   STRUCTOBJ can be a scalar struct or a struct array of
        %   messages. If STRUCTOBJ is a struct array, OBJ needs to be
        %   an object array with the same number of elements.
        %   An error will be displayed if OBJ is a heterogeneous object
        %   array (containing message objects with different message
        %   types), since this data structure cannot be represented as
        %   a struct.

        % Fast execution for scalar objects
            if isscalar(obj)
                obj.fromScalarROSStruct(sObj)
                return
            end

            % Special handling for empty objects / arrays
            if isempty(obj)
                return
            end

            % Special handling for structure array inputs
            % Ensure that structure and object array have the same size and
            % then assign data element by element.
            if numel(obj) ~= numel(sObj)
                error(message('ros:mlroscpp:message:FromStructArrayInputSize', ...
                              num2str(numel(obj)), num2str(numel(sObj))));
            end

            for k = 1:numel(sObj)
                obj(k).fromScalarROSStruct(sObj(k));
            end
        end
    end

    methods (Static, Sealed, Access = protected)
        function defaultObj = getDefaultScalarElement
        %getDefaultScalarElement Get default object for heterogeneous hierarchy
        %   This function is overloaded from matlab.mixin.Heterogeneous
        %   and allows proper initialization of missing array elements.
            defaultObj = ros.msg.internal.MessagePlaceholder;
        end
    end

    methods (Static, Access = {?ros.Message, ?matlab.unittest.TestCase})
        function valid = compareTypeStruct(msgClass, strObj)
        %compareTypeStruct Check scalar internal ROS struct has
        %expected fields based on provided packaged message class

        % Use meta class to avoid instantiating message object
            metaMsgClass = meta.class.fromName(msgClass);
            propList = findobj(metaMsgClass.PropertyList, 'Name', 'PropertyList').DefaultValue;
            propMsgTypes = findobj(metaMsgClass.PropertyList, 'Name', 'PropertyMessageTypes').DefaultValue;

            % Check each message field
            k = 1;
            valid = true;
            while valid && k <= numel(propList)
                fs = propList{k};                % Struct field
                ft = propMsgTypes{k};               % Field message type
                                                    % Recurse on nested messages
                                                    % Check against handle to account for Time/Duration
                valid = isfield(strObj, fs);
                if valid && ~isempty(ft) && ...
                        ~isempty(strObj) && ~isempty(strObj.(fs))
                    valid = ros.Message.compareTypeStruct(ft, strObj.(fs)(1));
                end
                k = k+1;
            end
        end

        function fixedArray = initializeFixedArray(array, ~)
        %initializeFixedArray Initialize fixed-size arrays correctly
        %   In the past, this method was used to work around a rosjava
        %   bug that prevented the correct initialization of fixed-size
        %   arrays. The method is maintained as a pass-through to allow
        %   message classes generated in 15a and 15b to be usable in
        %   16a+.
            fixedArray = array;
        end
    end

    methods (Access = protected)
        function delete(~)
        %delete Protected delete call to prevent explicit invocations
        %   This will avoid problems due to deleted message handle
        %   objects in copying or saving to a MAT file.
        end

        function cpObj = copyElement(obj)
        %copyElement Implements deep copy behavior for message

        % Call default copy method for shallow copy
            cpObj = copyElement@matlab.mixin.Copyable(obj);

            % Iterate over fields
            for k = 1:numel(obj.PropertyList)
                % Recurse on nested messages
                % Check against handle to account for Time/Duration
                if isa(obj.(obj.PropertyList{k}), 'handle')
                    cpObj.(obj.PropertyList{k}) = ...
                        copy(obj.(obj.PropertyList{k}));
                else
                    cpObj.(obj.PropertyList{k}) = ...
                        obj.(obj.PropertyList{k});
                end
            end
        end
    end

    methods (Access = ?ros.Message)
        function strObj = saveobj(obj)
        %saveobj Implements saving of message to MAT file

        % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end

            strObj = struct;
            for k = 1:numel(obj.PropertyList)
                fc = obj.PropertyList{k};           % Class field
                                                    % Recurse on nested messages
                                                    % Check against handle to account for Time/Duration
                if isa(obj.(fc), 'handle')
                    strObj.(fc) = toStruct(obj.(fc));
                else
                    strObj.(fc) = obj.(fc);
                end
            end
        end

        function reload(obj, strObj)
        %reload Called by loadobj to assign properties

        % Iterate over fields
            for k = 1:numel(obj.PropertyList)
                fc = obj.PropertyList{k};           % Class field
                ft = obj.PropertyMessageTypes{k};   % Field message type
                                                    % Recurse on nested messages
                if ~isempty(ft)
                    % Handle arrays of nested messages
                    if isscalar(strObj.(fc))
                        obj.(fc) = feval([ft '.loadobj'], ...
                                         strObj.(fc));
                    elseif isempty(strObj.(fc))
                        obj.(fc) = feval([ft '.empty'], 0, 1);
                    else
                        fieldCell = ...
                            arrayfun(@(x) feval([ft '.loadobj'], x), ...
                                     strObj.(fc), 'UniformOutput', false);
                        obj.(fc) = vertcat(fieldCell{:});
                    end
                else
                    obj.(fc) = strObj.(fc);
                end
            end
        end

        function strObj = toScalarROSStruct(obj)
        %toScalarROSStruct Scalar internal ROS struct from message

        % TODO: This is nearly the same as saveobj - needs special handling
        % for Time/Duration - try to merge

        % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end

            strObj = struct;
            for k = 1:numel(obj.PropertyList)
                fc = obj.PropertyList{k};           % Class field
                                                    % Recurse on nested messages
                                                    % Check against handle to account for Time/Duration
                if isa(obj.(fc), 'handle')
                    strObj.(fc) = toROSStruct(obj.(fc));
                else
                    strObj.(fc) = obj.(fc);
                end
            end
        end

        function fromScalarROSStruct(obj, strObj)
        %fromScalarROSStruct Assign properties from scalar internal ROS struct

            for k = 1:numel(obj.ROSPropertyList)
                fc = obj.PropertyList{k};           % Class field
                fs = obj.ROSPropertyList{k};        % Struct field
                ft = obj.PropertyMessageTypes{k};   % Field message type
                                                    % Recurse on nested messages
                if ~isempty(ft)
                    % Handle empty, scalar, and array nested messages
                    if isempty(strObj.(fs))
                        obj.(fc) = feval([ft '.empty'], 0, 1);
                    else
                        obj.(fc) = feval(ft, strObj.(fs));
                    end
                else
                    obj.(fc) = strObj.(fs);
                end
            end
        end
    end

    % Methods inherited from CustomDisplay
    % These have to be sealed to allow proper display in a heterogeneous
    % array.
    % See https://www.mathworks.com/help/matlab/matlab_oop/custom-display-for-heterogeneous-arrays.html
    methods (Sealed, Access = protected)
        function displayNonScalarObject(obj)
        %displayNonScalarObject Object display for non-scalar objects
        %   Overloaded from matlab.mixin.CustomDisplay
            displayNonScalarObject@matlab.mixin.CustomDisplay(obj);
        end

        function displayEmptyObject(obj)
        %displayEmptyObject Object display for empty objects
        %   Overloaded from matlab.mixin.CustomDisplay
            displayEmptyObject@matlab.mixin.CustomDisplay(obj);
        end

        function propgrp = getPropertyGroups(obj)
        %getPropertyGroups Property group display
        %   Overloaded from matlab.mixin.CustomDisplay
            propgrp = getPropertyGroups@matlab.mixin.CustomDisplay(obj);
        end

        function header = getHeader(obj)
        %getHeader Returns a custom header for ROS messages
        %   To disambiguate ROS message class names from MATLAB
        %   classes, we display specific text in the header that
        %   identifies this object as a ROS message.

            if ~isscalar(obj)
                % For an object array, extract its size
                headerStr = matlab.mixin.CustomDisplay.getClassNameForHeader(obj);
                headerStr = ['  ' matlab.mixin.CustomDisplay.convertDimensionsToString(obj) ...
                             ' ROS ', headerStr,' message array '];

                % If obj is a heterogeneous array of ros.Message
                % type, then no properties are visible, so we need to
                % account for this case.
                if numel(properties(obj)) > 0
                        headerStr = [headerStr 'with properties:'];
                else
                    headerStr = [headerStr 'with no properties.'];
                end

                header = sprintf('%s\n',headerStr);
            else
                % This is the custom display for scalar objects
                headerStr = matlab.mixin.CustomDisplay.getClassNameForHeader(obj);
                headerStr = ['  ROS ', headerStr,' message with properties:'];
                header = sprintf('%s\n',headerStr);
            end
        end

        function footer = getFooter(obj)
        %getFooter Returns a custom footer for ROS messages
        %   The footer points users to the showdetails method, which
        %   can display the full data contents of the message in
        %   textual form.

        %   In our original design, we envisioned that this link would
        %   call showdetails on the object, but this is not easily
        %   possible, since we cannot hard-code the variable name. This
        %   sort of dynamic invocation is not supported by the link.

            if ~isscalar(obj)
                footer = getFooter@matlab.mixin.CustomDisplay(obj);
                return;
            end

            % Dealing with a scalar object
            % Using the feature check, we will only display the HTML link
            % if hotlinks are enabled.
            if feature('hotlinks')
                showDetailsString = ['<a href="matlab:helpPopup ros.Message/showdetails" ' ...
                                    'style="font-weight:bold">showdetails</a>'];
            else
                showDetailsString = 'showdetails';
            end

            footerStr = ['  Use ' showDetailsString ' to show the contents of the message'];
            footer = sprintf('%s\n',footerStr);
        end
    end

    methods (Hidden)
        % The methods in this block are overloading methods from the handle
        % superclass and making them hidden, so they do not show up in the
        % tab completion for message objects.

        function varargout = addlistener(obj, varargin)
        %addlistener Overload of handle.addlistener
        %   This method passes through all arguments.

            [varargout{1:nargout}] = addlistener@handle(obj, varargin{:});
        end

        function notify(obj, varargin)
        %notify Overload of handle.notify
        %   This method passes through all arguments.

            notify@handle(obj, varargin{:});
        end

        function varargout = findobj(obj, varargin)
        %findobj Overload of handle.findobj
        %   This method passes through all arguments.

            [varargout{1:nargout}] = findobj@handle(obj, varargin{:});
        end

        function varargout = findprop(obj, varargin)
        %findprop Overload of handle.findprop
        %   This method passes through all arguments.

            [varargout{1:nargout}] = findprop@handle(obj, varargin{:});
        end
    end
end
