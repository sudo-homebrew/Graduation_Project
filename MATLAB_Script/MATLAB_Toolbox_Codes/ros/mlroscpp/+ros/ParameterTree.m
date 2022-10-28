classdef ParameterTree < ros.internal.mixin.ROSInternalAccess & ...
        robotics.core.internal.mixin.Unsaveable & handle
    %ParameterTree Create an object to access the ROS parameter server
    %   A ROS parameter tree communicates with the ROS parameter server.
    %   The ROS parameter server can store strings, integers, doubles,
    %   booleans and cell arrays. The parameters are accessible globally
    %   over the ROS network. The parameters can be used to store static
    %   data such as configuration parameters.
    %
    %   PTREE = ros.ParameterTree(NODE) returns a parameter tree object for
    %   communicating with the ROS parameter server. The parameter tree
    %   attaches to the ROS node NODE. If NODE is [], the parameter
    %   tree tries to attach to the global ROS node.
    %
    %   Once the parameter tree object is created, the connection to the
    %   parameter server will be persistent until the object is deleted or
    %   the ROS master becomes unavailable.
    %
    %   The following ROS data types are supported as values of parameters.
    %   For each ROS data type, the corresponding MATLAB data type is also
    %   listed:
    %   - 32-bit integers: MATLAB data type 'int32'
    %   - booleans: MATLAB data type 'logical'
    %   - strings: MATLAB data type 'char'
    %   - doubles: MATLAB data type 'double'
    %   - lists: MATLAB data type 'cell'
    %   - dictionaries: MATLAB data type 'struct'
    %   Base64-encoded binary data, and iso8601 dates are not currently supported.
    %
    %
    %   ParameterTree properties:
    %       AvailableParameters - (Read-Only) List of parameter names on server
    %
    %   ParameterTree methods:
    %       get     - Get value of a parameter
    %       has     - Check if a parameter exists
    %       search  - Search for parameter names
    %       set     - Set value or add a new parameter
    %       del     - Delete a parameter
    %
    %
    %   Example:
    %       % Create the ROS master and a node
    %       master = ros.Core;
    %       node = ros.Node('/testParameters');
    %
    %       % Create the parameter tree object and connect to global node
    %       ptree = ros.ParameterTree(node);
    %
    %       % Set a parameter with a double value
    %       set(ptree, 'DoubleParam', 1.0);
    %
    %       % Set a parameter with a string value
    %       set(ptree, 'CharParam', 'test');
    %
    %       % Set a parameter with a cell array value
    %       set(ptree, 'CellParam', {{'test'},{1,2}});
    %
    %       % Get the parameter value
    %       data = get(ptree, 'CellParam');
    %
    %       % Search for parameter names
    %       search(ptree, 'char');
    %
    %       % Get the full parameter tree as a struct
    %       treeStruct = get(ptree, '/')
    %
    %   See also ROSPARAM.

    %   Copyright 2020-2021 The MathWorks, Inc.

    properties (Dependent, SetAccess = private)
        %AvailableParameters - List of parameter names on server
        %   This property is an Nx1 cell array of strings containing the
        %   names of all parameters that are currently registered on the
        %   parameter server.
        AvailableParameters
    end

    properties (Transient, Access = ?ros.internal.mixin.ROSInternalAccess)
        %InternalNode - Internal representation of the node object
        %   Node required to get parameter information
        InternalNode
        ServerNodeHandle
    end

    properties (Constant, Access = private)
        GLOBALNS = '/'
    end

    methods
        function obj = ParameterTree(node)
        %ParameterTree Constructor for the ParameterTree
        %   This function takes a node object as an input and attaches
        %   the parameter tree to the node. If the input is empty,
        %   the global node is used.

        % If the input is empty, get the global node handle
            if (nargin < 1) || isempty(node)
                node = ros.internal.Global.getNodeHandle(false);
            end

            % Validate that node is a ros.Node object
            validateattributes(node, {'ros.Node'}, {'scalar','nonempty'}, ...
                               'ParameterTree', 'node', 1);

            % Gateway to ROS server
            obj.InternalNode = node.InternalNode;
            obj.ServerNodeHandle = node.ServerNodeHandle;
            node.ListofNodeDependentHandles{end+1} = matlab.internal.WeakHandle(obj);
        end

        function delete(obj)
        %DELETE Shut down parameter tree
        %   DELETE(OBJ) shuts down the ROS parameter tree object.
            obj.InternalNode = [];
            obj.ServerNodeHandle = [];
        end

        function paramNames = get.AvailableParameters(obj)
        %get.AvailableParameters Get AvailableParameters property

        % Get the list of parameter names from the parameter tree
        % getParamNames does not throw exceptions
            paramNames = sort(getParamNames(obj.InternalNode,...
                                            obj.ServerNodeHandle));
        end

        function set(obj,paramName,paramValue)
        %SET Set a value or add a new parameter
        %   SET(OBJ, PNAME, PVALUE) sets a value PVALUE for a parameter
        %   with name PNAME in the parameter tree object OBJ.
        %   PVALUE can be of any supported data type. This function
        %   adds a new parameter in the parameter tree if the
        %   parameter name PNAME does not exist.
        %
        %   SET(OBJ, NAMESPACE, DICTSTRUCT) allows the setting of
        %   multiple parameters in a namespace. The parameter namespace
        %   is defined by the structure DICTSTRUCT and all elements
        %   are created under NAMESPACE.
        %
        %   Example:
        %       % Create a parameter tree and set a parameter
        %       % This assumes that you already created a node object
        %       ptree = ros.ParameterTree(node);
        %       SET(ptree, 'myParam', 1);
        %       SET(ptree, 'myParam', {1,2,3});
        %
        %       % Set parameters in the /ns_param namespace
        %       SET(ptree, '/ns_param', struct('number', int32(5), 'string', 'some_string'));
        %
        %       % Display all available parameters
        %       ptree.AvailableParameters

        % Validate inputs
            paramName = robotics.internal.validation.validateString(paramName, false, 'set', 'name');
            validateattributes(paramValue,{'int32','double','char','string',...
                                'logical','cell','struct'},{},'set','value');

            % The following line handles validation of the parameter name, it
            % adds namespace if an environment variable is defined and adds
            % the node name for private parameters
            paramName = ros.internal.Namespace.canonicalizeName(paramName);
            formattedParamValue = [];
            if isnumeric(paramValue) || islogical(paramValue)
                % Set scalar numeric or logical value
                validateattributes(paramValue,{'int32','double',...
                                    'logical'},{'nonempty','scalar'},'set', 'value')
                formattedParamValue = paramValue;
            elseif ischar(paramValue) || isstring(paramValue)
                % Set a string
                paramValue = robotics.internal.validation.validateString(paramValue, true, 'set', 'value');
                % Parameter server uses XmlRpc which does not support
                % illegal characters
                % Legal characters are tab, carriage return, line feed, and the legal characters of Unicode and ISO/IEC 10646.
                %
                % Char ::= 0x9 | 0xA | 0xD | [0x20-0xD7FF] | [0xE000-0xFFFD] | [0x10000-0x10FFFF]
                % [0x10000-0x10FFFF] are out of range of MATLAB char which
                % is uint16
                for k = 1:numel(paramValue)
                    c = paramValue(k);
                    % Must be in range [0x9, 0xA, 0xD] | [0x20 - 0xD7FF] [0xE000 - 0xFFFD]
                    if ~( ((c == 0x9) || (c == 0xA) || (c == 0xD)) ...
                            || ((c >= 0x20) && (c <= 0xD7FF)) ...
                            || ((c >= 0xE000) && (c <= 0xFFFD)) )
                        error(message('ros:mlros:param:SetStringParamError',paramName,k));
                    end
                end
                formattedParamValue = paramValue;
            elseif iscell(paramValue)
                % Set cell array as a list
                % Recursive validation of each element
                obj.validateCellInput(paramValue);
                formattedParamValue = paramValue;
            elseif isstruct(paramValue)
                % Set struct as a dictionary
                obj.validateStructInput(paramValue);
                formattedParamValue = paramValue;
            end

            % Set the parameter value
            try
                setParam(obj.InternalNode,obj.ServerNodeHandle,...
                                       paramName,formattedParamValue);
            catch ex
                % Exceptions thrown by setParam
                % FAIL_SERVER_PARAM_INVALID_RESOURCE_NAME
                % FAIL_SERVER_PARAM_INVALID_DATATYPE
                % FAIL_SERVER_PARAM_EMPTY_PARAMETER_VALUE
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:param:SetParameterError',paramName));
                throw(newEx.addCustomCause(ex));
            end
        end

        function [data,status] = get(obj,paramName)
        %GET Get the value of a parameter
        %   DICTSTRUCT = GET(OBJ) returns a dictionary of
        %   parameter values under the root namespace /.
        %   DICTSTRUCT is returned as a MATLAB struct.
        %
        %   PVALUE = GET(OBJ, PNAME) retrieves the value of the parameter
        %   with name PNAME from the parameter tree OBJ.
        %   To retrieve the parameter value, the input parameter name
        %   should exactly match an existing parameter name.
        %   This function does not allow partial name input.
        %
        %   DICTSTRUCT = GET(OBJ, NAMESPACE) returns a dictionary of
        %   parameter values under the ROS namespace NAMESPACE.
        %   The parameter server represents ROS namespaces as dictionaries
        %   and the retrieved dictionary, DICTSTRUCT, is returned as a
        %   MATLAB struct.
        %
        %   The function displays an error if a parameter or
        %   parameter namespace with this name does not exist.
        %
        %   [PVALUE,STATUS] = GET(OBJ, PNAME) returns a STATUS indicating
        %   whether PVALUE has been received successfully. If the
        %   STATUS is false, no error will be thrown. Instead,
        %   an empty default value will be returned.
        %
        %   Example:
        %       % Create a parameter tree and get a parameter
        %       % This assumes that you already created a node object
        %       ptree = ros.ParameterTree(node);
        %       set(ptree, 'myParam', 1);
        %       value = GET(ptree, 'myParam');
        %
        %       % Get the full parameter tree as a struct
        %       treeStruct = get(ptree, '/')

            if nargin < 2
                paramName = '';
            end

            % Validate the input argument
            paramName = robotics.internal.validation.validateString(paramName, true, 'get', 'name');

            % The following line handles validation of the parameter name, it
            % adds namespace if an environment variable is defined and adds
            % the node name for private parameters
            paramName = ros.internal.Namespace.canonicalizeName(paramName);

            % Initialize status as false
            status = false; %#ok<NASGU> 
            
            % Get the data from the parameter server
            try
                % Fails if parameter cannot be retrieved from the server
                data = getParam(obj.InternalNode,obj.ServerNodeHandle,paramName);
            catch ex
                % Exceptions thrown by getParam
                % FAIL_SERVER_PARAM_INVALID_RESOURCE_NAME
                % FAIL_SERVER_PARAM_UNKNOWN_PARAM
                if nargout < 2
                    if isequal(ex.identifier,'ros:internal:transport:UnknownParameter')
                        newEx = ros.internal.ROSException( ...
                            message('ros:mlros:param:ParameterDoesNotExist',paramName));
                    else
                        newEx = ros.internal.ROSException(...
                            message('ros:mlros:param:GetParameterError',paramName));
                    end
                    throw(newEx.addCustomCause(ex));
                end
            end
            
            status = true;
        end

        function nameExists = has(obj, paramName)
        %HAS Check if the parameter name exists
        %   EXISTS = HAS(OBJ, PNAME) checks if the parameter with name
        %   PNAME exists in the parameter tree OBJ. EXISTS is
        %   a logical "true" if the parameter exists, or "false"
        %   otherwise. This function is case sensitive.
        %
        %   Example:
        %       % Check if the parameter exists
        %       % This assumes that you already created a node object
        %       ptree = ros.ParameterTree(node);
        %       set(ptree, 'myParam', 1);
        %       HAS(ptree, 'myParam')
        %       HAS(ptree, 'newParam')

        % Validate input arguments
            paramName = robotics.internal.validation.validateString(paramName, false, 'has', 'name');

            % The following line handles validation of the parameter name, it
            % adds namespace if an environment variable is defined and adds
            % the node name for private parameters
            paramName = ros.internal.Namespace.canonicalizeName(paramName);

            % Get the list of parameters in the parameter tree
            % Not using AvailableParameters to avoid recursion errors
            try
                nameExists = hasParam(obj.InternalNode,obj.ServerNodeHandle, ...
                                      paramName);
            catch ex
                % Exceptions thrown by getParam
                % FAIL_SERVER_PARAM_INVALID_RESOURCE_NAME
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:param:ParameterListError'));
                throw(newEx.addCustomCause(ex));
            end
        end

        function [pnames, pvalues] = search(obj, paramName)
        %SEARCH Search for parameter names
        %   [PNAMES, PVALUES] = SEARCH(OBJ, PNAMESTR) searches within
        %   the parameter tree OBJ for parameter names that contain the
        %   string PNAMESTR. It returns the matching parameter
        %   names in PNAMES as a cell array of strings. Optionally, you
        %   can retrieve the values of these parameters in PVALUES.
        %   PVALUES is a cell array with the same length as
        %   PNAMES.
        %
        %   Example:
        %       % Check if the parameter exists
        %       % This assumes that you already created a node object
        %       ptree = ros.ParameterTree(node);
        %       set(ptree, 'myParam', 1);
        %       set(ptree, 'newParam', 0);
        %       SEARCH(ptree, 'new')

        % Validate input arguments, allows empty string
            paramName = robotics.internal.validation.validateString(paramName, true, 'search', 'name');

            % Get the parameter list
            allnames = obj.AvailableParameters';

            % Return empty if parameter list is empty
            if isempty(allnames)
                pnames = {};
                pvalues = {};
                return;
            end

            % If input is empty, return entire tree
            if isempty(paramName)
                pnames = allnames;
            else
                % Find matching string and return the matching parameter list
                rowMatches = strfind(lower(allnames),lower(paramName));
                matchIdx = cellfun(@(x)~isempty(x),rowMatches);
                pnames = allnames(matchIdx);
            end

            % Retrieve values if requested by the user
            if nargout == 2
                pvalues = cell(numel(pnames),1);
                for i=1:numel(pnames)
                    pvalues{i,1} = obj.get(pnames{i});
                end
            end
        end

        function del(obj, paramName)
        %DEL Delete a parameter
        %   DEL(OBJ, PNAME) deletes a parameter with name PNAME from the parameter
        %   tree OBJ. The function displays an error if a parameter
        %   with this name does not exist.
        %
        %   DEL(OBJ, NAMESPACE) deletes a parameter namespace under
        %   NAMESPACE in the parameter tree OBJ. The function
        %   displays an error if a namespace with this name does not exist.
        %
        %   Example:
        %       % Check if the parameter exists
        %       % This assumes that you already created a node object
        %       ptree = ros.ParameterTree(node);
        %       set(ptree, 'myParam', 1);
        %
        %       % Delete the parameter with name "myParam"
        %       DEL(ptree, 'myParam');
        %       has(ptree, 'myParam')

        % Validate input arguments
        % Explicitly disallow empty inputs to avoid unintended deletion
        % of the whole parameter tree.
            paramName = robotics.internal.validation.validateString(paramName, false, 'del', 'name');

            % The following line handles validation of the parameter name, it
            % adds namespace if an environment variable is defined and adds
            % the node name for private parameters
            paramName = ros.internal.Namespace.canonicalizeName(paramName);
            if ~has(obj,paramName)
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:param:ParameterDoesNotExist',paramName));
                throw(newEx);
            end
            try
                ret = deleteParam(obj.InternalNode,obj.ServerNodeHandle, ...
                                  paramName);
                if ret ~= true
                    newEx = ros.internal.ROSException( ...
                        message('ros:mlros:param:DeleteParameterError',paramName));
                    throw(newEx);
                end
            catch ex
                % Exceptions thrown by deleteParam
                % FAIL_SERVER_PARAM_INVALID_RESOURCE_NAME
                newEx = ros.internal.ROSException( ...
                    message('ros:mlros:param:DeleteParameterError',paramName));
                throw(newEx.addCustomCause(ex));
            end
        end
    end

    methods (Access = private)
        function validateCellInput(obj,cellArray)
        %validateCellInput Validate elements of cell
        %   This function is used for recursively validating every
        %   element of the input cell array. The elements of cell array
        %   can only take 'char', 'logical', 'double', 'int32' or
        %   'uint32'.

            if ~isempty(cellArray)
                % Validate if cell array is a vector (row or column)
                validateattributes(cellArray, {'cell'}, {'vector'}, 'set', 'value');

                % Check data type for every element
                cellfun(@(x)obj.checkDataType(x), cellArray,'UniformOutput',false);
            end
        end

        function validateStructInput(obj, str)
        %validateStructInput Validate fields of structure
        %   This function is used for recursively validating every
        %   field of the input structure STR. The field values of the
        %   structure can only take on data types validated by
        %   checkDataType.

            if ~isempty(str)
                % Validate if struct is a scalar
                validateattributes(str, {'struct'}, {'scalar'}, 'set', 'value');

                % Check data type for every element
                structfun(@(x)obj.checkDataType(x), str, 'UniformOutput', false);
            end
        end

        function checkDataType(obj,inputData)
        %checkDataType Check data type
        %   This function is used for recursively checking the input
        %   data type for each element of the cell array

        % If cell array, then call this function recursively
            if iscell(inputData)
                obj.validateCellInput(inputData);
                return;
            end

            if isstruct(inputData)
                obj.validateStructInput(inputData);
                return;
            end

            if isnumeric(inputData)
                % Check if numeric data is scalar
                try
                    validateattributes(inputData,{'numeric'},{'scalar'},2)
                catch
                    error(message('ros:mlros:param:InvalidDataSize'));
                end
            end

            % Check if data belongs to supported types
            try
                validateattributes(inputData,{'int32','double',...
                                    'logical','char'},{},2)
            catch
                error(message('ros:mlros:param:CellArrayDataError'));
            end
        end
    end

    %----------------------------------------------------------------------
    % MATLAB Code-generation
    %----------------------------------------------------------------------
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'ros.internal.codegen.ParameterTree';
        end
    end
end
