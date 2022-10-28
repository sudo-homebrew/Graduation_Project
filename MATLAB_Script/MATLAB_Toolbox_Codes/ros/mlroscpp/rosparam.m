function output = rosparam(operation, varargin)
%ROSPARAM Access values on the parameter server
%
%   PTREE = ROSPARAM creates a parameter tree object and returns it in
%   PTREE. Once the parameter tree object is created, the connection to the
%   parameter server is persistent until the object is deleted or
%   the ROS master becomes unavailable.
%
%   LST = ROSPARAM('list') returns a list of parameter names from the ROS
%   master as a cell array of character vectors, LST. If the output argument
%   LST is not defined, the list is printed to the command line.
%   Simplified form: ROSPARAM list
%
%   LST = ROSPARAM('list', NAMESPACE) returns a list of parameter names
%   under the ROS namespace NAMESPACE. Use '/' to return all registered
%   parameter names.
%   Simplified form: ROSPARAM list NAMESPACE
%
%   VAL = ROSPARAM('get', PNAME) retrieves the value of the parameter
%   with name PNAME.
%   Simplified form: ROSPARAM get PNAME
%
%   NSSTRUCT = ROSPARAM('get', NAMESPACE) returns a struct of
%   parameter values under the ROS namespace NAMESPACE.
%   Simplified form: ROSPARAM get NAMESPACE
%
%   ROSPARAM('set', PNAME, VAL) sets a value VAL for a parameter
%   with name PNAME. VAL can be of any supported data type for the parameter.
%   This function adds a new parameter in the parameter tree if the
%   parameter name PNAME does not exist.
%   Simplified form: ROSPARAM set PNAME VAL
%
%   ROSPARAM('delete', PNAME) deletes a parameter with name PNAME.
%   The function displays an error if a parameter
%   with this name does not exist.
%   Simplified form: ROSPARAM delete PNAME
%
%   ROSPARAM('delete', NAMESPACE) deletes all parameter values under the
%   ROS namespace NAMESPACE.
%   Simplified form: ROSPARAM delete NAMESPACE
%
%
%   Example:
%       % Initialize the ROS master and global node
%       rosinit;
%
%       % Get the list of all parameters
%       rosparam list
%
%       % Get a list of parameters under the /gazebo namespace
%       rosparam list /gazebo
%
%       % Set a string parameter
%       rosparam set /string_param param_value
%
%       % Get the value of a string parameter
%       rosparam get /string_param
%
%       % Delete a parameter
%       rosparam delete /string_param
%
%       % Set a double parameter
%       rosparam set /double_param 1.2
%
%       % Set a list parameter (need to use functional form)
%       rosparam('set', '/list_param', {int32(5), 124.1, -20, 'some_string'});
%
%       % Get the value of the list parameter
%       rosparam get /list_param
%
%       % Get the value of all parameters under the root as a structure
%       rosparam get /
%
%   See also ros.ParameterTree.

%   Copyright 2015-2021 The MathWorks, Inc.
%#codegen

coder.internal.narginchk(0,5,nargin);
supportedOperations = {'list', 'get', 'set', 'delete'};

if isempty(coder.target)
    try
        if (nargin == 0)
            % If function is called without arguments, return parameter tree object
            output = ros.ParameterTree([]);
            return;
        end

        % Parse the specified operation and additional arguments
        [validOperation, paramName, paramValue] = ...
            parseInputs(supportedOperations, operation, varargin{:});

        if nargout == 0
            rosparamImpl(validOperation, paramName, paramValue);
        else
            output = rosparamImpl(validOperation, paramName, paramValue);
        end
    catch ex
        % Save stack traces and exception causes internally, but do not
        % print them to the console
        rosex = ros.internal.ROSException.fromException(ex);
        throwAsCaller(rosex);
    end
else
    %% Codegen
    if (nargin == 0)
        % If function is called without arguments, return parameter tree object
        output = ros.ParameterTree([]);
        return;
    end
    [validOperation, paramName, paramValue] = ...
            parseInputsCodegen(supportedOperations, operation, varargin{:});
     if nargout == 0
            rosparamImplCodegen(validOperation, paramName, paramValue);
     else
            output = rosparamImplCodegen(validOperation, paramName, paramValue);
     end
end

end

function [validOperation, paramName, paramValue] = parseInputs(supportedOperations, operation, varargin)
%parseInputs Parse the inputs to the rosparam function
%   Return the validated operation and the parameter / namespace name (if the user
%   specified one).

    paramName = '';
    paramValue = 0;

    validOperation = validatestring(operation, supportedOperations, 'rosparam', 'operation');

    % Convert strings to characters to ensure that, together with
    % validateattributes, "" is flagged as invalid input.
    if nargin > 1
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    switch validOperation
      case 'list'
        % Possible Syntaxes:
        %    rosparam list
        %    rosparam list NAMESPACE
        narginchk(2,3);

        if nargin == 3
            % Parse user-specified namespace
            paramName = validateParameterNamespace(varargin{1});
        else
            % Assume root namespace by default
            paramName = '/';
        end

      case {'get', 'delete'}
        % Syntax: rosparam OP PNAME
        % where OP is one of {'get', 'delete'}

        narginchk(3,3);
        paramName = validateParameterName(varargin{1});

      case 'set'
        % Syntax: rosparam set PNAME PVALUE

        narginchk(4,4);

        paramName = validateParameterName(varargin{1});
        paramValue = varargin{2};
        validateattributes(paramValue, {'int32', 'double', 'char', 'logical', 'cell', 'struct'}, {});
    end

end

function validNamespace = validateParameterNamespace(namespace)
    validateattributes(namespace, {'char','string'}, {'nonempty', 'scalartext'}, 'rosparam', 'namespace');
    validNamespace = ros.internal.Namespace.canonicalizeName(namespace);
end

function validParamName = validateParameterName(pname)
    validateattributes(pname, {'char','string'}, {'nonempty', 'scalartext'}, 'rosparam', 'pname');
    validParamName = ros.internal.Namespace.canonicalizeName(pname);
end



function output = rosparamImpl(operation, paramName, paramValue)
%rosparamImpl Actual implementation of rosparam functionality.
%   Note that the validation of the input parameters has already been
%   confirmed in the parseInputs function.

% Create a parameter tree object
    pTree = ros.ParameterTree([]);
    node = ros.internal.Global.getNodeHandle(false);

    switch operation
      case 'list'
        % Display a list of parameter names that are registered on the
        % parameter server.
        paramNames = pTree.AvailableParameters;

        % Narrow down list based on specified namespace
        namespace = paramName;

        % Resolve namespace through node
        namespace = node.resolveName(namespace);

        % If namespace is global "/", no filtering is required
        if ~strcmp(namespace, '/')
            % Print only the names of the parameters from specified namespace
            isParamInNamespace = string(paramNames).startsWith(namespace);
            paramNames = paramNames(isParamInNamespace);
        end

        if nargout == 1
            % If output argument specified, return
            output = paramNames;
        else
            % Otherwise, print on console
            for name = paramNames
                disp(char(name));
            end
        end

      case 'set'
        % Set value of a specific parameter
        parsedValue = parseParamValue(paramValue);
        set(pTree, paramName, parsedValue);

        if nargout == 1
            % If output argument specified, return empty
            output = [];
        end

      case 'get'
        % Get the value of a specific parameter
        val = get(pTree,paramName);

        if nargout == 1
            % If there is output specified, return parameter value through
            % that output
            output = val;
            return;
        else
            % If no output argument specified, print to console
            dispString = ros.msg.internal.MessageDisplay.printVariable(val, true);
            disp(dispString);
        end

      case {'delete'}
        % Delete a parameter from the parameter server
        del(pTree, paramName);

        if nargout == 1
            % If output argument specified, return empty
            output = [];
        end
    end

end

function parsedValue = parseParamValue(paramValue)
%parseParamValue Parse a parameter value
%   A scalar number is parsed and returned as double or int32 to mimic the
%   behavior of the Linux rosparam implementation. All other strings are
%   returned verbatim.
%   rosparam in Python uses the YAML parser to interpret a string provided
%   by the user (see http://yaml.org/type/). Note that we only support a
%   subset in MATLAB to correctly interpret scalar numeric values and
%   logicals.

% Set value directly if a data type other than a string is passed
    if ~isa(paramValue, 'char')
        parsedValue = paramValue;
        return;
    end

    % Parse the string to allow setting of numeric scalars
    % Don't allow imaginary numbers, or commas.
    % All of them are treated as strings in the Linux rosparam
    % implementation, so we do the same.

    if string(paramValue).lower.contains('i') ||  string(paramValue).lower.contains(',')
        parsedValue = paramValue;
        return;
    end

    % Special case: Value is a boolean. See http://yaml.org/type/bool.html.
    if contains(paramValue,'true') || contains(paramValue,'True') || contains(paramValue,'TRUE')
        parsedValue = true;
        return;
    end
    if contains(paramValue,'false') || contains(paramValue,'False') || contains(paramValue,'FALSE')
        parsedValue = false;
        return;
    end

    % Try calling str2double. It returns NaN if the conversion was not
    % successful. For example, lists [..] and dictionaries .. : .. are

    paramValueDouble = real(str2double(paramValue));
    if isnan(paramValueDouble)
        % Conversion to scalar double was not successful. Set as
        % string.
        parsedValue = paramValue;
        return;
    end

    % Conversion to scalar double was successful.
    if ~string(paramValue).contains('.')
        % If value is an integer, set as int32
        parsedValue = coder.const(int32(paramValueDouble));
    else
        % Otherwise, set as double
        parsedValue = paramValueDouble;
    end
end

function [validOperation, paramName, paramValue] = parseInputsCodegen(supportedOperations, operation, varargin)
%parseInputs Parse the inputs to the rosparam function
%   Return the validated operation and the parameter / namespace name (if the user
%   specified one).

paramName = '';
validOperation = validatestring(operation, supportedOperations, 'rosparam', 'operation');

switch validOperation
    case 'list'
        % Possible Syntaxes:
        %    rosparam list
        %    rosparam list NAMESPACE
        narginchk(2,3);
        paramValue = '';

        if nargin == 3
            % Parse user-specified namespace
            validateattributes(varargin{1}, {'char','string'}, {'nonempty', 'scalartext'}, 'rosparam', 'namespace');
            paramName = convertStringsToChars(varargin{1});
        else
            % Assume root namespace by default
            paramName = '/';
        end

    case 'get'
        % Syntax: rosparam get PNAME DataType PType

        narginchk(3,5);
        validateattributes(varargin{1}, {'char','string'}, {'nonempty', 'scalartext'}, 'rosparam', 'namespace');

        opArgs = {};
        % Define the parameter names
        NVPairNames = {'DataType'};
        % Select parsing options
        pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
        % Parse the inputs
        pStruct = coder.internal.parseInputs(opArgs, NVPairNames,pOpts,varargin{2:end});
        % Retrive name-value pairs
        mlDataType = coder.internal.getParameterValue(pStruct.DataType,'double',varargin{2:end});
        mlDataType = convertStringsToChars(mlDataType);
        validateattributes(mlDataType, {'int32','single' 'double',...
            'logical','char','string'},{},2);
        paramName = convertStringsToChars(varargin{1});
        paramValue = mlDataType;

    case 'delete'
        % Syntax: rosparam delete PNAME

        narginchk(3,3);
        validateattributes(varargin{1}, {'char','string'}, {'nonempty', 'scalartext'}, 'rosparam', 'pname');
        paramName = convertStringsToChars(varargin{1});
        paramValue = '';

    case 'set'
        % Syntax: rosparam set PNAME PVALUE

        narginchk(4,4);
        validateattributes(varargin{1}, {'char','string'}, {'nonempty', 'scalartext'}, 'rosparam', 'pname');
        paramName = convertStringsToChars(varargin{1});
        paramValue = convertStringsToChars(varargin{2});
        validateattributes(paramValue, {'int32', 'double', 'char', 'logical', 'cell', 'struct'}, {});

    otherwise
        paramValue = 0;
end
end

function output = rosparamImplCodegen(operation, paramName, paramValue)
%rosparamImpl Actual implementation of rosparam functionality.
%   Note that the validation of the input parameters has already been
%   confirmed in the parseInputs function.

% Create a parameter tree object
pTree = ros.ParameterTree([]);
if ~strcmp(paramName, '/')
    paramName = ros.internal.codegen.ParameterTree.canonicalizeName(pTree,paramName);
end

switch operation
    case 'list'
        % Display a list of parameter names that are registered on the
        % parameter server.
        paramNames = pTree.AvailableParameters;
        coder.varsize('paramNames')

        % Narrow down list based on specified namespace
        namespace = char(zeros(1,numel(paramName)));

        % Resolve namespace
        coder.ceval('pTree.ParameterHelper.resolveName', paramName, coder.ref(namespace));

        % If namespace is global "/", no filtering is required
        if ~strcmp(namespace, '/')
            % Print only the names of the parameters from specified namespace
            for index = 1:numel(paramNames)
                isParamInNamespace = string(paramNames{index}).startsWith(string(namespace));
                if ~isParamInNamespace
                    paramNames{index} = '';
                end
            end
        end

        if nargout == 1
            % If output argument specified, return
            output = paramNames;
        else
            % Otherwise, print on console
            for index=1:numel(paramNames)
                fprintf("%s\n", char(paramNames{index}));
            end
        end

    case 'set'
        % Set value of a specific parameter
        parsedValue = parseParamValue(paramValue);
        set(pTree, paramName, parsedValue);

        if nargout == 1
            % If output argument specified, return empty
            output = [];
        end

    case 'get'
        % Get the value of a specific parameter
        [val,~] = get(pTree,paramName, 'DataType', paramValue);

        if nargout == 1
            % If there is output specified, return parameter value through
            % that output
            output = val;
            return;
        end

    case {'delete'}
        % Delete a parameter from the parameter server
        paramName = ros.internal.codegen.ParameterTree.canonicalizeName(pTree,paramName);
        del(pTree, paramName);

        if nargout == 1
            % If output argument specified, return empty
            output = [];
        end
end
end
