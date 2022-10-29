classdef ParameterTree < ros.internal.mixin.ROSInternalAccess & ...
        coder.ExternalDependency
% This class is for internal use only. It may be removed in the future.
%
% ParameterTree - Code generation equivalent for ros.ParameterTree
% See also ros.ParameterTree
%
%#codegen

% Copyright 2021 The MathWorks, Inc.

    properties (Access = private)
        %ParameterHelper - helper member
        ParameterHelper
    end

    properties (Dependent, SetAccess = private)
        %AvailableParameters - List of parameter names on server
        %   This property is an Nx1 cell array of strings containing the
        %   names of all parameters that are currently registered on the
        %   parameter server.
        AvailableParameters
    end

    properties (Constant, Access = private)
        %% Root Namespace
        GLOBALNS = '/'
    end

    methods
        function obj = ParameterTree(node)
        %ParameterTree Constructor for the ParameterTree
        %   This function takes a empty node to the input,

        % Ensure input arguments are as expected
            if ~isempty(node)
                % A node cannot create another node in code generation
                coder.internal.assert(false,'ros:mlroscpp:codegen:NodeMustBeEmpty');
            end

            % Create an instance of MATLABROSParameter object and
            % store handle to MATLABROSParameter object
            obj.ParameterHelper = coder.opaque('MATLABROSParameter','HeaderFile','mlroscpp_param.h');
            obj.ParameterHelper = coder.ceval('MATLABROSParameter');
            coder.ceval('UNUSED_PARAM',obj.ParameterHelper);
        end

        function paramNames = get.AvailableParameters(obj)
        %get.AvailableParameters Get AvailableParameters property
        % Get the list of parameter names from the parameter tree

            numberOfParameters = int32(0);
            coder.cinclude('<functional>');
            numberOfParameters = coder.ceval('std::mem_fn(&MATLABROSParameter::getNumOfROSParameters)', ...
                                             coder.ref(obj.ParameterHelper));
            paramNames = cell(numberOfParameters,1);

            coder.ceval('std::mem_fn(&MATLABROSParameter::sortParameters)', coder.ref(obj.ParameterHelper));
            sizeUpperBound = 1e9; % This large number ensures arrays are handled using coder::array
            coder.varsize('paramEntry',[1 sizeUpperBound], [0 1]);

            for key=1:numel(paramNames)
                index = int32(key - 1);
                paramSize = int32(0);
                paramSize = coder.ceval('std::mem_fn(&MATLABROSParameter::getParameterLength)', ...
                                        coder.ref(obj.ParameterHelper), index);
                paramEntry = char(zeros(1,paramSize));
                coder.ceval('std::mem_fn(&MATLABROSParameter::getAvailableParameter)', ...
                            coder.ref(obj.ParameterHelper), index, coder.ref(paramEntry));
                paramNames{key} = paramEntry;
            end
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
            validateattributes(paramValue,{'int32','single','double','char','string',...
                                           'logical','cell','struct'},{},'set','value');

            % The following line handles validation of the parameter name, it
            % adds namespace if an environment variable is defined and adds
            % the node name for private parameters
            coder.cinclude('<functional>');
            parameterName = ros.internal.codegen.ParameterTree.canonicalizeName(obj,paramName);
            parameterName = ros.internal.codegen.ParameterTree.cString(parameterName);

            if isnumeric(paramValue) || islogical(paramValue)
                % Set scalar numeric or logical value
                validateattributes(paramValue,{'int32','single','double',...
                                               'logical'},{'nonempty','scalar'},'set', 'value')
                cppType = coder.const(obj.mapToCppType(class(paramValue)));
                formattedParamValue = paramValue;
                coder.ceval(['std::mem_fn(&MATLABROSParameter::setParameter<' cppType '>)'], ...
                            obj.ParameterHelper, parameterName, formattedParamValue);
            elseif ischar(paramValue) || isstring(paramValue)
                % Set a string
                paramValue = robotics.internal.validation.validateString(paramValue, true, 'set', 'value');
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
                        coder.internal.assert(false, 'ros:mlros:param:SetStringParamError',paramName,k);
                    end
                end
                formattedParamValue = paramValue;
                cppType = coder.const(obj.mapToCppType(class(paramValue)));
                coder.ceval(['std::mem_fn(&MATLABROSParameter::setParameter<' cppType '*>)'], ...
                            obj.ParameterHelper, parameterName, coder.ref(formattedParamValue));
            elseif iscell(paramValue)
                % Set cell array as a list
                % Recursive validation of each element
                obj.validateCellInput(paramValue);

                % Map MATLAB types to c++ types
                if isempty(paramValue)
                    formattedParamValue = coder.opaque('double *', 'nullptr');
                    coder.ceval('-layout:rowMajor','std::mem_fn(&MATLABROSParameter::setArrayParameter<double>)', ...
                                obj.ParameterHelper, parameterName, formattedParamValue, uint32(0));
                else
                    mlDataType = class(paramValue{1});
                    if ischar(paramValue{1}) || isstring(paramValue{1})

                        coder.cinclude("<string>");
                        arrSize = uint32(numel(paramValue));
                        coder.varsize('formattedParamValue');

                        for strIndex = 1:arrSize
                            [nRows,nCols] = size(paramValue{strIndex});
                            formattedParamValue = cast(zeros([nRows nCols+1]), 'char');
                            strLen = numel(paramValue{strIndex});
                            for charIndex = 1:strLen
                                formattedParamValue(charIndex) = paramValue{strIndex}(charIndex);
                            end
                            coder.ceval('-layout:rowMajor','std::mem_fn(&MATLABROSParameter::setStringValues)', ...
                                        obj.ParameterHelper, parameterName, formattedParamValue);
                        end
                        coder.ceval('-layout:rowMajor','std::mem_fn(&MATLABROSParameter::setStringArrayParameter)', ...
                                    obj.ParameterHelper, parameterName);
                    else
                        cppType = coder.const(obj.mapToCppType(mlDataType));
                        formattedParamValue = cast(zeros(size(paramValue)), mlDataType);
                        arrSize = uint32(numel(paramValue));
                        for arrIndex = 1:numel(paramValue)
                            formattedParamValue(arrIndex) = paramValue{arrIndex};
                        end

                        coder.ceval('-layout:rowMajor',['std::mem_fn(&MATLABROSParameter::setArrayParameter<' cppType '>)'], ...
                                    obj.ParameterHelper, parameterName, formattedParamValue, arrSize);
                    end
                end
            elseif isstruct(paramValue)
                % Setting struct fields of ros parameter is not
                % supported in codegen
                coder.internal.assert(false, 'ros:mlroscpp:codegen:UnsupportedDataElementStruct', 'set', 'struct');
            end
        end

        function [data, status] = get(obj, paramName, varargin)
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

        % Validate the input argument
            narginchk(2, 4);
            paramName = robotics.internal.validation.validateString(paramName, true, 'get', 'name');

            coder.internal.prefer_const(varargin{:});
            opArgs = {};
            % Define the parameter names
            NVPairNames = {'DataType'};
            % Select parsing options
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            % Parse the inputs
            pStruct = coder.internal.parseInputs(opArgs, NVPairNames,pOpts,varargin{:});
            % Retrive name-value pairs
            mlDataType = coder.internal.getParameterValue(pStruct.DataType,'double',varargin{:});

            validatestring(mlDataType, {'int32','single' 'double',...
                                        'logical','char','string'},'get','DataType',4);

            % The following line handles validation of the parameter name
            coder.cinclude('<functional>');
            parameterName = ros.internal.codegen.ParameterTree.canonicalizeName(obj,paramName);
            parameterName = ros.internal.codegen.ParameterTree.cString(parameterName);

            % Get the data from the parameter server
            [cppType, outData] = coder.const(@ros.internal.codegen.ParameterTree.mapToCppType, mlDataType);
            status = false;

            if strcmpi(mlDataType, 'string') || strcmpi(mlDataType, 'char')
                status = coder.ceval('std::mem_fn(&MATLABROSParameter::getStringParameter)', ...
                                     coder.ref(obj.ParameterHelper), parameterName, coder.ref(outData));
            else
                status = coder.ceval(['std::mem_fn(&MATLABROSParameter::getParameter<' cppType '>)'], ...
                                     coder.ref(obj.ParameterHelper), parameterName, coder.ref(outData));
            end

            statusIndicator = ~status;
            if ~statusIndicator
                coder.internal.error('ros:mlros:param:GetParameterError',parameterName);
            end
            data = outData;
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
            coder.cinclude('<functional>');
            parameterName = ros.internal.codegen.ParameterTree.canonicalizeName(obj,paramName);
            parameterName = ros.internal.codegen.ParameterTree.cString(parameterName);
            % Get the list of parameters in the parameter tree
            % Not using AvailableParameters to avoid recursion errors
            nameExists = false;
            nameExists = coder.ceval('std::mem_fn(&MATLABROSParameter::hasParam)', ...
                                     obj.ParameterHelper, parameterName);
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

            if nargout == 2
                % Code generation is not support to retrieve values
                coder.internal.assert(false,'ros:mlroscpp:codegen:UnsupportedOutputCodegen','search');
            end

            % Validate input arguments, allows empty string
            paramName = robotics.internal.validation.validateString(paramName, true, 'search', 'name');

            % Get the parameter list
            allnames = obj.AvailableParameters;

            % Return empty if parameter list is empty
            if isempty(allnames)
                pnames = {};
                pvalues = {};
                return;
            end

            paramNames = cell(1,numel(allnames));
            for i=1:numel(paramNames)
                paramNames{i} = allnames{i};
            end

            % If input is empty, return entire tree
            if isempty(paramName)
                pnames = paramNames;
            else
                % Find matching string and return the matching parameter list
                coder.varsize('pnames');
                pnames = {};
                for i=1:numel(paramNames)
                    if contains(lower(paramNames{i}), lower(paramName))
                        pnames{end+1} = paramNames{i};
                    end
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
            coder.cinclude('<functional>');
            parameterName = ros.internal.codegen.ParameterTree.canonicalizeName(obj,paramName);
            parameterName = ros.internal.codegen.ParameterTree.cString(parameterName);

            if ~has(obj,parameterName)
                coder.internal.error('ros:mlros:param:ParameterDoesNotExist',parameterName);
            end

            ret = false;
            ret = coder.ceval('std::mem_fn(&MATLABROSParameter::deleteParam)', ...
                              obj.ParameterHelper, parameterName);
            if ~ret
                coder.internal.error('ros:mlros:param:DeleteParameterError',paramName);
            end
        end
    end

    methods (Access = private)
        function validateCellInput(obj,cellArray)
        %validateCellInput Validate elements of cell
        %   This function is used for recursively validating every
        %   element of the input cell array. The elements of cell array
        %   can only take 'char', 'logical', 'single', 'double', 'int32'

            if ~isempty(cellArray)
                % Validate if cell array is a vector (row or column)
                validateattributes(cellArray, {'cell'}, {'vector'}, 'set', 'value');

                % Check data type for every element
                firstElementType = class(cellArray{1});
                for elementIndex = 1:numel(cellArray)
                    isHomogeneousInType = strcmp(class(cellArray{elementIndex}), firstElementType);
                    if ~isHomogeneousInType
                        coder.internal.assert(false, 'ros:mlroscpp:codegen:CellArrayElementDataTypesMismatch');
                    end
                    obj.checkDataType(cellArray{elementIndex});
                end
            end
        end

        function checkDataType(obj,inputData)
        %checkDataType Check data type
        %   This function is used for recursively checking the input
        %   data type for each element of the cell array

        % If cell array, then call this function recursively
            if iscell(inputData)
                coder.ceval('UNUSED_PARAM',obj.ParameterHelper);
                coder.internal.assert(false, 'ros:mlroscpp:codegen:UnsupportedDataElementCell');
            end

            if isstruct(inputData)
                coder.ceval('UNUSED_PARAM',obj.ParameterHelper);
                coder.internal.assert(false, 'ros:mlroscpp:codegen:UnsupportedDataElementStruct', 'set', 'struct');
            end

            if isnumeric(inputData)
                % Check if numeric data is scalar
                validateattributes(inputData,{'numeric'},{'scalar'},2)
            end

            % Check if data belongs to supported types
            validateattributes(inputData,{'int32','single', 'double',...
                                          'logical','char','string'},{},2)
        end
    end

    methods (Static)
        function ret = getDescriptiveName(~)
            ret = 'ROS ParameterTree';
        end

        function ret = isSupportedContext(bldCtx)
            ret = bldCtx.isCodeGenTarget('rtw');
        end

        function updateBuildInfo(buildInfo, bldCtx)
            if bldCtx.isCodeGenTarget('rtw')
                srcFolder = ros.slros.internal.cgen.Constants.PredefinedCode.Location;
                addIncludeFiles(buildInfo,'mlroscpp_param.h',srcFolder);
            end
        end

        function validName = canonicalizeName(obj, name)
        %canonicalizeName Validate ROS name
        %   Validate and convert the ROS name into its canonical representation.
        %   Canonical representations have no trailing slashes and can be
        %   global (/...), private (~...), or relative. Please note
        %   that this function will not change the type of
        %   representation. For example, a relative name will not be
        %   changed to an absolute one.
        %
        %   An empty NAME string is a valid relative graph name (and
        %   will be converted to absolute once resolved).
        %
        %   This function will throw an error if the name is not a valid
        %   ROS graph resource name.
        %
        %   See also ros.internal.Namespace.isValidGraphName

            validateattributes(name, {'char'}, {}, 'canonicalizeName', 'name');

            if isempty(name)
                validName = '';
                return
            end

            validPattern = false;
            validPattern = coder.ceval('std::mem_fn(&MATLABROSParameter::isValidPattern)',...
                                       coder.ref(obj.ParameterHelper), name);
            if ~validPattern
                coder.internal.error('ros:mlros:util:NameInvalid', 'graph', name);
            end

            coder.varsize('parameterName');
            parameterName = name;
            while ~strcmp(name, ros.internal.codegen.ParameterTree.GLOBALNS) && (name(end) == ros.internal.codegen.ParameterTree.GLOBALNS)
                parameterName = name(1:end-1);
            end

            if startsWith(parameterName,'~/')
                relativeParameterName = "~" + parameterName(3:end);
                parameterName = char(relativeParameterName);
            end

            validName = char(parameterName);
        end
    end

    methods (Access = private)
        function doNotOptimize(obj)
        %DONOTOPTIMIZE - avoid optimizing away codes during Code Generation
            coder.ceval('//',coder.wref(obj.ParameterHelper));
        end
    end

    % Static helpers
    methods(Static, Access=private)

        % Put a C termination character '\0' at the end of MATLAB character vector
        function out = cString(in)
            out = [in char(0)];
        end

        function [cppDataType, defaultInitialValue] = mapToCppType(mlDataType)
            switch(coder.const(mlDataType))
              case 'logical'
                cppDataType = coder.internal.const('bool');
                defaultInitialValue = false;
              case 'int32'
                cppDataType = coder.internal.const('int');
                defaultInitialValue = int32(0);
              case 'single'
                cppDataType = coder.internal.const('float');
                defaultInitialValue = single(0);
              case 'double'
                cppDataType = coder.internal.const('double');
                defaultInitialValue = double(0);
              case 'char'
                cppDataType = coder.internal.const('char');
                defaultInitialValue = '';
              case 'string'
                cppDataType = coder.internal.const('char*');
                defaultInitialValue = char(zeros(1,256));
              otherwise
                coder.internat.assert(false, 'ros:mlroscpp:codegen:UnsupportedDataType',mlDataType);
            end
        end
    end
end
