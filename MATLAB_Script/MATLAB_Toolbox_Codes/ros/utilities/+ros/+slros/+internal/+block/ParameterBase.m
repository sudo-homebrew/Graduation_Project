classdef (Abstract) ParameterBase < ...
        matlab.System & ...
        ros.slros.internal.block.mixin.NodeDependent
    %This class is for internal use only. It may be removed in the future.

    %ParameterBase Base class for ROS parameter blocks

    %   Copyright 2015-2021 The MathWorks, Inc.

    %#codegen

    properties (Nontunable)
        %ParameterName Name
        %   This system object will use ParameterName as specified in both
        %   simulation and code generation. In particular, it will not add a
        %   "/" in front of the name, as that forces the topic to be in the
        %   absolute namespace.
        ParameterName = '/my_param'

        %ParameterSource Source
        %   By default, the user can choose a parameter name from the ROS
        %   network.
        ParameterSource = ros.slros.internal.block.CommonMask.TopicSourceFromNetwork

        %ParameterType Data type
        %   This is the data type of the Value output. Possible values are
        %   double, int32, uint8[] (string), and boolean. This data type is only valid in
        %   Simulink. For the corresponding MATLAB data type string, refer
        %   to the ParameterTypeML property.
        %   Default: 'double'
        ParameterType = 'double'
    end

    % The following should ideally not show up in the MATLAB System block
    % dialog. However, setting them as 'Hidden' will prevent them from
    % being accessible via set_param & get_param.
    %
    %   ModelName is needed for managing the node instance
    %   BlockId is needed to generate a unique identifier in codegen
    properties (Nontunable)
        %ModelName - Name of Simulink model
        ModelName = ros.slros.internal.block.mixin.NodeDependent.DefaultModelName

        %BlockId - Simulink Block Identifier
        %   Used to generate unique identifier for the block during code
        %   generation. This should be obtained using Simulink.ID.getSID()
        %   on the library block (*not* the MATLAB system block). The
        %   SID has the format '<modelName>:<blocknum>'
        BlockId = 'param_'

        %StreamWSVar - Get and set parameters from variable in global scope
        %   Used to redirect system objects to get and set parameters from a workspace
        %   variable instead of from ROS network (used for testing only).
        %   The workspace variable should be created like this:
        %
        %   workspaceVar = ros.slros.internal.sim.ParameterListStream;
        %   workspaceVar.ParameterList = {paramValue1, paramValue2, paramValue3} );
        %
        %   This capability of using workspace variables is only used in
        %   internal testing, and is not documented for Simulink users.
        StreamWSVar = ''
    end

    properties (Nontunable, Hidden)
        %ParameterTypeML - The ROS parameter type as MATLAB data type
        %   To avoid always converting back and forth between the two
        %   representations, I convert it once whenever the ParameterType
        %   is set.
        ParameterTypeML = 'double'
    end

    properties (Access = protected, Transient)
        %ParameterStream - Handle to object that implements ros.slros.internal.sim.ParameterStream
        %   This can be a sim.ParameterNetStream (for getting messages off
        %   the ROS network) or sim.ParameterListStream (for getting messages
        %   from an object in the global scope; used in conjunction with
        %   StreamWsVar property)
        ParameterStream
    end

    properties (Abstract, Access = protected)
        %StateHandler - Object handling the state of the parameter value
        %   The derived classes will use a different object for handling
        %   scalar and array values.
        StateHandler
    end

    % These are the options for the dropdown lists
    % The properties are constant, hidden, and transient based on the
    % system object documentation
    properties (Constant, Hidden, Transient)
        %ParameterSourceSet - Dropdown choices for parameter source
        ParameterSourceSet = matlab.system.StringSet({...
            ros.slros.internal.block.CommonMask.TopicSourceFromNetwork, ...
            ros.slros.internal.block.CommonMask.TopicSourceSpecifyOwn});

        %ParameterTypeSet - Dropdown choices for parameter data type
        ParameterTypeSet = matlab.system.StringSet({'double', 'int32', 'boolean', ...
                            ros.slros.internal.sim.DataTypes.SimulinkStringType});
    end

    properties(Constant, Access = protected)
        %HeaderFile - Name of header file with declarations for variables and types
        %   It is referred to in code emitted by setupImpl and stepImpl.
        HeaderFile = ros.slros.internal.cgen.Constants.InitCode.HeaderFile
    end

    methods
        function set.ParameterName(obj, val)
        %set.ParameterName Set the parameter name
        %   The name is validated based on standard ROS naming rules
        %   and an error is thrown if the parameter name does not
        %   conform.

            validateattributes(val, {'char'}, {'nonempty'}, '', 'ParameterName');
            if coder.target('MATLAB')
                ros.internal.Namespace.canonicalizeName(val); % throws error
            end
            obj.ParameterName = val;
        end

        function set.ParameterType(obj, val)
        %set.ParameterType Set the parameter type
        %   This is already a dropdown, so no separate data type
        %   validation is necessary.
        %   Convert the Simulink data type into the corresponding
        %   MATLAB data type.

            obj.ParameterTypeML = ros.slros.internal.sim.DataTypes.simulinkToMatlab(val); %#ok<MCSUP>
            obj.ParameterType = val;

            % Set appropriate state handler for scalar and array parameters
            obj.initializeStateHandler;
        end

        function set.ModelName(obj, val)
        %set.ModelName Set model name property

            validateattributes(val, {'char'}, {'nonempty'}, '', 'ModelName');
            obj.ModelName = val;
        end

        function set.BlockId(obj, val)
        %set.BlockId Set block ID property

            validateattributes(val, {'char'}, {'nonempty'}, '', 'BlockId');
            obj.BlockId = val;
        end

        function set.StreamWSVar(obj, val)
        %set.StreamWSVar Set parameter stream property

            validateattributes(val, {'char'}, {'nonempty'}, '', 'StreamWSVar');
            obj.StreamWSVar = val;
        end

    end

    %% Methods that are implementations of abstract NodeDependent mixin
    methods (Access = protected)
        function initializeDataStream(obj, modelState)
        %initializeDataStream Initialize parameter data stream
            obj.initializeParameterStream(modelState);
        end

        function name = modelName(obj)
            name = obj.ModelName;
        end
    end

    methods (Access = protected)

        function sts = getSampleTimeImpl(obj)
        % Enable this system object to inherit constant ('inf') sample times
            sts = createSampleTime(obj, 'Type', 'Inherited', 'Allow', 'Constant');
        end

        function initializeStateHandler(obj)
        %initializeStateHandler Set up the state handler

        % Make sure that StateHandler is initialized correctly
            isScalarType = ros.slros.internal.sim.DataTypes.isSimulinkDataTypeScalar(obj.ParameterType);
            obj.setStateHandler(isScalarType);
        end

        function setupImpl(obj)
        %setupImpl Model initialization call
        %   setupImpl is called when model is being initialized at the
        %   start of a simulation.

            obj.setupNodeDependent;
        end

        %%
        function releaseImpl(obj)
        %releaseImpl Release all resources

            obj.releaseNodeDependent;
        end

        % We don't save SimState, since there is no way save & restore
        % the GetParameter object. However, saveObjectImpl and loadObjectImpl
        % are required since we have private properties.
        function s = saveObjectImpl(obj)
        % The errorIf() below will ensure that FastRestart cannot be used
        % in a model with ROS blocks
            coder.internal.error('ros:slros:nodedependent:SimStateNotSupported', ...
                                 obj.MessageCatalogName);
            s = saveObjectImpl@matlab.System(obj);
        end

        function initializeParameterStream(obj, modelState)
        %initializeParameterStream Initialize the ParameterStream property
        %   The stream is either connected to the ROS network or
        %   connected to a workspace variable.

            if ~isempty(obj.StreamWSVar)
                % Use a variable in MATLAB Workspace as source of messages.
                obj.ParameterStream = evalin('base', obj.StreamWSVar);
                validateattributes(obj.ParameterStream, {'ros.slros.internal.sim.ParameterListStream'}, {'scalar'}, ...
                                   'setup', 'ParameterStream');
            else
                % Use ROS network as a source of parameters.
                if isempty(modelState.ROSNode) || ~isvalid(modelState.ROSNode)
                    obj.ROSMaster = ros.slros.internal.sim.ROSMaster();
                    %  verifyReachable() errors if ROS master is unreachable
                    obj.ROSMaster.verifyReachable();
                    % createNode() errors if unable to create node
                    % (e.g., if node with same name already exists)
                    uniqueName = obj.ROSMaster.makeUniqueName(obj.ModelName);
                    modelState.ROSNode = obj.ROSMaster.createNode(uniqueName);
                end

                % Initialize the MATLAB ROS parameter stream
                obj.ParameterStream = ros.slros.internal.sim.ParameterNetStream(...
                    obj.ParameterName, modelState.ROSNode);
            end
        end
    end

    methods(Static, Access = protected)
        function simMode = getSimulateUsingImpl
        %getSimulateUsingImpl Restrict simulation mode to interpreted execution
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
        %showSimulateUsingImpl Do now show simulation execution mode dropdown in block mask
            flag = false;
        end
    end

    %% Methods that need to be implemented by derived classes
    methods (Abstract, Access = protected)
        setStateHandler(obj, isScalar)
        %setStateHandler Set the correct state handler object (for scalar or array parameter)
    end
end
