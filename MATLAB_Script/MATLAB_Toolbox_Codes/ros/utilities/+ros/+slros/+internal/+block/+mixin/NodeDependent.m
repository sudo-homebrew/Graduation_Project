classdef (Abstract) NodeDependent < handle
%NodeDependent Base class for all ROS blocks that depend on a ROS node
%   This class handles the ROS Master connection gracefully and also
%   cleans up on release.
%   In addition, this base class is handling the switching between data
%   streams received via ROS and from the workspace.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    properties (Constant, Access=?ros.slros.internal.block.mixin.NodeDependent)
        %DefaultModelName - Default model name
        %   Note that since the name contains a forward slash, it is not a valid
        %   Simulink model name. This way, we can always distinguish if the
        %   ModelName is still the default or not.
        DefaultModelName = 'de/fault'
    end

    properties (Access = protected, Transient)
        %ROSMaster - Handle to an object that handles ROS master interaction
        %   It is initialized to indicate the class of the object.
        ROSMaster = ros.slros.internal.sim.ROSMaster.empty
    end

    properties (Abstract, Constant, Access=?ros.slros.internal.block.mixin.NodeDependent)
        %MessageCatalogName - Name of this block used in message catalog
        %   This has to be a constant, so we can throw a compile time error
        %   message to the user.
        %   The Access rights also have to use the meta class of
        %   NodeDependent, otherwise Simulink Coder throws an access error.
        MessageCatalogName
    end

    methods (Abstract, Access = protected)
        initializeDataStream(obj, modelState)
        %initializeDataStream Initialize the data stream for this block

        modelName(obj)
        %modelName Return name of model
    end

    methods (Access = {?ros.slros.internal.block.mixin.NodeDependent, ...
                       ?matlab.unittest.TestCase})

        function setupNodeDependent(obj)
        %setupNodeDependent Model initialization call
        %   setupImpl is called when model is being initialized at the
        %   start of a simulation.

            mdlName = obj.modelName;

            if coder.target('MATLAB')
                % Executing in MATLAB interpreted mode
                modelHasPriorState = ros.slros.internal.sim.ModelStateManager.hasState(mdlName);

                try
                    modelState = ros.slros.internal.sim.ModelStateManager.getState(mdlName, 'create');

                    % Initialize data stream (typically either from ROS network
                    % or from workspace variable)
                    obj.initializeDataStream(modelState);
                catch ME
                    if ~modelHasPriorState || ~modelState.nodeHasReferrers()
                        ros.slros.internal.sim.ModelStateManager.clearState(mdlName);
                    end
                    % RETHROW will generate a hard-to-read stack trace, so
                    % use THROW instead.
                    throw(ME);
                end

                % Increment the node reference count if the data stream was
                % initialized without exceptions.
                modelState.incrNodeRefCount();

            elseif coder.target('RtwForRapid')
                % Rapid Accelerator. In this mode, coder.target('Rtw')
                % returns true as well, so it is important to check for
                % 'RtwForRapid' before checking for 'Rtw'

                % Throw a compile-time error
                coder.internal.errorIf(true, 'ros:slros:nodedependent:RapidAccelNotSupported', ...
                                       obj.MessageCatalogName);

            elseif coder.target('Rtw')
                % ROS target code generation.
                % Do nothing. Write custom code in derived class
                % implementations.

            elseif  coder.target('Sfun')
                % 'Sfun'  - SimThruCodeGen target
                % Do nothing. MATLAB System block first does a pre-codegen
                % compile with 'Sfun' target, & then does the "proper"
                % codegen compile with Rtw or RtwForRapid, as appropriate.

            else
                % 'RtwForSim' - ModelReference SIM target
                % 'MEX', 'HDL', 'Custom' - Not applicable to MATLAB System block

                % Throw a compile-time error
                coder.internal.errorIf(true, 'ros:slros:sysobj:UnsupportedCodegenMode', coder.target);
            end
        end

        function releaseNodeDependent(obj)
        %releaseNodeDependent Release all resources

            mdlName = obj.modelName;

            if coder.target('MATLAB')
                st = ros.slros.internal.sim.ModelStateManager.getState(mdlName);
                st.decrNodeRefCount();

                if  ~st.nodeHasReferrers()
                    ros.slros.internal.sim.ModelStateManager.clearState(mdlName);
                end
            else % coder.target('Rtw')
                 % Nothing to do. The system object will never be released
                 % in the generated node.
            end
        end
    end
end
