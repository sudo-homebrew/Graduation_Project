classdef IKBlock < matlab.System
    % This class is for internal use only. It may be removed in the future.
    
    %IKBLOCK Inverse Kinematics block system object

    % Copyright 2018-2020 The MathWorks, Inc.

    %#codegen
    
    properties(Nontunable)
        
        %TreeStruct - Struct from which wrapper SysObj builds the TreeInternal
        TreeStruct = 0;
        
        %TreeInternal - A robotics.manip.internal.RigidBodyTree instantiated from TreeStruct
        TreeInternal
        
        %IKInternal - The inverseKinematics object
        IKInternal
        
        %EEBodyName - The name of the End Effector body
        EEBodyName = 0;
        
        %SolverName - The name of the solver
        SolverName = 0;
        
        %SolverParameters - Structure of solver parameters
        SolverParameters = 0;
        
        %InfoBusName
        InfoBusName
    
        %ShowInfo
        ShowInfo (1, 1) logical = true;
        
        %UseTimer Flag to explicitly control when timer is used.0 
        % When a user specifies the MaxTime, this is set to TRUE, and the
        % underlying IK object calls the System Time Provider. While this
        % will work in simulation, it is not compatible with cross-platform
        % deployment, since the System Time Provider is platform specific.
        % When a user sets the MaxTime to infinity, this is set to FALSE,
        % and the System Time provider will not be called, enabling
        % cross-functional deployment.
        UserTimer (1, 1) logical = true;
    end

    methods
        function obj = IKBlock(varargin)
            %IKBlock Constructor for Inverse Kinematics block system object
            
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            %setupImpl Perform one-time calculations, such as computing constants
            
            % Determine whether to check max time
            if isfinite(obj.SolverParameters.MaxTime)
                obj.UserTimer = true;
            else
                obj.UserTimer = false;
            end
            
            % Assign the robot internal property by instantiating from the structure      
            obj.TreeInternal = robotics.manip.internal.RigidBodyTree(obj.TreeStruct.NumBodies, obj.TreeStruct);
            obj.IKInternal = inverseKinematics('RigidBodyTree', obj.TreeInternal, ...
                'SolverAlgorithm', obj.SolverName, 'SolverParameters', obj.SolverParameters, ...
                'UserTimer', obj.UserTimer);
        end

        function varargout = stepImpl(obj,pose,weights,q0)   
            %stepImpl Call IK object to generate a solution & info
            
            % Ensure that weights is passed as a row vector
            wRow = coder.nullcopy(ones(1,6));
            wRow(:) = weights;
            [configVec, infoStruct] = obj.IKInternal(obj.EEBodyName, double(pose), wRow, double(q0));
            varargout{1} = cast(configVec, 'like', pose);
            
            if obj.ShowInfo
                info.Iterations = infoStruct.Iterations;
                info.PoseErrorNorm = infoStruct.PoseErrorNorm;
                info.ExitFlag = uint16(infoStruct.ExitFlag);
                if strcmp(infoStruct.Status, 'success')
                    info.Status = uint8(1);
                else
                    info.Status = uint8(2);
                end
                varargout{2} = info;
            end
        end

        function resetImpl(~)
            %resetImpl Initialize / reset discrete-state properties
        end
        
        function validateInputsImpl(obj,pose,weights,q0)
            %validateInputsImpl Validate inputs to the step method at initialization
            validateattributes(pose,{'single','double'},{'size', [4 4]},'IKBlock','pose'); 
            validateattributes(weights,{'single','double'},{'vector','numel',6},'IKBlock','weights');
            validateattributes(q0,{'single','double'},{'vector','numel',obj.TreeStruct.VelocityNumber},'IKBlock','q0');  
        end

        function flag = isInputSizeMutableImpl(~,~)
            %isInputSizeMutableImpl Specify input size mutability
            %   Return false if input size cannot change
            %   between calls to the System object
            flag = false;
        end

        function flag = isInputDataTypeMutableImpl(~,~)
            %isInputDataTypeMutableImpl Specify input type mutability
            %   Return false if input data type cannot change
            %   between calls to the System object
            flag = false;
        end

        function num = getNumOutputsImpl(obj)
            %getNumOutputsImpl Define total number of outputs for system with optional outputs
            num = 1;
            if obj.ShowInfo
                num = 2;
            end
        end
        
        function varargout = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            varargout{1} = [obj.TreeStruct.VelocityNumber 1];
            if obj.ShowInfo
                varargout{2} = [1 1]; %Simulink bus
            end
        end

        function varargout = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            varargout{1} = propagatedInputDataType(obj,1);
            if obj.ShowInfo
                varargout{2} = obj.InfoBusName;
            end
        end

        function varargout = isOutputComplexImpl(obj)
            %isOutputComplexImpl Return true for each output port with complex data
            varargout{1} = false;
            if obj.ShowInfo
                varargout{2} = false;
            end
        end

        function varargout = isOutputFixedSizeImpl(obj)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            varargout{1} = true;
            if obj.ShowInfo
                varargout{2} = true;
            end
        end
    end
    
    methods(Access = protected, Static)
        function header = getHeaderImpl
            %getHeaderImpl Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            %getPropertyGroupsImpl Define property section(s) for System block dialog
            mainGroup = matlab.system.display.SectionGroup(...
                'Title','Initial conditions', ...
                'PropertyList',{'TreeStruct', 'EEBodyName','SolverName',...
                'SolverParameters','InfoBusName', 'ShowInfo'});
            
            group = mainGroup;
        end

        function flag = showSimulateUsingImpl
            %showSimulateUsingImpl Return false if simulation mode hidden in System block dialog
            flag = true;
        end
    end
end
