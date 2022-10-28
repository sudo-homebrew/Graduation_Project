classdef JointSpaceMotionModelBlock < matlab.System
    % This class is for internal use only. It may be removed in the future.
    
    %JOINTSPACEMOTIONMODELBLOCK Joint-Space Motion Model block system object

    % Copyright 2019-2020 The MathWorks, Inc.

    %#codegen
    
    properties (Hidden, SetAccess = protected)
        %Motion Model
        %   This must be initialized to empty since it cannot be
        %   initialized to an object unless it is also a constant property.
        %   However, it may be changed to an object later
        MotionModel = []
    end
    
    properties (Nontunable)
        
        %MotionType
        MotionType
        
        %TreeStruct
        TreeStruct = 0;
    
        
        %Show External Force Inport
        ShowExternalForcePort (1, 1) logical = true

    end
    
    properties
        %Kp - Proportional Gain
        Kp
        
        %Kd - Derivative Gain
        Kd
        
        %DampingRatio
        DampingRatio
        
        %NaturalFrequency
        NaturalFrequency
    end

    properties (Constant, Hidden)
        % String set for MotionType
        MotionTypeSet = matlab.system.StringSet(...
            robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.JointSpaceMotionTypes ...
            )
        
        MotionTypeStrings = robotics.slmanip.internal.block.RBTAlgorithmBlockMasks.JointSpaceMotionTypes
    end
    
    properties (SetAccess = private, Nontunable)
        NumJoints
        
        %TreeInternal
        TreeInternal
    end
    
    %% Simulation methods

    methods(Access = protected)
        function setupImpl(obj)
            %setupImpl Perform one-time calculations
            %   Perform one-time calculations, which primarily includes
            %   setting up handle objects, which may only be instantiated
            %   once. This method also sets the properties on those
            %   objects, though that is also done again later in the
            %   resetImpl methods.
            
            % Assign the robot internal property by instantiating from the structure      
            obj.TreeInternal = robotics.manip.internal.RigidBodyTree(obj.TreeStruct.NumBodies, obj.TreeStruct);
            obj.NumJoints = obj.TreeStruct.VelocityNumber;
            
            % Create the taskSpaceMotionModel object
            obj.MotionModel = jointSpaceMotionModel('RigidBodyTree', obj.TreeInternal, 'MotionType', obj.getMotionTypeString);
            
            % Assign motion model properties
            obj.assignModelProperties;
        end

        function qdd = stepImpl(obj, q, qd, qRef, qRefDot, qRefDDot, fExt)   
            %stepImpl Call RBT object to generate a solution
            
            % Validate inputs
            obj.validateInputs(qRef, qRefDot, qRefDDot, fExt);
            
            % Parse input arguments
            [state, cmds, fExt] = obj.computeConfigurationDependentInputs(q, qd, qRef, qRefDot, qRefDDot, fExt);
                        
            % Compute state derivative
            if strcmp(obj.MotionType, obj.MotionTypeStrings{4}) %Open Loop Dynamics
                computedDerivative = [qd; robotics.manip.internal.RigidBodyTreeDynamics.forwardDynamicsCRB(obj.TreeInternal, q(:), qd(:), zeros([numel(q) 1]), fExt)];
            else
                computedDerivative = obj.MotionModel.derivative(state, cmds, fExt);
            end
            
            % Assign output
            qdd = cast(zeros(obj.NumJoints,1),'like',qRef);
            qdd(:,:) = computedDerivative(obj.NumJoints+1:end,:);
        end
        
        function resetImpl(obj)
            %resetImpl Reset is used for codegen fast restart
            %   The reset method is automatically called after setup and
            %   before every subsequent simulation run
            
            assignModelProperties(obj);
        end

        function s = saveObjectImpl(obj)
            %saveObjectImpl Set properties in structure s to values in object obj
            %   This is used in Simulink for interpreted mode Fast Restart

            % Save only the public properties, as private and internal
            % properties will be reset when the object is loaded from the
            % locked state
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            s.MotionModel = copy(obj.MotionModel);
            s.TreeInternal = copy(obj.TreeInternal);
            s.NumJoints = obj.NumJoints;
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            %loadObjectImpl Load object is used for fast-restart in interpreted mode
            %   In interpreted mode, Fast Restart uses load and save
            %   (loadObjectImpl and saveObjectImpl). After initialization,
            %   the System object is saved. Restarting simulation will
            %   invoke load and then run the System object. In code
            %   generation mode, an entirely different mechanism is used
            %   that doesn't use save/load.
            loadObjectImpl@matlab.System(obj, s, wasLocked);
            
            % The System object is saved from a locked state (setupImpl and
            % resetImpl are already run, so they will not be run again when
            % the simulation restarts).
            if wasLocked
                % Assign the robot internal property by instantiating from the structure      
                obj.TreeInternal = robotics.manip.internal.RigidBodyTree(obj.TreeStruct.NumBodies, obj.TreeStruct);
                obj.NumJoints = obj.TreeStruct.VelocityNumber;

                % Create the jointSpaceMotionModel object and assign properties
                obj.MotionModel = jointSpaceMotionModel('RigidBodyTree', obj.TreeInternal, 'MotionType', obj.getMotionTypeString);
                obj.assignModelProperties;
            end
        end
    end
    
    %% System Object methods
    
    methods (Access = protected)
        function processTunedPropertiesImpl(obj)
            % Perform actions when tunable properties change
            % between calls to the System object
            if isChangedProperty(obj, 'Kp')
                obj.MotionModel.Kp = obj.Kp;
            end
            
            if isChangedProperty(obj, 'Kd')
                obj.MotionModel.Kd = obj.Kd;
            end
            
            if isChangedProperty(obj, 'DampingRatio')
                obj.MotionModel.DampingRatio = obj.DampingRatio;
            end
            
            if isChangedProperty(obj, 'NaturalFrequency')
                obj.MotionModel.NaturalFrequency = obj.NaturalFrequency;
            end
        end

        function flag = isInputDataTypeMutableImpl(~)
            %isInputDataTypeMutableImpl Identify whether input can change type 
            %   Return false if input data type cannot change between calls
            %   to the System object
            
            flag = false;
        end

        function num = getNumInputsImpl(~)
            %getNumInputsImpl Define total number of inputs for system with optional inputs
            %   While the derivative function can take different numbers of
            %   inputs, that is all handled by the mask. To ensure a
            %   minimal number of moving parts, the number of ports in the
            %   system block remains constant, and only some are used
            %   depending on the controller. The assignment of ports to
            %   inputs to the derivative methods is handled by the
            %   "computeConfigurationDependentInputs" method.
            
            num = 6;
        end
        
        function validateInputsImpl(obj, ~, ~, qRef, qRefDot, qRefDDot, fExt)
            %validateInputsImpl Validate inputs to the step method at initialization
            
            obj.validateInputs(qRef, qRefDot, qRefDDot, fExt);
        end

        function num = getNumOutputsImpl(~)
            %getNumOutputsImpl Define total number of outputs
            
            num = 1;
        end

        function out = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            out = [obj.TreeStruct.VelocityNumber 1];
        end

        function out = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            
            out = propagatedInputDataType(obj,3);
        end

        function out = isOutputComplexImpl(~)
            %isOutputComplexImpl Return true for each output port with complex data
            
            out = false;
        end

        function out = isOutputFixedSizeImpl(~)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            
            out = true;
        end
    end

%% Helper methods
    
    methods (Access = private)
        function motionTypeStr = getMotionTypeString(obj)
            %getMotionTypeString Get motionType string input for motion model
            %   The strings provided from the block differ from those used
            %   as inputs to the MATLAB motion model object. This function
            %   maps the Simulink block input to the value assigned to the
            %   object.
            
            % Convert specified motion type to valid string for object
            actMotionTypeStr = obj.MotionType(~isspace(obj.MotionType));
            if strcmp(actMotionTypeStr, 'OpenLoopDynamics')
                motionTypeStr = 'PDControl';
            else
                motionTypeStr = actMotionTypeStr;
            end
        end
        
        function assignModelProperties(obj)
            %assignModelProperties Assign properties to the stored motion model object
            
            obj.MotionModel.Kp = obj.Kp;
            obj.MotionModel.Kd = obj.Kd;
            obj.MotionModel.DampingRatio = obj.DampingRatio;
            obj.MotionModel.NaturalFrequency = obj.NaturalFrequency;
        end
        
        function validateInputs(obj, qRef, qRefDot, qRefDDot, fExt)
            
            % Validate all the user-facing inputs
            if ~strcmp(obj.MotionType, obj.MotionTypeStrings{4})
                % All cases except open loop dynamics
                validateattributes(qRef,{'double'},{'vector', 'nonempty', 'finite', 'real', 'numel', obj.TreeStruct.VelocityNumber},'JointSpaceMotionModelBlock','qRef');
                validateattributes(qRefDot,{'double'},{'vector', 'nonempty', 'finite', 'real', 'numel', obj.TreeStruct.VelocityNumber},'JointSpaceMotionModelBlock','qRefDot');
            end
            if strcmp(obj.MotionType, obj.MotionTypeStrings{1}) || strcmp(obj.MotionType, obj.MotionTypeStrings{3})
                % Only Computed Torque or Independent Joint Motion
                validateattributes(qRefDDot,{'double'},{'vector', 'nonempty', 'finite', 'real', 'numel', obj.TreeStruct.VelocityNumber},'JointSpaceMotionModelBlock','qRefDDot');
            end
            if obj.ShowExternalForcePort
                % Only when port is visible
                validateattributes(fExt,{'double'},{'2d', 'nonempty', 'finite', 'real', 'size', [6 obj.TreeStruct.NumBodies]},'JointSpaceMotionModelBlock','FExt');
            end
        end
        
        function [state, cmds, extForce] = computeConfigurationDependentInputs(obj, qIn, qdIn, qRefIn, qRefDotIn, qRefDDotIn, fExtIn)
            
            % Convert all the inputs to the same type. Use qRefIn because
            % that is the first input that the user has control over (q and
            % qd are passed inside the mask)
            q = cast(qIn, 'like', qRefIn);
            qd = cast(qdIn, 'like', qRefIn);
            qRef = qRefIn;
            qRefDot = cast(qRefDotIn, 'like', qRefIn);
            qRefDDot = cast(qRefDDotIn, 'like', qRefIn);
            fExt = cast(fExtIn, 'like', qRefIn);
            
            
            % Assemble the state, a column vector
            state = [q(:); qd(:)];
            
            % Define whether to compute default external force
            useDefaultExtForce = ~obj.ShowExternalForcePort;
            
            % Assemble the input commands column vector
            switch obj.MotionType
                case obj.MotionTypeStrings{1} %Computed Torque Control
                    cmds = [qRef(:); qRefDot(:); qRefDDot(:)];
                case obj.MotionTypeStrings{2} %PD Control
                    cmds = [qRef(:); qRefDot(:)];
                case obj.MotionTypeStrings{3} %Independent Joint Motion
                    cmds = [qRef(:); qRefDot(:); qRefDDot(:)];
                    useDefaultExtForce = true;
                otherwise                     %Open Loop Dynamics
                    cmds = [];
            end
            
            % Assign external force or create zero-force placeholder
            if useDefaultExtForce
                extForce = zeros(6, obj.TreeInternal.NumBodies);
            else
                extForce = fExt;
            end
        end
    end
end
