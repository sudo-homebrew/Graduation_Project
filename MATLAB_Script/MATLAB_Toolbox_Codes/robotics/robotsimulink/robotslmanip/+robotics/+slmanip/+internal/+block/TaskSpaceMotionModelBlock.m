classdef TaskSpaceMotionModelBlock < matlab.System
    % This class is for internal use only. It may be removed in the future.
    
    %TASKSPACEMOTIONMODELBLOCK Task-Space Motion Model block system object

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
        %TreeStruct - A struct created from the RBT
        TreeStruct = 0;
        
        %EEBodyName - The name of the end effector body
        EEBodyName = 0;
    end
    
    properties
        %Proportional Gain
        Kp;
        
        %Derivative Gain
        Kd;
        
        %Joint damping
        JointDamping;
    end
    
    properties (Nontunable)
        
        %ShowExternalForcePort - Show External Force Inport
        ShowExternalForcePort (1, 1) logical = true

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
            obj.MotionModel = taskSpaceMotionModel('RigidBodyTree', obj.TreeInternal, 'EndEffectorName', obj.EEBodyName);
            
            % Assign motion model properties
            obj.assignModelProperties;
        end

        function qdd = stepImpl(obj, q, qd, refPose, refVel, fExt)   
            %stepImpl Call RBT object to generate a solution
            
            % Parse the optional arguments
            fExt = obj.computeConfigurationDependentInputs(fExt);
            
            % Validate inputs
            obj.validateInputs(refPose, refVel, fExt);
            
            % Compute state derivative
            qState = cast(q, 'like', refPose);
            qdState = cast(qd, 'like', refPose);
            state = [qState(:); qdState(:)];
            computedDerivative = obj.MotionModel.derivative(state, refPose, ...
                cast(refVel, 'like', refPose), cast(fExt, 'like', refPose));
            
            % Assign output
            qdd = cast(zeros(obj.NumJoints,1),'like',refPose);
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

                % Create the taskSpaceMotionModel object and assign properties
                obj.MotionModel = taskSpaceMotionModel('RigidBodyTree', obj.TreeInternal, 'EndEffectorName', obj.EEBodyName);   
                assignModelProperties(obj);
            end
        end
    end
    
    %% System object methods
    
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
        end
        
        function num = getNumInputsImpl(~)
            %getNumInputsImpl Get number of inputs
            %   Define total number of inputs for system with optional
            %   inputs
            num = 5;
        end

        function num = getNumOutputsImpl(~)
            %getNumOutputsImpl Get number of outputs
            %   Define total number of outputs for system with optional
            %   outputs
            num = 1;
        end

        function out = getOutputSizeImpl(obj)
            %getOutputSizeImpl Get output size
            %   Return size for each output port. Note that any system
            %   object size greater than one will be output as a vector
            %   (unlike Simulink blocks that differentiate between
            %   N-element signals and [Nx1] or [1xN] signals). This can be
            %   mitigated outside the block via the use of a Reshape block.
            
            out = [obj.TreeStruct.VelocityNumber 1];
        end
        
        function validateInputsImpl(obj, ~, ~, refPose, refVel, fExt)
            %validateInputsImpl Validate inputs to the step method at initialization
            %   No values are passed at initialization, so only the
            %   propagated data type and size will be caught by these
            %   checks.
            
            obj.validateInputs(refPose, refVel, fExt);
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
        function assignModelProperties(obj)
            %assignModelProperties Assign properties to the stored motion model object
            
            obj.MotionModel.Kp = obj.Kp;
            obj.MotionModel.Kd = obj.Kd;
            obj.MotionModel.JointDamping = obj.JointDamping;
        end
        
        function fExt = computeConfigurationDependentInputs(obj, fExtInportValue)
            
            if obj.ShowExternalForcePort
                fExt = fExtInportValue;
            else
                fExt = zeros(6, obj.TreeInternal.NumBodies);
            end
        end
        
        function validateInputs(obj, refPose, refVel, fExt)
            
            % Validate all the user-facing inputs
            validateattributes(refPose,{'double'},{'2d', 'nonempty', 'finite', 'real', 'size', [4 4]},'TaskSpaceMotionModelBlock','refPose');
            validateattributes(refVel,{'double'},{'vector', 'nonempty', 'finite', 'real', 'numel', 6},'TaskSpaceMotionModelBlock','refVel');
            if obj.ShowExternalForcePort
                validateattributes(fExt,{'double'},{'2d', 'nonempty', 'finite', 'real', 'size', [6 obj.TreeStruct.NumBodies]},'TaskSpaceMotionModelBlock','FExt');
            end
        end
    end
end
