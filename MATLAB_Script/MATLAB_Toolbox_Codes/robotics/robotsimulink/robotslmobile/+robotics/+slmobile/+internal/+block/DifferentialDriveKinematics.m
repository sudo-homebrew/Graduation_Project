classdef DifferentialDriveKinematics < matlab.System & ...
        robotics.mobile.internal.RoboticsAccess
    % This class is for internal use only. It may be removed in the future.

    % DIFFERENTIALDRIVEKINEMATICS System object for differential-drive kinematic model block
    
    % Copyright 2019-2021 The MathWorks, Inc.
    
    %#codegen
    
    properties (Hidden, SetAccess = protected)
        %Kinematic Model
        %   This must be initialized to empty since it cannot be
        %   initialized to an object unless it is also a constant property.
        %   However, it may be changed to an object later
        KinModel = []
    end
    
    properties
        %TrackWidth - The vehicle track width in meters
        TrackWidth
        
        %WheelRadius - The wheel radius of the vehicle in meters
        WheelRadius
        
        %WheelSpeedRange - The range of vehicle wheel speeds in rad/s
        WheelSpeedRange
    end
    
    properties (Nontunable)
        %VehicleInputsStr - A string or character vector indicating the type of model inputs
        VehicleInputsStr
    end
    
    properties (Constant, Hidden)
        %VehicleInputsMapping
        VehicleInputsMapping = {'WheelSpeeds', 'VehicleSpeedHeadingRate'}
        
        %VehicleInputs Strings
        VehicleInputsStrOpts = getVehicleInputOptions
        
        % String set for VehicleInputsStr
        %   The string set stores the properties, but for codegen
        %   compatible code, it cannot be referenced in the code.
        %   Therefore, a duplicate property, VehicleInputsStrOpts, is used as
        %   a reference of possible options in the code itself
        VehicleInputsStrSet = matlab.system.StringSet(getVehicleInputOptions)
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
            
            % Create the bicycleKinematics object
            obj.KinModel = differentialDriveKinematics;
            
            % Assign properties
            obj.assignModelProperties;
        end

        function [stateDot] = stepImpl(obj, input1, input2, state)   
            %stepImpl Call RBT object to generate a solution
            
            % Validate input and states
            obj.validateInputs(input1, input2);
            obj.validateState(state);
            
            % Convert inputs to avoid mixing data types, then compute the
            % state derivative
            input2MatchedData = cast(input2,'like',input1);
            stateMatchedData = cast(state,'like',input1);
            computedDerivative = obj.KinModel.derivative(stateMatchedData, [input1; input2MatchedData]);
            
            % Assign output
            stateDot = cast(zeros(3,1),'like',input1);
            stateDot(:,:) = cast(computedDerivative,'like',input1);
        end
        
        function resetImpl(obj)
            %resetImpl Reset is used for codegen fast restart
            %   The reset method is automatically called after setup and
            %   before every subsequent simulation run
            
            assignModelProperties(obj);
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
                obj.KinModel = differentialDriveKinematics;
                assignModelProperties(obj);
            end
        end

        function s = saveObjectImpl(obj)
            %saveObjectImpl Set properties in structure s to values in object obj
            %   This is used in Simulink for interpreted mode Fast Restart

            % Save only the public properties, as private and internal
            % properties will be reset when the object is loaded from the
            % locked state

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
        end
    end
        
    %% Helper methods
    methods (Access = private)
        function assignModelProperties(obj)
        %assignModelProperties Assign properties to the stored motion model object
            
            obj.assignVehicleInputs;
            obj.KinModel.TrackWidth(:) = obj.TrackWidth;
            obj.KinModel.WheelRadius(:) = obj.WheelRadius;
            obj.KinModel.WheelSpeedRange(:) = obj.WheelSpeedRange;
            
        end
    end
    
    methods (Access = protected)
        function assignVehicleInputs(obj)
           switch obj.VehicleInputsStr
               case obj.VehicleInputsStrOpts{1}
                   obj.KinModel.VehicleInputs = obj.VehicleInputsMapping{1};
               case obj.VehicleInputsStrOpts{2}
                   obj.KinModel.VehicleInputs = obj.VehicleInputsMapping{2};
           end
        end
        
        function validateInputs(obj, input1, input2)
            %validateInputs Validate inputs to the block
            %   Since the inputs are dependent on the VehicleInputs
            %   dropdown, the error message here is updated depending on
            %   the value of that parameter.
            
            switch obj.VehicleInputsStr
               case obj.VehicleInputsStrOpts{1}
                   % Case 1: Wheel Speeds
                   input1Str = 'phiDotL';
                   input2Str = 'phiDotR';
                   
               case obj.VehicleInputsStrOpts{2}
                   % Case 2: Vehicle Speed & Heading Angular Velocity
                   input1Str = 'v';
                   input2Str = 'omega';
            end
            
            validateattributes(input1, {'double', 'single'}, {'nonempty', 'scalar', 'real', 'finite'}, 'validateInputsImpl', input1Str);
            validateattributes(input2, {'double', 'single'}, {'nonempty', 'scalar', 'real', 'finite'}, 'validateInputsImpl', input2Str);
        end
    end
    
    methods (Static, Access = protected)
        function validateState(state)
            %validateState Error if the state reaches an unexpected value
            %   If the state becomes infinite or NaN, the integrators below
            %   the user-facing block mask will eventually fail. This check
            %   ensures that such an event is preceded by an error from the
            %   block that states the problem and helps direct the user to
            %   better resolve the issue.
            
            isInvalidState = any(isnan(state)) || any(isinf(state)); 
            coder.internal.errorIf(isInvalidState, 'robotics:robotslmobile:sharedblockmask:NonFiniteStateVector');
        end
    end
    
    %% System Object methods
    
    methods (Access = protected)
        function processTunedPropertiesImpl(obj)
            % Perform actions when tunable properties change
            % between calls to the System object
            if isChangedProperty(obj, 'WheelRadius')
                obj.KinModel.WheelRadius(:) = obj.WheelRadius;
            end
            
            if isChangedProperty(obj, 'TrackWidth')
                obj.KinModel.TrackWidth(:) = obj.TrackWidth;
            end
            
            if isChangedProperty(obj, 'WheelSpeedRange')
                obj.KinModel.WheelSpeedRange(:) = obj.WheelSpeedRange;
            end
        end

        function validateInputsImpl(obj, input1, input2, state) %#ok<INUSD>
            %validateInputsImpl Validate inputs to the step method at initialization
            %   These errors are thrown at initialization. In the unlikely
            %   event that the input attributes change during run-time,
            %   those errors will be thrown directly from the step method.
            %   The "state" input is not validated, as this is only passed
            %   internally, within the masked user-facing block.
            
            obj.validateInputs(input1, input2);
        end
        
        function num = getNumInputsImpl(obj) %#ok<MANU>
            % Define total number of inputs for system with optional inputs
            num = 3;
        end

        function num = getNumOutputsImpl(obj) %#ok<MANU>
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
        end

        function [out1] = getOutputSizeImpl(obj) %#ok<MANU>
            % Return size for each output port
            out1 = [3 1];
        end

        function [out] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = propagatedInputDataType(obj,1);
        end

        function [out] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            out = false;
        end

        function [out] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            out = true;
        end

        function flag = supportsMultipleInstanceImpl(~)
            %supportsMultipleInstanceImpl Return true to enable support for
            % For-Each Subsystem
            flag = true;
        end

    end
end

function strOptions = getVehicleInputOptions
    %getVehicleInputOptions Function to initialize constant list of string options for Vehicle Inputs popup
    %   This method is used to update two constant properties that both
    %   reference this information. It is defined as a function outside the
    %   class so that the options are defined in a single location. They
    %   are then referenced by the StringSet, which updates the display,
    %   and by the VehicleInputsStrOpts property, which provides the
    %   options for use in the downstream code.
    
    strOptions = {...
            message("robotics:robotslmobile:differentialdrive:InputSpeedsPopup").getString, ...
            message("robotics:robotslmobile:sharedblockmask:InputSpeedAngVelPopup").getString
            };
end
