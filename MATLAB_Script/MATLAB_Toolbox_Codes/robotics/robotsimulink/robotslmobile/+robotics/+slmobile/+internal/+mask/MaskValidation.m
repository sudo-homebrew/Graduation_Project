classdef MaskValidation
    %This class is for internal use only and may be removed in a future release
    
    %MASKVALIDATION Methods for validating the block mask values
	
	% Copyright 2019 The MathWorks, Inc.
    
    methods (Static)
        function validateUnicycleParameters(wheelRadius, wheelSpeedRange, initialState)
            %VALIDATEUNICYCLEPARAMETERS Validate unicycle edit-field parameters
            %   This method is called from the block mask and occurs at
            %   set-time. If the mask is opened, the Apply or OK buttons
            %   must be clicked. If done via set_param, the callback will
            %   be triggered immediately. These blocks often call internal
            %   MATLAB methods that perform similar checks, but those
            %   checks will not occur until runtime.
            
            validateattributes(wheelRadius, {'single', 'double'}, {'nonempty', 'scalar', 'finite', 'real', 'positive'}, 'validateUnicycleParameters', 'wheelRadius');
            validateattributes(wheelSpeedRange, {'single', 'double'}, {'nonempty', 'vector', 'numel', 2, 'nonnan', 'real', 'nondecreasing'}, 'validateUnicycleParameters', 'wheelSpeedRange');
            
            robotics.slmobile.internal.mask.MaskValidation.validateInitialState(initialState, 3, 'validateUnicycleParameters');
        end
        
        function validateBicycleParameters(wheelBase, vehicleSpeedRange, maxSteeringAngle, initialState)
            %VALIDATEBICYCLEPARAMETERS Validate bicycle edit-field parameters
            %   This method is called from the block mask and occurs at
            %   set-time. If the mask is opened, the Apply or OK buttons
            %   must be clicked. If done via set_param, the callback will
            %   be triggered immediately. These blocks often call internal
            %   MATLAB methods that perform similar checks, but those
            %   checks will not occur until runtime.
            
            validateattributes(wheelBase, {'single', 'double'}, {'nonempty', 'scalar', 'finite', 'real', 'positive'}, 'validateBicycleParameters', 'wheelBase');
            validateattributes(vehicleSpeedRange, {'single', 'double'}, {'nonempty', 'vector', 'numel', 2, 'nonnan', 'real', 'nondecreasing'}, 'validateBicycleParameters', 'vehicleSpeedRange');
            validateattributes(maxSteeringAngle, {'single', 'double'}, {'nonempty', 'scalar', 'finite', 'real', 'nonnegative'}, 'validateBicycleParameters', 'maxSteeringAngle');
            
            robotics.slmobile.internal.mask.MaskValidation.validateInitialState(initialState, 3, 'validateBicycleParameters');
        end
        
        function validateAckermannParameters(wheelBase, vehicleSpeedRange, maxSteeringAngle, initialState)
            %VALIDATEACKERMANNPARAMETERS Validate Ackermann edit-field parameters
            %   This method is called from the block mask and occurs at
            %   set-time. If the mask is opened, the Apply or OK buttons
            %   must be clicked. If done via set_param, the callback will
            %   be triggered immediately. These blocks often call internal
            %   MATLAB methods that perform similar checks, but those
            %   checks will not occur until runtime.
            
            validateattributes(wheelBase, {'single', 'double'}, {'nonempty', 'scalar', 'finite', 'real', 'positive'}, 'validateAckermannParameters', 'wheelBase');
            validateattributes(vehicleSpeedRange, {'single', 'double'}, {'nonempty', 'vector', 'numel', 2, 'nonnan', 'real', 'nondecreasing'}, 'validateAckermannParameters', 'vehicleSpeedRange');
            validateattributes(maxSteeringAngle, {'single', 'double'}, {'nonempty', 'scalar', 'finite', 'real', 'nonnegative'}, 'validateAckermannParameters', 'maxSteeringAngle');
            
            robotics.slmobile.internal.mask.MaskValidation.validateInitialState(initialState, 4, 'validateAckermannParameters');
            
            % Initial state steering angle cannot exceed the maximum imposed by the max
            % steering angle
            expandedInitialState = initialState(:).*[1 1 1 1]';
            if abs(expandedInitialState(4)) > maxSteeringAngle
                error(message('robotics:robotslmobile:ackermann:MaxSteeringAngleViolation'));
            end 
        end
        
        function validateDifferentialDriveParameters(wheelRadius, wheelSpeedRange, trackWidth, initialState)
            %VALIDATEDIFFERENTIALDRIVEPARAMETERS Validate differential-drive vehicle edit-field parameters
            %   This method is called from the block mask and occurs at
            %   set-time. If the mask is opened, the Apply or OK buttons
            %   must be clicked. If done via set_param, the callback will
            %   be triggered immediately. These blocks often call internal
            %   MATLAB methods that perform similar checks, but those
            %   checks will not occur until runtime.
            
            validateattributes(wheelRadius, {'single', 'double'}, {'nonempty', 'scalar', 'finite', 'real', 'positive'}, 'validateDifferentialDriveParameters', 'wheelRadius');
            validateattributes(wheelSpeedRange, {'single', 'double'}, {'nonempty', 'vector', 'numel', 2, 'nonnan', 'real', 'nondecreasing'}, 'validateDifferentialDriveParameters', 'wheelSpeedRange');
            validateattributes(trackWidth, {'single', 'double'}, {'nonempty', 'scalar', 'finite', 'real', 'positive'}, 'validateDifferentialDriveParameters', 'trackWidth');
            
            robotics.slmobile.internal.mask.MaskValidation.validateInitialState(initialState, 3, 'validateDifferentialDriveParameters');
        end        
    end
    
    %% Helper methods
    
    methods (Static, Access = protected)
        function validateInitialState(initialState, expNumel, callerFcnStr)
            %VALIDATEINITIALSTATE Validate initial state for mobile robot kinematic models
            %   The initial state is passed to the integrator, which can
            %   handle implicit expansion, since the dimensions are
            %   otherwise well defined by the system object. Therefore,
            %   this initial value can be both a scalar or a vector with
            %   EXPNUMEL elements.
            
            validateattributes(initialState, {'single', 'double'}, {'nonempty', 'vector', 'finite', 'real'}, callerFcnStr, 'initialState');
            if numel(initialState) ~= 1 && numel(initialState) ~= expNumel
                error(message('robotics:robotslmobile:sharedblockmask:InitialStateSize', expNumel));
            end            
        end            
    end
end

