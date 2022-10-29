classdef differentialDriveKinematics < robotics.mobile.internal.InternalAccess
    %DIFFERENTIALDRIVEKINEMATICS Create differential-drive vehicle model
    %   DIFFERENTIALDRIVEKINEMATICS creates a differential-drive vehicle
    %   model to simulate simplified vehicle dynamics. This model
    %   approximates a vehicle with a single fixed axle and wheels
    %   separated by a specified track width. The wheels can be driven
    %   independently. Vehicle speed and heading is defined from the axle
    %   center.
    %
    %   OBJ = differentialDriveKinematics creates a differential drive
    %   kinematic model object with default property values.
    %
    %   OBJ = differentialDriveKinematics("PropertyName", PropertyValue, ...)
    %   sets properties on the object to the specified value. You can
    %   specify multiple properties in any order.
    %
    %   DIFFERENTIALDRIVEKINEMATICS Properties:
    %      WheelRadius       - Wheel radius of vehicle
    %      WheelSpeedRange   - Range of vehicle wheel speeds
    %      TrackWidth        - Distance between wheels on axle
    %      VehicleInputs     - Type of command inputs with options:
    %          - "WheelSpeeds"               - (Default) Wheel speed and heading angular velocity
    %          - "VehicleSpeedHeadingRate"   - Vehicle speed and heading angular velocity
    %
    %   DIFFERENTIALDRIVEKINEMATICS Methods:
    %      derivative        - Time derivative of vehicle state
    %      copy              - Create a copy of the object 
    %
    %   Example:
    %      % Define robot and initial conditions
    %      obj = differentialDriveKinematics;
    %      initialState = [0 0 0];
    %
    %      % Set up simulation
    %      tspan = 0:0.1:1;
    %      inputs = [50 40]; %Left wheel is spinning faster
    %      [t,y] = ode45(@(t,y)derivative(obj, y, inputs), tspan, initialState);
    %
    %      % Plot resultant path
    %      figure
    %      plot(y(:,1),y(:,2))
    %   See also ackermannKinematics, bicycleKinematics, unicycleKinematics
    
    %   Copyright 2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Dependent)        
        %VehicleInputs - Type of model inputs
        %   The VehicleInputs property specifies the format of the model
        %   input commands when using the derivative function. Options are
        %   specified one of the following strings:
        %      "WheelSpeeds"               - Angular speeds of the two wheels (Default)
        %      "VehicleSpeedHeadingRate"   - Vehicle speed and heading angular velocity
        VehicleInputs
    end
    
    properties                
        %TrackWidth - Distance between wheels on axle
        %   The vehicle track width refers to the length between the
        %   wheels, or the axle length, specified in meters.
        TrackWidth
        
        %WheelRadius - Wheel radius of vehicle
        %   The wheel radius of the vehicle, specified in meters.
        WheelRadius
        
        %WheelSpeedRange - Range of vehicle wheel speeds
        %   The wheel speed range is a two-element vector that provides the
        %   minimum and maximum vehicle wheel speeds, [MinSpeed MaxSpeed],
        %   specified in rad/s.
        WheelSpeedRange
    end    
    
    properties (Constant, Access = protected)
        %VehicleInputsOptions - The set of user-facing options for the VehicleInputs Name-Value pair
        VehicleInputsOptions = {'WheelSpeeds', 'VehicleSpeedHeadingRate'};
        
        %VehicleInputsInternalOptions -  The set of internal options for the internal implementation of the VehicleInputs property, VehicleInputsInternal
        %   To ensure that these functions can be called in Simulink
        %   without relying on dynamic memory allocation, this class uses
        %   strings of equivalent length to represent the vehicle options
        %   internally. Therefore this set of options fills the values of
        %   the strings with less than the maximum number of characters
        %   with the equivalent number of hyphens
        VehicleInputsInternalOptions = {'WheelSpeeds------------', 'VehicleSpeedHeadingRate'};
        
        TrackWidthDefault = 0.2;
        
        VehicleInputsDefault = 'WheelSpeeds';
        
        WheelRadiusDefault = 0.05;
        
        WheelSpeedRangeDefault = [-inf inf];
    end
    
    properties (Hidden)
        %VehicleInputsInternal - Fixed-length strings for codegen compatibility
        %   To ensure that these functions can be called in Simulink
        %   without relying on dynamic memory allocation, this class uses
        %   strings of equivalent length to represent the vehicle options
        %   internally. These are applied in the VehicleInputs Set method.
        VehicleInputsInternal
    end

    methods
        function obj = differentialDriveKinematics(varargin)
            %DIFFERENTIALDRIVEKINEMATICS Constructor
            
            % Convert strings to chars
            charInputs = cell(1,nargin);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            
            % Parse inputs
            names = {'WheelRadius', 'TrackWidth', 'WheelSpeedRange', 'VehicleInputs'};
            defaults = {obj.WheelRadiusDefault, obj.TrackWidthDefault, obj.WheelSpeedRangeDefault, obj.VehicleInputsDefault};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            obj.WheelRadius = parameterValue(parser, names{1});
            obj.TrackWidth = parameterValue(parser, names{2});
            obj.WheelSpeedRange = parameterValue(parser, names{3});
            obj.VehicleInputs = parameterValue(parser, names{4});            
        end
        
        function stateDot = derivative(obj, state, cmds)
            %DERIVATIVE Time derivative of  model states
            %   STATEDOT = derivative(OBJ, STATE, CMDS) returns the current
            %   state derivative, STATEDOT, as a three-element vector [XDOT
            %   YDOT THETADOT]. XDOT and YDOT refer to the vehicle velocity
            %   in meters per second. THETADOT is the angular velocity of
            %   the vehicle heading in rad/s.
            %
            %      OBJ     - Differential drive vehicle model
            %
            %      STATE   - A three-element state vector [X Y THETA],
            %                where X and Y are the global vehicle position
            %                in meters, and THETA is the global vehicle
            %                heading in radians.
            %
            %      CMDS    - A two-element vector of input commands. The
            %                format is dependent on the value of the
            %                VEHICLEINPUTS property of OBJ:
            %                   - "WheelSpeeds": The input is the two
            %                     element vector, [PHIDOTL PHIDOTR], where
            %                     PHIDOTL and PHIDOTR refer the left and
            %                     right wheel angular velocities,
            %                     respectively, in rad/s.
            %                   - "VehicleSpeedHeadingRate": The input is 
            %                     the two-element vector [V THETADOT] where
            %                     V is the vehicle speed and THETADOT is
            %                     the angular velocity of the vehicle
            %                     heading, THETA, in rad/s.
            %
            %   Example:
            %      % Define robot and initial conditions
            %      obj = differentialDriveKinematics;
            %      initialState = [0 0 0];
            %
            %      % Set up simulation
            %      tspan = 0:0.1:1;
            %      inputs = [50 40]; % Slight right turn
            %      [t,y] = ode45(@(t,y)derivative(obj, y, inputs), tspan, initialState);
            %
            %      % Plot resultant path
            %      figure
            %      plot(y(:,1),y(:,2))
            
            narginchk(3,3);

            % Validate inputs
            validateattributes(state, {'double', 'single'}, {'nonempty', 'vector', 'numel', 3, 'real', 'nonnan'}, 'derivative', 'state');
            validateattributes(cmds, {'double', 'single'}, {'nonempty', 'vector', 'numel', 2, 'real', 'finite'}, 'derivative', 'cmds');
                        
            % Process inputs
            [v, omega] = processInputs(obj, cmds);
            
            % Compute state derivative
            theta = state(3);
            stateDot = [cos(theta) 0; sin(theta) 0; 0 1]*[v; omega];
        end
        
        function newVehicle = copy(obj)
            %COPY Copy kinematic model
            %   NEWVEHICLE = COPY(VEHICLE) returns a deep copy of VEHICLE.
            %   NEWVEHICLE and VEHICLE are two different
            %   differentialDriveKinematics objects with the same
            %   properties.
            %   
            %   Example:
            %       % Create an differential-drive kinematic model
            %       d1 = differentialDriveKinematics('WheelRadius', 3)
            %
            %       % Make a copy
            %       d2 = COPY(d1)
            
            newVehicle = differentialDriveKinematics(...
                'VehicleInputs', obj.VehicleInputs, ...
                'TrackWidth', obj.TrackWidth, ...
                'WheelRadius', obj.WheelRadius, ...
                'WheelSpeedRange', obj.WheelSpeedRange ...
                );
        end
    end
    
    %% Get/Set Methods
    methods        
        function set.WheelRadius(obj, radius)
            %SET.WHEELRADIUS Setter method for WheelRadius
            
            validateattributes(radius, {'double', 'single'}, {'nonempty', 'scalar', 'finite', 'positive'}, 'differentialDriveKinematics', 'WheelRadius');
            obj.WheelRadius = radius;
        end
        
        function set.WheelSpeedRange(obj, speedRange)
            %SET.WHEELSPEEDRANGE Setter method for WheelSpeedRange
            
            validateattributes(speedRange, {'double', 'single'}, {'nonempty', 'vector', 'numel', 2, 'nonnan', 'nondecreasing'}, 'differentialDriveKinematics', 'WheelSpeedRange');
            obj.WheelSpeedRange = speedRange(:)';
        end
        
        function set.TrackWidth(obj, distance)
            %SET.TRACKWIDTH Setter method for MaxHeadingAngle
            
            validateattributes(distance, {'double', 'single'}, {'nonempty', 'scalar', 'finite', 'positive'}, 'differentialDriveKinematics', 'TrackWidth');
            obj.TrackWidth = distance;
        end
        
        function set.VehicleInputs(obj, inputString)
            %SET.VEHICLEINPUTS Setter method for VehicleInputs
            
            vehicleInputs = validatestring(inputString, obj.VehicleInputsOptions, 'differentialDriveKinematics', 'VehicleInputs'); 
            
            % Inputs have to be the same length for code generation
            if strcmp(vehicleInputs, obj.VehicleInputsOptions{1})
                obj.VehicleInputsInternal = obj.VehicleInputsInternalOptions{1};
            else
                obj.VehicleInputsInternal = obj.VehicleInputsInternalOptions{2};
            end
        end
                
        function strValue = get.VehicleInputs(obj)
            %GET.VEHICLEINPUTS Getter method for VehicleInputs
            
            % Map the string back from the internal version
            if strcmp(obj.VehicleInputsInternal, obj.VehicleInputsInternalOptions{1})
                strValue = obj.VehicleInputsOptions{1};
            else
                strValue = obj.VehicleInputsOptions{2};
            end
        end
    end
        
    %% Helper methods
    methods (Access = protected)        
        function [v, omega] = processInputs(obj, inputs)
            %processInputs Convert inputs to generalized form and acknowledge constraints
            %   This function takes in the inputs and produces v and omega,
            %   the generalized vehicle inputs. During the conversion, it
            %   checks that all input constraints are satisfied, and
            %   saturates them if that is not the case.
            
            % Initialize outputs to ensure codegen compatibility
            v = cast(0, 'like', inputs);
            omega = cast(0, 'like', inputs);
            
            % Saturate inputs and convert to generalized format
            switch obj.VehicleInputsInternal
                case 'WheelSpeeds------------'
                    [v(:), omega(:)] = processModelSpecificInputs(obj, inputs);
                case 'VehicleSpeedHeadingRate'
                    [v(:), omega(:)] = processGeneralizedVehicleInputs(obj, inputs);
            end
        end
        
        function [v, omega] = processModelSpecificInputs(obj, msInputs)
            %processModelSpecificInputs Check constraints and convert to generalized form
            %   This function accepts the model specific inputs, wL and wR
            %   for the differential drive model. It verifies that the
            %   inputs meet constraints and converts them to the
            %   generalized vehicle inputs, v and omega.
            
            % Saturate wheel speeds
            wL = min(max(msInputs(1), obj.WheelSpeedRange(1)), obj.WheelSpeedRange(2));
            wR = min(max(msInputs(2), obj.WheelSpeedRange(1)), obj.WheelSpeedRange(2));
            
            % Convert to generalized form
            d = obj.TrackWidth/2;
            r = obj.WheelRadius;
            v = r*(wR + wL)/2;
            omega = r*(wR - wL)/(2*d);
        end
        
        function [v, omega] = processGeneralizedVehicleInputs(obj, gInputs)
            %processModelSpecificInputs Check constraints and output
            %   This function accepts the generalized vehicle inputs, v and
            %   omega. It verifies that the inputs meet constraints (via
            %   algebraic relationships to the model specific inputs), and
            %   then outputs the processed values.
            
            % It is easiest to convert and check the converted values
            d = obj.TrackWidth/2;
            r = obj.WheelRadius;
            vIn = gInputs(1);
            omegaIn = gInputs(2);
            wL = (vIn - omegaIn*d)/r;
            wR = (vIn + omegaIn*d)/r;
            
            [v, omega] = processModelSpecificInputs(obj, [wL; wR]);
        end
    end
end
