classdef ackermannKinematics < robotics.mobile.internal.InternalAccess
    %ACKERMANNKINEMATICS Create car-like steering vehicle model
    %   ACKERMANNKINEMATICS creates a car-like vehicle model that uses
    %   Ackermann steering. This model represents a vehicle with two axles
    %   separated by the distance, WHEELBASE. The state of the vehicle is
    %   defined as a four-element vector, [X Y THETA PSI], with an XY
    %   global position, vehicle heading, THETA, and steering angle, PSI.
    %   The vehicle origin and heading are defined at the center of the
    %   rear axle. Angles are given in radians. The steering input for the
    %   vehicle is given as PSIDOT, in radians per second. To compute time
    %   derivative states for the model, use the derivative function with
    %   input steering commands and the current robot state.
    %
    %   The Ackermann steering geometry ensures a no-slip condition that 
    %   constrains the two front wheels to concentric circles. This
    %   constraint means the wheels are at different angles for a single
    %   steering angle input. This behavior can be represented by a
    %   bicycle-like model where the input is the rate of steering, rather
    %   than the steering angle, and the kinematics relate the rate of
    %   steering to the rate of heading change.
    %
    %   OBJ = ackermannKinematics creates an Ackermann kinematic model
    %   object with default property values.
    %
    %   OBJ = ackermannKinematics("PropertyName", PropertyValue, ...)
    %   sets properties on the object to the specified value. You can
    %   specify multiple properties in any order.
    %
    %   ACKERMANNKINEMATICS Properties:
    %      WheelBase           - Distance between front and rear axles
    %      VehicleSpeedRange   - Range of vehicle speeds
    %
    %   ACKERMANNKINEMATICS Methods:
    %      derivative          - Time derivative of vehicle state
    %      copy                - Create a copy of the object
    %
    %   Example:
    %      % Define robot and initial conditions
    %      obj = ackermannKinematics;
    %      initialState = [0 0 0 0];
    %
    %      % Set up simulation
    %      tspan = 0:0.1:1;
    %      cmds = [10 1]; % 10m/s velocity and left turn
    %      [t,y] = ode45(@(t,y)derivative(obj, y, cmds), tspan, initialState);
    %
    %      % Plot resultant path
    %      figure
    %      plot(y(:,1),y(:,2))
    %
    %   See also bicycleKinematics, unicycleKinematics, differentialDriveKinematics
    
    %   Copyright 2019-2020 The MathWorks, Inc.
    
    %#codegen
    
    properties
        
        %WheelBase - Distance between front and rear axles
        %   The wheel base refers to the distance between the front and
        %   rear vehicle axles, specified in meters.
        WheelBase
        
        %VehicleSpeedRange - Range of vehicle speeds
        %   The vehicle speed range is a two-element vector that provides
        %   the minimum and maximum vehicle speeds, [MinSpeed MaxSpeed],
        %   specified in m/s.
        VehicleSpeedRange
    end
    
    properties (Constant, Access = protected)
        
        WheelBaseDefault = 1;
        
        VehicleSpeedRangeDefault = [-inf inf];
    end
    
    methods
        function obj = ackermannKinematics(varargin)
            %ACKERMANNKINEMATICS Constructor
            
            % Convert strings to chars
            charInputs = cell(1,nargin);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            
            % Parse inputs
            names = {'WheelBase', 'VehicleSpeedRange'};
            defaults = {obj.WheelBaseDefault, obj.VehicleSpeedRangeDefault};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            obj.WheelBase = parameterValue(parser, names{1});
            obj.VehicleSpeedRange = parameterValue(parser, names{2});     
        end
        
        function stateDot = derivative(obj, state, cmds)
            %DERIVATIVE Time derivative of model states
            %   STATEDOT = derivative(OBJ, STATE, CMDS) returns the current
            %   state derivative, STATEDOT, as a four-element vector [XDOT
            %   YDOT THETADOT PSIDOT]. XDOT and YDOT refer to the vehicle
            %   velocity in meters per second. THETADOT is the angular
            %   velocity of the vehicle heading in rad/s, and PSIDOT is the
            %   angular velocity of the vehicle steering in rad/s.
            %
            %      OBJ     - Ackermann vehicle model object
            %
            %      STATE   - A four-element state vector [X Y THETA PSI], 
            %                where X and Y are the global vehicle position
            %                in meters, THETA is the global vehicle heading
            %                in radians, and PSI is the vehicle steering
            %                angle in radians.
            %
            %      CMDS    - A two-element vector of input commands 
            %                [V PSIDOT], where V is the vehicle speed, and
            %                PSIDOT is the steering angle angular velocity.
            %
            %   Example:
            %      % Define robot and initial conditions
            %      obj = ackermannKinematics;
            %      initialState = [0 0 0 0];
            %
            %      % Set up simulation
            %      tspan = 0:0.1:1;
            %      cmds = [10 1]; % 10 m/s velocity and left turn
            %      [t,y] = ode45(@(t,y)derivative(obj, y, cmds), tspan, initialState);
            %
            %      % Plot resultant path
            %      figure
            %      plot(y(:,1),y(:,2))
            
            narginchk(3,3);
            
            % Validate inputs
            validateattributes(state, {'double', 'single'}, {'nonempty', 'vector', 'numel', 4, 'real', 'nonnan'}, 'derivative', 'state');
            validateattributes(cmds, {'double', 'single'}, {'nonempty', 'vector', 'numel', 2, 'real', 'finite'}, 'derivative', 'cmds');
            
            % Gather inputs and states
            v = cmds(1);
            psiDot = cmds(2);
            theta = state(3);
            psi = state(4);
            
            % Saturate velocity if it exceeds vehicle speed range
            v = max(v, obj.VehicleSpeedRange(1));
            v = min(v, obj.VehicleSpeedRange(2));

            % Compute state derivative
            J = [cos(theta) 0; sin(theta) 0; tan(psi)/obj.WheelBase 0; 0 1];
            stateDot = J*[v; psiDot];
        end
        
        function newVehicle = copy(obj)
            %COPY Copy kinematic model
            %   NEWVEHICLE = COPY(VEHICLE) returns a deep copy of VEHICLE.
            %   NEWVEHICLE and VEHICLE are two different
            %   ackermannKinematics objects with the same properties.
            %   
            %   Example:
            %       % Create an Ackermann kinematic model
            %       a1 = ackermannKinematics('WheelBase', 3)
            %
            %       % Make a copy
            %       a2 = COPY(a1)
            
            newVehicle = ackermannKinematics(...
                'WheelBase', obj.WheelBase, ...
                'VehicleSpeedRange', obj.VehicleSpeedRange ...
                );
        end
    end
    
    %% Get/Set methods
    methods      
        function set.WheelBase(obj, vehicleLength)
            %SET.WHEELBASE Setter method for WheelBase
            
            validateattributes(vehicleLength, {'double', 'single'}, {'nonempty', 'scalar', 'finite', 'positive'}, 'ackermannKinematics', 'WheelBase');
            obj.WheelBase = vehicleLength;
        end
        
        function set.VehicleSpeedRange(obj, speedRange)
            %SET.VEHICLESPEEDRANGE Setter method for VehicleSpeedRange
            
            validateattributes(speedRange, {'double', 'single'}, {'nonempty', 'vector', 'numel', 2, 'nonnan', 'nondecreasing'}, 'ackermannKinematics', 'VehicleSpeedRange');
            obj.VehicleSpeedRange = speedRange(:)';
        end
    end
end
