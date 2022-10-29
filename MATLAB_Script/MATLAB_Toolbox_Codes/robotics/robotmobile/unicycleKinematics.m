classdef unicycleKinematics < robotics.mobile.internal.InternalAccess
    %UNICYCLE Create unicycle vehicle model
    %   UNICYCLEKINEMATICS creates a unicycle vehicle model to simulate
    %   simplified car-like vehicle dynamics. This model approximates a
    %   vehicle as a unicycle with a given wheel radius, WHEELRADIUS, that
    %   can spin in place according to a heading angle, THETA.
    %
    %   OBJ = unicycleKinematics creates a unicycle kinematic model object
    %   with default property values.
    %
    %   OBJ = unicycleKinematics("PropertyName", PropertyValue, ...)
    %   sets properties on the object to the specified value. 
    %   You can specify multiple properties in any order.
    %
    %   UNICYCLEKINEMATICS Properties:
    %      WheelRadius       - Wheel radius of vehicle
    %      WheelSpeedRange   - Range of vehicle wheel speeds
    %      VehicleInputs     - Type of command inputs with options:
    %         "WheelSpeedHeadingRate"   - (Default) Wheel speed and heading angular velocity
    %         "VehicleSpeedHeadingRate" - Vehicle speed and heading angular velocity
    %
    %   UNICYCLEKINEMATICS Methods:
    %      derivative        - Time derivative of the vehicle state
    %      copy              - Create a copy of the object 
    %
    %   Example:
    %      % Define robot and initial conditions
    %      obj = unicycleKinematics;
    %      initialState = [0 0 0];
    %
    %      % Set up simulation
    %      tspan = 0:0.1:1;
    %      inputs = [10 1]; %Constant speed and turning left
    %      [t,y] = ode45(@(t,y)derivative(obj, y, inputs), tspan, initialState);
    %
    %      % Plot resultant path
    %      figure
    %      plot(y(:,1),y(:,2))
    %
    %   See also bicycleKinematics, ackermannKinematics, differentialDriveKinematics
    
    %   Copyright 2019 The MathWorks, Inc.
    
    %#codegen
    
    
    properties (Dependent)        
        %VehicleInputs - Type of model inputs
        %   The VehicleInputs property specifies the format of the model
        %   input commands when using the derivative function. Options are
        %   specified one of the following strings:
        %      "WheelSpeedHeadingRate"   - Wheel speed and heading angular velocity (Default)
        %      "VehicleSpeedHeadingRate"   - Vehicle speed and heading angular velocity
        VehicleInputs
    end
    
    properties        
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
        VehicleInputsOptions = {'WheelSpeedHeadingRate', 'VehicleSpeedHeadingRate'};
        
        %VehicleInputsInternalOptions -  The set of internal options for the internal implementation of the VehicleInputs property, VehicleInputsInternal
        %   To ensure that these functions can be called in Simulink
        %   without relying on dynamic memory allocation, this class uses
        %   strings of equivalent length to represent the vehicle options
        %   internally. Therefore this set of options fills the values of
        %   the strings with less than the maximum number of characters
        %   with the equivalent number of hyphens
        VehicleInputsInternalOptions = {'WheelSpeedHeadingRate--', 'VehicleSpeedHeadingRate'};
        
        VehicleInputsDefault = 'WheelSpeedHeadingRate';
        
        WheelRadiusDefault = 0.1;
        
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
        function obj = unicycleKinematics(varargin)
            %UNICYCLEKINEMATICS Constructor
            
            % Convert strings to chars
            charInputs = cell(1,nargin);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            
            % Parse inputs
            names = {'WheelRadius', 'WheelSpeedRange', 'VehicleInputs'};
            defaults = {obj.WheelRadiusDefault, obj.WheelSpeedRangeDefault, obj.VehicleInputsDefault};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            obj.WheelRadius = parameterValue(parser, names{1});
            obj.WheelSpeedRange = parameterValue(parser, names{2});
            obj.VehicleInputs = parameterValue(parser, names{3});            
        end
        
        function stateDot = derivative(obj, state, cmds)
            %DERIVATIVE Time derivative of  model states
            %   STATEDOT = derivative(OBJ, STATE, CMDS) returns the current
            %   state derivative, STATEDOT, as a three-element vector [XDOT
            %   YDOT THETADOT]. XDOT and YDOT refer to the vehicle velocity
            %   in meters per second. THETADOT is the rate of change of
            %   vehicle heading in rad/s.
            %
            %      OBJ     - Unicycle vehicle model
            %
            %      STATE   - A three-element state vector [X Y THETA], 
            %                where X and Y are the global vehicle position
            %                in meters, and THETA is the global vehicle
            %                heading in radians.
            %
            %      CMDS    - A two-element vector of input commands. The
            %                format is dependent on the value of the
            %                VEHICLEINPUTS property of OBJ:
            %                   - "WheelSpeedHeadingRate": The input is the
            %                     two element vector, [PHIDOT OMEGA],
            %                     where PHIDOT is the wheel speed and
            %                     THETADOT is the angular velocity of the
            %                     vehicle heading. Both are specified in
            %                     rad/s.
            %                
            %                   - "VehicleSpeedHeadingRate": The input is 
            %                     the two-element vector, [V THETADOT],
            %                     where V is the vehicle speed in m/s and
            %                     THETADOT is the angualr velocity of the
            %                     vehicle heading in rad/s.
            %
            %   Example:
            %      % Define robot and initial conditions
            %      obj = unicycleKinematics;
            %      initialState = [0 0 0];
            %
            %      % Set up simulation
            %      tspan = 0:0.1:1;
            %      inputs = [10 1]; %Constant speed and turning left
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
            %   NEWVEHICLE and VEHICLE are two different unicycleKinematics
            %   objects with the same properties.
            %   
            %   Example:
            %       % Create a unicycle kinematic model
            %       u1 = unicycleKinematics('WheelRadius', 3)
            %
            %       % Make a copy
            %       u2 = COPY(u1)
            
            newVehicle = unicycleKinematics(...
                'VehicleInputs', obj.VehicleInputs, ...
                'WheelRadius', obj.WheelRadius, ...
                'WheelSpeedRange', obj.WheelSpeedRange ...
                );
        end
    end
    
    %% Get/Set methods
    methods        
        function set.WheelRadius(obj, radius)
            %SET.WHEELRADIUS Setter method for WheelRadius
            
            validateattributes(radius, {'double', 'single'}, {'nonempty', 'scalar', 'finite', 'positive'}, 'unicycleKinematics', 'WheelRadius');
            obj.WheelRadius = radius;
        end
        
        function set.WheelSpeedRange(obj, speedRange)
            %SET.WHEELSPEEDRANGE Setter method for WheelSpeedRange
            
            validateattributes(speedRange, {'double', 'single'}, {'nonempty', 'vector', 'numel', 2, 'nonnan', 'nondecreasing'}, 'unicycleKinematics', 'WheelSpeedRange');
            obj.WheelSpeedRange = speedRange(:)';
        end
        
        function set.VehicleInputs(obj, inputString)
            %SET.VEHICLEINPUTS Setter method for VehicleInputs
            
            vehicleInputs = validatestring(inputString, obj.VehicleInputsOptions, 'unicycleKinematics', 'VehicleInputs'); 
            
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
                case 'WheelSpeedHeadingRate--'
                    % Saturate inputs
                    phiDot = max(inputs(1), obj.WheelSpeedRange(1));
                    phiDot = min(phiDot, obj.WheelSpeedRange(2));
                    thetaDot = inputs(2);
                    [v(:), omega(:)] = getGeneralizedInputsFromModelInputs(obj, phiDot, thetaDot);
                case 'VehicleSpeedHeadingRate'
                    [phiDot, thetaDot] = getWheelControlFromVehicleControl(obj, inputs(1), inputs(2));
                    [v(:), omega(:)] = getGeneralizedInputsFromModelInputs(obj, phiDot, thetaDot);
            end
        end
        
        function [v, omega] = getGeneralizedInputsFromModelInputs(obj, phiDot, thetaDot)
            %getGeneralizedInputsFromModelInputs Convert inputs to from model to generalized form
            %   Convert the model specific inputs, phiDot and thetaDot for
            %   the unicycle, to the generalized vehicle inputs, v and
            %   omega.
            
            v = phiDot*obj.WheelRadius;
            omega = thetaDot;
        end
        
        function [phiDot, thetaDot] = getWheelControlFromVehicleControl(obj, v, w)
            %getWheelControlFromVehicleControl Convert inputs to from generalized to model form
            %   Convert the generalized vehicle inputs, v and omega, to the
            %   model specific inputs, which are phiDot and thetaDot for
            %   the unicycle.
            
            phiDot = v/obj.WheelRadius;
            phiDot = max(phiDot, obj.WheelSpeedRange(1));
            phiDot = min(phiDot, obj.WheelSpeedRange(2));
            thetaDot = w; %omega = dTheta/dt
        end
    end
end
