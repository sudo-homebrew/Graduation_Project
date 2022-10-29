classdef bicycle < nav.algs.internal.getSetters
%This class is for internal use only. It may be removed in the future.

%bicycle Helper class representing bicycle model with steer-angle control inputs

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        SettableParams = {'WheelBase','SpeedLimit','SteerLimit'};
    end
    
    properties (Dependent)
        WheelBase
        SpeedLimit
        SteerLimit
    end
    
    properties (SetAccess = ?nav.algs.internal.InternalAccess)
        ControlLimits = [-1 1; -pi/4 pi/4];
        LimitSpan = [2 pi/2];
        StateSpace
        BaseModel
    end
    
    methods
        function obj = bicycle(ss)
            narginchk(0,1);
            if nargin < 1
                ss = stateSpaceSE2;
            end
            % Initialize state-space and underlying kinematic model
            obj.StateSpace = ss;
            obj.BaseModel = bicycleKinematics('VehicleSpeedRange',[-1 1]);
            
            % Cache control limits
            obj.ControlLimits = [obj.SpeedLimit; obj.SteerLimit];
            obj.LimitSpan = diff(obj.ControlLimits');
        end
        
        function set.SpeedLimit(obj, limits)
            validateattributes(limits,{'numeric'},{'nondecreasing','numel',2,'finite'},'mobileRobotPropagator','SpeedLimit');
            obj.BaseModel.VehicleSpeedRange = limits;
            
            % Cache control limits
            obj.ControlLimits(1,:) = limits;
            obj.LimitSpan(1) = diff(limits);
        end
        
        function limits = get.SpeedLimit(obj)
            limits = obj.BaseModel.VehicleSpeedRange;
        end
        
        function set.SteerLimit(obj, limits)
            validateattributes(limits,{'numeric'},{'increasing','numel',2,'finite'},'mobileRobotPropagator','SteeringLimits');
            coder.internal.assert(all(abs(limits) <= pi/2),'nav:navalgs:mobilerobotpropagator:InvalidSteerLimit','-pi/2','pi/2');
            obj.BaseModel.MaxSteeringAngle = max(abs(limits));
            
            % Cache control limits
            obj.ControlLimits(2,:) = limits;
            obj.LimitSpan(2) = diff(limits);
            
            if ~isa(obj.StateSpace,'stateSpaceSE2')
                % Update state-space turn radius
                obj.StateSpace.MinTurnRadius = obj.BaseModel.MinTurningRadius;
            end
        end
        
        function limits = get.SteerLimit(obj)
            limits = obj.ControlLimits(2,:);
        end
        
        function length = get.WheelBase(obj)
            length = obj.BaseModel.WheelBase;
        end
        
        function set.WheelBase(obj, length)
            validateattributes(length,{'numeric'},{'scalar','nonnegative','finite'},'mobileRobotPropagator','WheelBase');
            obj.BaseModel.WheelBase = length;
            
            if ~isa(obj.StateSpace,'stateSpaceSE2')
                % Update state-space turn radius
                obj.StateSpace.MinTurnRadius = obj.BaseModel.MinTurningRadius;
            end
        end
        
        function derivfcn = getDerivativeFcn(obj)
        %getDerivativeFcn Returns handle to bicycle model derivative function
        %
        %   Function prototype: qDot = fcn(q,u)
        %       Inputs:
        %           q    - 1x3 row vector [x,y,theta]
        %           u    - 1x2 row vector [v, alpha]
        %       Output:
        %           qDot - 1x3 row vector [xDot,yDot,thetaDot]
            
            model = obj.BaseModel;
            lims = obj.ControlLimits;
            derivfcn = @(q,u)nav.algs.internal.models.bicycle.derivative(model,lims,q,u);
        end
        
        function updateFcn = getUpdateFcn(~)
        %getUpdateFcn Returns handle to bicycle model update function
        %
        %   Function prototype: qNew = fcn(q,dq)
        %       Inputs:
        %           q    - 1x3 row vector [x,y,theta]
        %           dq   - 1x3 row vector [dx,dy,dTheta]
        %       Output:
        %           qNew - 1x3 row vector [x,y,theta]
            updateFcn = @nav.algs.internal.models.bicycle.update;
        end
    end
    
    methods (Access = ?nav.algs.internal.InternalAccess)
        function postParamUpdate(obj, propagator, propStruct) %#ok<INUSD> 
        %postParamUpdate Overrides the post param update hook in getSetters
            propagator.ControlLimits = obj.ControlLimits;
        end
    end
    
    methods (Hidden, Static)
        function validLaws = validControlLaws()
            validLaws = {'RandomSample','LinearPursuit','ArcPursuit'};
        end
        
        function qDot = derivative(model,limits,q,u)
            % Saturate commands if they exceed input limits
            u = min(max(u(:),limits(:,1)),limits(:,2))';
            
            % Calculate derivative
            qDot = model.derivative(q,u)';
        end
        
        function qNew = update(q, dq)
            qNew = q+dq;
        end
    end
end
