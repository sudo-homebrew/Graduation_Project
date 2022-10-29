classdef ackermann < nav.algs.internal.getSetters
%This class is for internal use only. It may be removed in the future.

%ackermann Helper class representing Ackermann model with steer-rate control inputs

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        SettableParams = {'WheelBase','SpeedLimit','SteerRateLimit'};
    end
    
    properties (Dependent)
        WheelBase
        SpeedLimit
        SteerRateLimit
    end
    
    properties (SetAccess = ?nav.algs.internal.InternalAccess)
        ControlLimits = [-1 1; [-pi/4 pi/4]*10];
        LimitSpan = [2 pi/2*10];
        StateSpace
        BaseModel
    end
    
    methods
        function obj = ackermann(ss)
            narginchk(0,1);
            if nargin < 1
                ss = stateSpaceSE2;
            end
            % Initialize state-space and underlying kinematic model
            obj.StateSpace = nav.algs.internal.StateSpace.AckermannStateSpace(ss);
            obj.BaseModel = ackermannKinematics('VehicleSpeedRange',[-1 1]);
            
            % Cache control limits
            obj.ControlLimits = [obj.SpeedLimit; obj.SteerRateLimit];
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
        
        function set.SteerRateLimit(obj, limits)
            validateattributes(limits,{'numeric'},{'increasing','numel',2,'finite'},'mobileRobotPropagator','SteeringLimits');
            
            % Cache control limits
            obj.ControlLimits(2,:) = limits;
            obj.LimitSpan(2) = diff(limits);
        end
        
        function limits = get.SteerRateLimit(obj)
            limits = obj.ControlLimits(2,:);
        end
        
        function length = get.WheelBase(obj)
            length = obj.BaseModel.WheelBase;
        end
        
        function set.WheelBase(obj, length)
            validateattributes(length,{'numeric'},{'scalar','nonnegative','finite'},'mobileRobotPropagator','WheelBase')
            obj.BaseModel.WheelBase = length;
        end
        
        function fcn = getDerivativeFcn(obj)
        %getDerivativeFcn Returns handle to ackermann model derivative function
        %
        %   Function prototype: qDot = fcn(q,u)
        %       Inputs:
        %           q    - 1x4 row vector [x,y,theta,psi]
        %           u    - 1x2 row vector [v, psiDot]
        %       Output:
        %           qDot - 1x4 row vector [xDot,yDot,thetaDot,psiDot]
            
            model = obj.BaseModel;
            lims = obj.ControlLimits;
            fcn = @(q,u)nav.algs.internal.models.ackermann.derivative(model,lims,q,u);
        end
        
        function updateFcn = getUpdateFcn(~)
        %getUpdateFcn Returns handle to ackermann model update function
        %
        %   Function prototype: qNew = fcn(q,dq)
        %       Inputs:
        %           q    - 1x4 row vector [x,y,theta,psi]
        %           dq   - 1x4 row vector [dx,dy,dTheta,dPsi]
        %       Output:
        %           qNew - 1x4 row vector [x,y,theta,psi]
            updateFcn = @nav.algs.internal.models.ackermann.update;
        end
    end
    
    methods (Access = ?nav.algs.internal.InternalAccess)
        function postParamUpdate(obj, propagator, propStruct) %#ok<INUSD> 
        %postParamUpdate Overrides the post param update hook in getSetters
            propagator.ControlLimits = obj.ControlLimits;
        end
        
        function updateLimits(obj)
        %updateLimits Cache control limits used in derivative function
            obj.ControlLimits = [obj.SpeedLimit; obj.SteerRateLimit];
            obj.LimitSpan = diff(obj.ControlLimits');
        end
    end
    
    methods (Hidden, Static)
        function validLaws = validControlLaws()
            validLaws = {'RandomSample','LinearPursuit','ArcPursuit'};
        end
        
        function qDot = derivative(model,limits,q,u)
            % Saturate commands if they exceed input limits
            u = min(max(u(:),limits(:,1)),limits(:,2))';
            
            % Call base-model derivative
            qDot = model.derivative(q,u)';
        end
        
        function qNew = update(q, dq)
            qNew = q+dq;
        end
    end
end
