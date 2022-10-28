classdef linearPursuit < nav.algs.internal.getSetters
%This class is for internal use only. It may be removed in the future.

%linearPursuit Calculate steering angle using linear reference curve
%
%   linearPursuit is a control object which provides the mobileRobotPropagator 
%   with a policy for calculating a steering angle for a 2D mobile robot
%   given the robot's geometry, current pose and goal pose (both in SE(2)), 
%   and lookahead distance. The lookahead point is found along a straight 
%   line that connects the current pose to the goal pose, and the policy 
%   will randomly sample an initial velocity and hold it constant during 
%   propagation. This differs from the standard Pure Pursuit algorithm in
%   that the lookahead path is not fixed.
%
%   See also arcPursuit, randomSamples

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        SettableParams = {'LookaheadDist'};
        LookaheadDist = 2
    end
    
    methods
        function [u, numStep] = sampleControl(obj, propagator, q, u, qTgt)
        %sampleControl Randomly samples velocity and calculate steering angle
            model = propagator.KinematicModelInternal;
            velocity = model.ControlLimits(1)+rand*diff(model.ControlLimits(1,:));
            steerAngle = obj.simpleLinearPursuit(model.WheelBase,obj.LookaheadDist,q,qTgt,velocity);
            
            switch class(model)
                case 'nav.algs.internal.models.bicycle'
                    u = [velocity steerAngle];
                otherwise % ackermann
                    u = [velocity (steerAngle - q(end))/propagator.ControlStepSize];
            end
            
            numStep = randi([1,propagator.MaxControlSteps]);
        end
        
        function u = sampleNext(obj, propagator, q, u, qTgt)
        %sampleNext Holds velocity constant and updates steering input
            model = propagator.KinematicModelInternal;
            steerAngle = obj.simpleLinearPursuit(model.WheelBase,obj.LookaheadDist,q,qTgt,u(1));
            
            switch class(model)
                case 'nav.algs.internal.models.bicycle'
                    u(2) = steerAngle;
                otherwise
                    u(2) = (steerAngle - q(end))/propagator.ControlStepSize;
            end
        end
    end
    
    methods (Static)
        function desiredSteer = simpleLinearPursuit(wheelBase,lookAhead,q,qTgt,velocity)
        %simpleLinearPursuit Calculates a steering angle for linear lookahead primitive
        %
        %   ANGLE = simpleLinearPursuit(WHEELBASE, LOOKAHEAD, Q, QTGT, VELOCITY)
        %   Takes in a vehicle's WHEELBASE, VELOCITY, current and goal SE2 
        %   poses, Q and QTGT. 
        % 
        %   A line is extended from Q to QTGT and a lookahead point is 
        %   calculated a distance of LOOKAHEAD from Q along the line. An 
        %   arc is fit between Q and the lookahead point, tangent to Q, and 
        %   a steering angle, ANGLE, is calculated which will lead Q to 
        %   intersect the lookahead point.
        %
        %   See also evaluateArc, plotArc, posePointArc
            
            v = qTgt(1:2)-q(1:2);

            if velocity == 0
                velocity = 1;
            end
            pTgt = q(1:2)+v/norm(v,2)*lookAhead*sign(velocity);
            
            % Calculate arc tangent to vehicle pose and intersects with lookahead point
            [rAhead, ~, ~, ~] = nav.algs.internal.ControlPolicies.posePointArc(q,pTgt(1:2));

            % Calculate steering angle which guides rear-axle to intersection point
            desiredSteer = atan2(wheelBase,abs(rAhead))*sign(rAhead)*sign(velocity);
        end
    end
end
