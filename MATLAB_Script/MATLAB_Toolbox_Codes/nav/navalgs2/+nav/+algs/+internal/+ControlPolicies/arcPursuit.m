classdef arcPursuit < nav.algs.internal.getSetters
%This class is for internal use only. It may be removed in the future.

%arcPursuit Calculate steering angle using arc-based reference curve
%
%   arcPursuit is a control object which provides the mobileRobotPropagator 
%   with a policy for calculating a steering angle for a 2D mobile robot,
%   given the robot's geometry, current pose and goal pose (both in SE(2)), 
%   and lookahead distance. The lookahead point is found along an arc which
%   intersects the current xy location and ends tangent to the goal pose.
%   The policy will randomly sample an initial velocity and hold it 
%   constant during propagation. 
%
%   See also linearPursuit, randomSamples

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        SettableParams = {'LookaheadDist'};
        LookaheadDist = 2
    end
    
    methods
        function [u, numStep] = sampleControl(obj, propagator, q, u, qTgt)
        %sampleControl Samples a velocity and calculates steering angle based on model properties
            
            model = propagator.KinematicModelInternal;
            velocity = model.ControlLimits(1)+rand*diff(model.ControlLimits(1,:));
            steerAngle = obj.simpleArcPursuit(model.WheelBase,obj.LookaheadDist,q,qTgt,velocity);
            
            switch class(model)
                case 'nav.algs.internal.models.bicycle'
                    u = [velocity steerAngle];
                otherwise % ackermann
                    u = [velocity (steerAngle - q(end))/propagator.ControlStepSize];
            end
            
            numStep = propagator.MaxControlSteps;
        end
        
        function u = sampleNext(obj, propagator, q, u, qTgt)
        %sampleNext Holds velocity constant and updates steering input
            model = propagator.KinematicModelInternal;
            steerAngle = obj.simpleArcPursuit(model.WheelBase,obj.LookaheadDist,q,qTgt,u(1));
            
            switch class(model)
                case 'nav.algs.internal.models.bicycle'
                    u(2) = steerAngle;
                otherwise
                    u(2) = (steerAngle - q(end))/propagator.ControlStepSize;
            end
        end
    end
    
    methods (Static)        
        function angle = simpleArcPursuit(wheelBase,lookAhead,q,qTgt,velocity)
        %simpleArcPursuit Calculates a steering angle for arc-based lookahead primitive
        %
        %   ANGLE = simpleArcPursuit(WHEELBASE, LOOKAHEAD, Q, QTGT, VELOCITY)
        %   Takes in a vehicle's WHEELBASE, VELOCITY, current and goal SE2 
        %   poses, Q and QTGT. A constant curvature arc is created which
        %   intesects with Q's xy position and tangentially intersects with
        %   QTGT. 
        %   
        %   A lookahead point is calculated a distance of LOOKAHEAD from Q 
        %   along the arc, and a second arc is fit between Q and the
        %   lookahead point, tangent to Q. A steering angle, ANGLE, is then
        %   calculated which will lead Q to intersect the lookahead point.
        %
        %   See also evaluateArc, plotArc, posePointArc
        
            q = q(:)';
            qTgt = qTgt(:)';

            if velocity == 0
                velocity = 1;
            elseif velocity < 0
                qTgt(3) = qTgt(3)+pi;
            end
            
            [rGoal,~,~,sGoal] = nav.algs.internal.ControlPolicies.posePointArc(qTgt,q,true);

            % Calculate desired heading angle based on an arc that is tangent to
            % the goal pose and intesects the current xy point.
            if rGoal > 1e6
                v1 = qTgt(1:2)-q(1:2);
                v2 = [cos(q(3)) sin(q(3))]*sign(velocity);
                converging = sum(v1.*v2)/(norm(v1)*norm(v2)) >= 0;
                % Likely in a singular state
                if converging
                    angle = 0;
                else
                    angle = pi/2;
                end
            else
                if abs(sGoal) > lookAhead
                    s = (abs(sGoal)-lookAhead)*sign(sGoal);
                    pTgt = nav.algs.internal.ControlPolicies.evaluateArc(qTgt,rGoal,s);
                else
                    pTgt = qTgt(1:2);
                end
                pTgt = pTgt(:)';

                % Calculate arc tangent to vehicle pose and intersects with lookahead point
                [rAhead, ~, ~, ~] = nav.algs.internal.ControlPolicies.posePointArc(q,pTgt);

                % Calculate steering angle which guides rear-axle to intersection point
                angle = atan2(wheelBase,abs(rAhead))*sign(rAhead);
            end
        end
    end
end
