% function handles 
classdef IKHelpers < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %   Copyright 2016-2018 The MathWorks, Inc.
    
    %#codegen

    %IKHELPERS Defines IK-specific function handles required by NLP solver
    methods(Static)
        
        function [cost, W, Jac, args] = computeCost(x, args)
            %computeCost 
            treeInternal = args.Robot; %robotics.manip.internal.RigidBodyTree
            bodyName = args.BodyName;
            Td = args.Tform; 
            weightMatrix = args.WeightMatrix;

            % T = robot.getTransform(x, bodyName);
            % J = robot.geometricJacobian(x, bodyName); % To be optimized
            
            [T, J] = efficientFKAndJacobianForIK(treeInternal, x, bodyName);
            
            % Note the geometric Jacobian of the robot and the Jacobian of
            % the cost error differ by a negative sign.
            Jac = -J;
            e = robotics.manip.internal.IKHelpers.poseError(Td, T);
            args.ErrTemp = e;
            args.CostTemp = 0.5*e'*weightMatrix*e; % 0.5 need justification
            args.GradTemp = (e'*weightMatrix*Jac)';
            cost = args.CostTemp;
            W = weightMatrix;
        end

        function grad = computeGradient(x, args)
            %computeGradient
            grad = args.GradTemp;
        end  

        function [en, evec] = evaluateSolution(x, args)
            %evaluateSolution
            en = norm(args.WeightMatrix * args.ErrTemp);
            evec = args.ErrTemp;
        end 

        function rc = randomConfig(args)
            %randomConfig
            rc = randomJointPositions(args.Robot);
        end

        function xc = clamping(x, args)
            %clamping
              xc = min(args.Limits(:,2), max(args.Limits(:,1), x) );
        end
        

        function errorvec = poseError(Td, T)
            %poseError
            R = T(1:3,1:3);
            Rd = Td(1:3,1:3);
            p = T(1:3,4);
            pd = Td(1:3,4);

            v = rotm2axang(Rd*R');
            err_orientation = v(4)*v(1:3)';
            err_linear = pd - p;

            errorvec = [err_orientation; err_linear ];
        end

        
  
    end

end