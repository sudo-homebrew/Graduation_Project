classdef PoseGraphHelpers < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%POSEGRAPHHELPERS Helper utilities to support pose graph representation
%   and optimization.

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen

    methods (Static)
        function wrappedTheta = wrapToPi_(theta)
            %wrapToPi_
            wrappedTheta = mod(theta + pi, 2*pi)-pi;
        end
    end

    % SE2
    methods (Static)

        function [TNew, infoMatNew]= invertConstraintSE2(T, infoMat)
            %invertConstraintSE2 Invert the SE2 edge constraints
            R = T(1:2, 1:2)'; % transpose
            p = R*(-T(1:2,3));
            TNew = [R, p; [0 0 1]];
            RR = blkdiag(R', 1);
            infoMatNew = RR*infoMat*RR';
        end

        function [e, Jaci, Jacj] = poseErrorSE2(Toi, Toj, Tij) %
            %poseErrorSE2

            Roi = Toi(1:2,1:2);
            ti = Toi(1:2,3);

            Roj = Toj(1:2,1:2);
            tj = Toj(1:2,3);

            Rij = Tij(1:2,1:2);
            tij = Tij(1:2,3);

              
            deltaTheta = robotics.core.internal.SEHelpers.veelogmSO2((Roi*Rij).'*Roj);
            e = [ (Rij')*(Roi'*(tj - ti) - tij); nav.algs.internal.PoseGraphHelpers.wrapToPi_(deltaTheta)];
            

            dRoidtheta = [Roi(1,2), -Roi(1,1); Roi(1,1), Roi(1,2) ];
            Jaci = [-Rij'*Roi', Rij'*dRoidtheta'*(tj - ti); 0 0 -1];
            Jacj = [ Rij'*Roi', [0;0]; 0 0 1];
        end

    end


    % SE3
    methods (Static)

        function [TNew, infoMatNew]= invertConstraintSE3(T, infoMat)
            %invertConstraintSE3 Invert the SE3 edge constraints
            %   If previously T transforms points in node B frame to node A
            %   frame, and infoMat is expressed in node B frame, this
            %   method returns TNew which transforms points in node A frame
            %   to node B frame, and infoMatNew which is expressed in node
            %   A frame.
            R = T(1:3, 1:3)'; % transpose
            p = R*(-T(1:3,4));
            TNew = [R, p; [0 0 0 1]];
            RR = blkdiag(R', R');
            infoMatNew = RR*infoMat*RR';
        end

        function [e, Jaci, Jacj] = poseErrorSE3(Toi, Toj, Tij) %
            %poseErrorSE3

            Tio = robotics.core.internal.SEHelpers.tforminvSE3(Toi);
            Tji = robotics.core.internal.SEHelpers.tforminvSE3(Tij);

            e = robotics.core.internal.SEHelpers.veelogmSE3(Tji*Tio*Toj);
            
            tau = e(1:3);
            phi = e(4:6);
            A = robotics.core.internal.SEHelpers.skew(phi);
            B = robotics.core.internal.SEHelpers.skew(tau);

            Mi = [-A, -B; zeros(3), -A];
            Mj = [ A, B; zeros(3), A];
            MSq = Mi * Mi;
            MSqSq = MSq * MSq;

            Jaci = (eye(6) + 0.5*Mi + (1/12)*MSq  - (1/720)*MSqSq )*robotics.core.internal.SEHelpers.adjointSE3(Tji);
            Jacj = -(eye(6) + 0.5*Mj + (1/12)*MSq - (1/720)*MSqSq );
            
        end
    end
    
    % SE2 & point
    methods (Static)
        function [e, Jaci, Jacj] = posePointErrorSE2(Toi, Toj, measurement)
            %posePointErrorSE2 Compute error between the estimated relative
            %   position of the 2D feature point and the measurement.
            %   The error is expressed in Toi frame. The 2D point position
            %   estimate w.r.t. world frame is encoded in Toj. The error
            %   derivatives w.r.t. the SE(2) pose and the 2D point are also
            %   computed.
            %
            %   reference: https://ieeexplore.ieee.org/document/5979949
            
            pose = robotics.core.internal.SEHelpers.tformToPoseSE2(Toi);
            x1 = pose(1);
            y1 = pose(2);
            sinTheta = sin(pose(3));
            cosTheta = cos(pose(3));
            x2 = Toj(1,3);
            y2 = Toj(2,3);
            
            sinThetaDy = (y2 - y1)*sinTheta;
            cosThetaDy = (y2 - y1)*cosTheta;
            sinThetaDx = (x2 - x1)*sinTheta;
            cosThetaDx = (x2 - x1)*cosTheta;
            
            e = [sinThetaDy + cosThetaDx;
                 cosThetaDy - sinThetaDx] - measurement(1:2,3);
             
            % derivative of the group action
            Jaci = [ -cosTheta,  -sinTheta,  cosThetaDy - sinThetaDx;
                      sinTheta,  -cosTheta, -sinThetaDy - cosThetaDx];
            Jacj = [  cosTheta,   sinTheta;
                     -sinTheta,   cosTheta];
            
        end
    end
    
    % SE3 & point
    methods (Static)
        function [e, Jaci, Jacj] = posePointErrorSE3(Toi, Toj, Tij)
            %posePointErrorSE3 Compute error between the estimated relative
            %   position of a 3D landmark point and its measurement.
            %   The Jacobian formulation in this method assumes
            %   "left multiply on the inverse" for the SE(3) perturbation,
            %   i.e. inv(expm(hat(epsilon))*Tio). The perturbation
            %   for the 3D point is just the simple "plus" on the original 
            %   (not inversed pose) Toj, i.e. Toj.position + delta_position
            %
            %   This is the formulation used in pose graph optimization,
            %   i.e. the handling of the SE(3) pose perturbation needs to
            %   be consistent with that in poseErrorSE3 method

            Tio = robotics.core.internal.SEHelpers.tforminvSE3(Toi);
            Tji = robotics.core.internal.SEHelpers.tforminvSE3(Tij);
            
            R = Tio(1:3,1:3);
            q = Toj(1:3,4);
            t = Tio(1:3,4);
            m = Tji(1:3,4);
            
            e = R*q + t + m;
            
            Jaci = [eye(3), -robotics.core.internal.SEHelpers.skew(R*q + t)];
            Jacj = R;
            
        end
        
        function [e, Jaci, Jacj] = posePointErrorSE3RightMultiply(Toi, Toj, measurement)
            %posePointErrorSE3RightMultiply Compute error between the estimated relative
            %   position of a 3D landmark point and its measurement. The
            %   Jacobian formulation in this method assumes 
            %   "direct right multiply" for SE(3) pose perturbation, 
            %   i.e. Toi*expm(hat(epsilon))
            R1T = Toi(1:3,1:3)';
            p1 = Toi(1:3,4);
            p2 = Toj(1:3,4);
            dp = p2 - p1;
            d = R1T * dp;
            
            e = d - measurement(1:3,4);
            
            Jaci = [-eye(3), robotics.core.internal.SEHelpers.skew(d)];
            Jacj = R1T;
        end
        
        function [e, Jaci, Jacj] = posePointErrorSE3LeftMultiply(Toi, Toj, measurement)
            %posePointErrorSE3LeftMultiply Compute error between the estimated relative
            %   position of the 3D landmark point and the measurement.
            %   The error is expressed in Toi frame. The 3D point position
            %   estimate w.r.t. world frame is encoded in Toj. The error
            %   derivatives w.r.t. the SE(3) pose (Toi) and the 3D point (Toj)
            %   are also computed and returned in Jaci and Jacj, respectively.
            %   This Jacobian formulation in this method assumes that the  
            %   perturbation on SE(3) pose is applied through 
            %   "direct left multiply", i.e. expm(hat(epsilon))*Toi
            
            R1T = Toi(1:3,1:3)';
            p1 = Toi(1:3,4);
            p2 = Toj(1:3,4);
            dp = p2 - p1;
            d = R1T * dp;
            e = d - measurement(1:3,4);
            
            % derivative of the group action
            Jaci = [ -R1T, R1T*robotics.core.internal.SEHelpers.skew(p2)]; % i.e. dp + t 
            % Jaci is 3-by-6, NOTE: changing orientation in Toi will also affect dp due to nontrivial t
            % This, however, won't be a problem if we use right multiply.
            Jacj = R1T; % 3-by-3
           
        end
    end
    
    % Sim3
    methods (Static)
        
        function [TNew, infoMatNew] =  invertConstraintSim3(T, infoMat)
            %invertConstraintSim3 Invert the Sim3 edge constraints
            %   If previously T transforms points in node B frame to node A
            %   frame, and infoMat is expressed in node B frame, this
            %   method returns TNew which transforms points in node A frame
            %   to node B frame, and infoMatNew which is expressed in node
            %   A frame.
        
            TNew = robotics.core.internal.Sim3Helpers.sforminvSim3(T);
            RR = blkdiag(T(1:3,1:3)/T(4,4), T(1:3,1:3)/T(4,4), 1);
            infoMatNew = RR*infoMat*RR';
        end
        
        function [e, Jaci, Jacj] = poseErrorSim3(Soi,Soj,Sij)
            %poseErrorSim3 computes the log of pose error(log(Sji*Soi*Soj)), 
            %   jacobian with respect to first node Soi (Jaci) and 
            %   jacobian with respect to second node Soj (Jacj).
            
            Sio = robotics.core.internal.Sim3Helpers.sforminvSim3(Soi);
            Sji = robotics.core.internal.Sim3Helpers.sforminvSim3(Sij);
            e = robotics.core.internal.Sim3Helpers.multiplyLogSim3(Sji,Sio,Soj);
            [Jaci,Jacj] = robotics.core.internal.Sim3Helpers.computeNumericalJacobianSim3(Sji,Soi,Soj);
        end
    end

    % shared
    methods (Static)

        function [cost, gradient, hessian] = poseGraphCost(posesMat, args)
            %poseGraphCost Compute pose graph optimization cost

            % extract edges and edge constraints
            edgeNodePairs = args.edgeNodePairs;
            edgeMeasurements = robotics.core.internal.BlockMatrix(args.edgeMeasurements, args.tformSize);
            edgeInfoMats = robotics.core.internal.BlockMatrix(args.edgeInfoMats, args.infoMatSize);

            cost = 0;
            numEdges = size(edgeNodePairs,1);

            poses = robotics.core.internal.BlockMatrix(posesMat, args.tformSize);
            numNodes = poses.NumRowBlocks;

            %bi = nav.algs.internal.BlockInserter(numNodes, numEdges, args.poseDeltaLength);
            nodeDims = args.nodeDims;
            bi = nav.algs.internal.BlockInserter2(numNodes, args.nodeMap, numEdges, nodeDims, args.poseDeltaLength);
            isLandmarkNode = args.IsLandmarkNode;

            %COST = [];
            for k = 1:numEdges
                i = edgeNodePairs(k,1);
                j = edgeNodePairs(k,2);

                Tij = edgeMeasurements.extractBlock(k, 1);
                OmegaIn = edgeInfoMats.extractBlock(k, 1);
                if isLandmarkNode(j)
                    Omega = OmegaIn(1:nodeDims(j), 1:nodeDims(j));
                else
                    Omega = OmegaIn;
                end
                Toi = poses.extractBlock(i, 1);
                Toj = poses.extractBlock(j, 1);

                [c, gradi, gradj, hessii, hessij, hessji, hessjj] = nav.algs.internal.PoseGraphHelpers.costBetweenTwoNodes(Toi, Toj, Tij, Omega, isLandmarkNode(j));

                cost = cost + c; %COST = [COST cost];
                bi.insertGradientBlock(i, gradi);
                bi.insertGradientBlock(j, gradj);
                bi.insertHessianFourBlocks(i, j, hessii, hessij, hessji, hessjj);
            end

            bi.insertHessianBlock(1, 1, eye(args.poseDeltaLength));

            gradient = bi.Gradient(1:args.nodeMap(numNodes) + args.nodeDims(numNodes) - 1);

            [I,J,V] = bi.getHessianCSC();
            hessian = sparse(I, J, V);

        end


        
        function [cost, gradi, gradj, hessii, hessij, hessji, hessjj] = costBetweenTwoNodes(Toi, Toj, measurement, Omega, nodejIsLandmark)
            %costBetweenTwoNodes
            %   Toi          - Estimate for node i, in world frame
            %   Toj          - Estimate for node j, in world frame
            %   measurement  - Measured relative pose or point observation
            %   Omega        - Information matrix associated with the measurement

            % define fixed input matrix sizes for codegen purpose.
            
            % pick the right "computeError" function
            if size(Omega, 1) == 2
                [e, Jaci, Jacj] = nav.algs.internal.PoseGraphHelpers.posePointErrorSE2(Toi(1:3,1:3), Toj(1:3,1:3), measurement(1:3,1:3));
            elseif size(Omega, 1) == 3
                if nodejIsLandmark
                    [e, Jaci, Jacj] = nav.algs.internal.PoseGraphHelpers.posePointErrorSE3(Toi(1:4,1:4), Toj(1:4,1:4), measurement(1:4,1:4));
                else
                    [e, Jaci, Jacj] = nav.algs.internal.PoseGraphHelpers.poseErrorSE2(Toi(1:3,1:3), Toj(1:3,1:3), measurement(1:3,1:3));
                end
            elseif size(Omega, 1) == 6
                %[e, Jaci, Jacj] = nav.algs.internal.PoseGraphHelpers.poseErrorSE3(Toi(1:4,1:4), Toj(1:4,1:4), measurement(1:4,1:4));
                [e, Jaci, Jacj] = nav.algs.internal.poseErrorSE3Numerical(Toi(1:4,1:4), Toj(1:4,1:4), measurement(1:4,1:4));
            else
                [e, Jaci, Jacj] = nav.algs.internal.PoseGraphHelpers.poseErrorSim3(Toi(1:4,1:4), Toj(1:4,1:4), measurement(1:4,1:4));
            end


            cost = e'*Omega*e;

            gradi = Jaci'*Omega*e;
            gradj = Jacj'*Omega*e;

            hessii = Jaci'*Omega*Jaci;
            hessij = Jaci'*Omega*Jacj;
            hessji = Jacj'*Omega*Jaci;
            hessjj = Jacj'*Omega*Jacj;
        end
        
    end
end
