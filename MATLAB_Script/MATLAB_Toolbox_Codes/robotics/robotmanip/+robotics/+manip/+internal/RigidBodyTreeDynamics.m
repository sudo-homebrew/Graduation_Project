classdef RigidBodyTreeDynamics < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.

    %RIGIDBODYTREEDYNAMICS Collection of rigid body tree dynamics
    %   algorithms.
    
    %   Copyright 2016-2021 The MathWorks, Inc.    
    
    %#codegen

    methods (Static)
        function [H, lambda] = massMatrix(robot, q)
            %massMatrix Compute the mass matrix, H, of the robot in the
            %   configuration q.
            %   robot - a valid robotics.manip.internal.RigidBodyTree obj
            %   q     - a valid configuration (position) vector for robot
            %
            %   H      - mass matrix of robot for configuration q
            %   lambda - a vector of indices, lambda(i) is the column
            %            number of the last nonzero entry (count from left) 
            %            on row i of the lower triangle of H (not counting 
            %            main diagonal). If no nonzero entry is found,
            %            lambda(i) is 0.
            nb = robot.NumBodies;
            Ic = cell(1,nb); % composite-rigid-body inertia
            
            X = cell(1,nb); % spatial transform from parent of body i to body i
            
            vNum = robot.VelocityNumber;
            H = zeros(vNum, vNum);
            lambda_ = zeros(1,nb);
            lambda = zeros(1,vNum);
            
            % preparation
            for i = 1:nb
                Ic{i} = robot.Bodies{i}.SpatialInertia;
                p = robot.PositionDoFMap(i,:);
                if p(2) < p(1)
                    T = robot.Bodies{i}.Joint.transformBodyToParent(0); % fixed joint
                else
                    T = robot.Bodies{i}.Joint.transformBodyToParent(q(p(1):p(2)));
                end
                Tinv = robotics.manip.internal.tforminv(T);
                X{i} = robotics.manip.internal.tformToSpatialXform(Tinv);
            end
            
            
            
            % main loop
            for i = nb:-1:1
                pid = robot.Bodies{i}.ParentIndex; % parent id
                a = robot.VelocityDoFMap(i,:);
                if pid > 0
                    Ic{pid} = Ic{pid} + X{i}'*Ic{i}*X{i};
                    lambda_(i) = pid;
                    
                    while lambda_(i)>0 && strcmp(robot.Bodies{lambda_(i)}.Joint.Type, 'fixed')
                        lambda_(i) = robot.Bodies{lambda_(i)}.ParentIndex;
                    end
                end
                
                b = robot.VelocityDoFMap(i,:); 
                if b(1) <= b(2)
                    TiDHOffset = robot.Bodies{i}.Joint.ChildToJointTransform; % an offset is needed when ChildToJointTransform is not identity (i.e. when rigid body tree is created from DH parameters)
                    XiDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TiDHOffset));
                    Si = XiDHOffset * robot.Bodies{i}.Joint.MotionSubspace; % check dimension
                    Fi = Ic{i}*Si;
                    H(a(1):a(2), a(1):a(2)) = Si'*Fi;
                    
                    Fi = X{i}'*Fi;
                    j = pid;

                    while j > 0
                        TjDHOffset = robot.Bodies{j}.Joint.ChildToJointTransform;
                        XjDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TjDHOffset));
                        Sj = XjDHOffset*robot.Bodies{j}.Joint.MotionSubspace;
                        b = robot.VelocityDoFMap(j,:);
                        if b(1) <= b(2)
                            Hji = Sj'*Fi;
                            H(b(1):b(2), a(1):a(2)) = Hji;
                            H(a(1):a(2), b(1):b(2)) = Hji';   
                        end
                        Fi = X{j}'*Fi; % X{j}: parent(j) -> j
                        
                        j = robot.Bodies{j}.ParentIndex;
                        
                    end
                end
            end
            
            % post-processing lambda vector
            % lambda_(i) is either the index of the first non-fixed parent of
            % body i or 0 (base)
            mask = robot.VelocityDoFMap(:,1) <=  robot.VelocityDoFMap(:,2); % non fixed mask
            
            nonFixedIndices = find(mask)'; % make it a row vector
            for k = 1:length(nonFixedIndices)
                id = nonFixedIndices(k);
                a = robot.VelocityDoFMap(id,:);
                lambda(a(1):a(2)) = (a(1):a(2)) - ones(1, a(2)-a(1)+1);
                if lambda_(id) == 0
                    lambda(a(1)) = 0;
                else
                    b = robot.VelocityDoFMap(lambda_(id),:);
                    lambda(a(1)) = b(2);
                end
            end
        end
        
        function tau = inverseDynamics(robot, q, qdot, qddot, fext)
            %inverseDynamics Given the current (or desired) joint positions
            %   and velocities, the desired joint accelerations, and
            %   optionally the external forces applied to each body,
            %   compute the required joint torque
            %
            %   robot - a valid robotics.manip.internal.RigidBodyTree obj
            %   q     - a valid configuration (position) vector for robot
            %   qdot  - a valid joint velocity vector for robot
            %   qddot - a valid joint acceleration vector for robot
            %   fext  - a valid external force matrix for robot
            %
            %   tau   - joint torque vector
            
            % base (always fixed)
            %v0 = zeros(6,1);
            a0 = [zeros(3,1); -robot.Gravity'];
            
            nb = robot.NumBodies;
            
            X = cell(1,nb); % spatial transform from body parent(i) to body i
            Xtree = cell(1,nb); % spatial transform from body i to base
            vJ = zeros(6, nb); % spatial velocity across joint i, in frame i
            vB = zeros(6, nb); % spatial velocity of body i
            
            aB = zeros(6, nb); % spatial acceleration of body i
            f = zeros(6, nb); % spatial force on body i
            tau = zeros(size(qdot));
            
            for k = 1:nb % required by codegen
                Xtree{k} = eye(6);
                X{k} = eye(6);
            end
            
            % outbound 
            for i = 1:nb
                S = robot.Bodies{i}.Joint.MotionSubspace;
                a = robot.PositionDoFMap(i,:);
                b = robot.VelocityDoFMap(i,:);
                
                XDHOffset = eye(6);
                if a(2) < a(1) % if fixed joint
                    T = robot.Bodies{i}.Joint.transformBodyToParent(0); % fixed joint
                    qddoti = 0;
                    vJ(:,i) = zeros(6,1);
                else
                    qi = q(a(1):a(2));
                    qdoti = qdot(b(1):b(2));
                    qddoti = qddot(b(1):b(2));
                    T = robot.Bodies{i}.Joint.transformBodyToParent(qi);
                    TDHOffset = robot.Bodies{i}.Joint.ChildToJointTransform; % an offset is needed when ChildToJointTransform is not identity (i.e. when rigid body tree is created from DH parameters)
                    XDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TDHOffset));
                    vJ(:,i) = XDHOffset*S*qdoti;
                end
                
                Tinv = robotics.manip.internal.tforminv(T);
                X{i} = robotics.manip.internal.tformToSpatialXform(Tinv);% parent(i) -> i
                
                pid = robot.Bodies{i}.ParentIndex;
                
                if pid > 0
                    vB(:,i) = vJ(:,i) + X{i}*vB(:,pid);
                    aB(:,i) = X{i}*aB(:,pid) + XDHOffset*S*qddoti + ...
                                robotics.manip.internal.crossMotion(vB(:,i), vJ(:,i));
                    Xtree{i} = Xtree{pid}*robotics.manip.internal.tformToSpatialXform(T); %i -> parent(i) -> ... -> 0
                else % if parent is base
                    vB(:,i) = vJ(:,i);
                    aB(:,i) = X{i}*a0 + XDHOffset*S*qddoti;
                    Xtree{i} = robotics.manip.internal.tformToSpatialXform(T);
                end
                I = robot.Bodies{i}.SpatialInertia;
                h = I*vB(:,i);
                f(:,i) = I*aB(:,i) + robotics.manip.internal.crossForce(vB(:,i), h) ...
                          - Xtree{i}'*fext(:,i);
            end
            
            % inbound
            for i = nb:-1:1
                if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
                    TDHOffset = robot.Bodies{i}.Joint.ChildToJointTransform;
                    XDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TDHOffset));
                    S = XDHOffset*robot.Bodies{i}.Joint.MotionSubspace;
                    taui = S'*f(:,i);
                    b = robot.VelocityDoFMap(i,:);
                    tau(b(1):b(2)) = taui;
                end
                pid = robot.Bodies{i}.ParentIndex;
                if pid > 0
                    f(:,pid) = f(:,pid) + X{i}'*f(:,i);
                end
                
            end

        end
        
        function [IcrbBase, IcrbBodies, X] = compositeRigidBodyInertia(robot, q)
            %compositeRigidBodyInertia Compute the composite rigid body
            %   inertia (CRBI) of each body of robot under configuration q. 
            %
            %   robot is a robotics.manip.internal.RigidBodyTree obj
            %
            %   IcrbBase   - The composite rigid body inertia at base (a
            %                6x6 matrix)
            %   IcrbBodies - An 1-by-nb cell array, where nb is the number 
            %                of bodies. Each cell contains the composite
            %                rigid body inertia of body i, relative to body
            %                i frame.
            %   X          - An 1-by-nb cell array, with cell i containing 
            %                the spatial transform (for motion vectors) 
            %                from body parent(i) to body i.
            
            nb = robot.NumBodies;
            IcrbBodies = cell(1,nb); % composite-rigid-body inertia for each body
            IcrbBase = zeros(6,6);
            X = cell(1,nb); % spatial transform from parent to current body
            
            % outbound
            for i = 1:nb
                IcrbBodies{i} = robot.Bodies{i}.SpatialInertia;
                p = robot.PositionDoFMap(i,:);
                if p(2) < p(1)
                    T = robot.Bodies{i}.Joint.transformBodyToParent(0); % fixed joint
                else
                    T = robot.Bodies{i}.Joint.transformBodyToParent(q(p(1):p(2)));
                end
                Tinv = robotics.manip.internal.tforminv(T);
                X{i} = robotics.manip.internal.tformToSpatialXform(Tinv);
            end
            
            % inbound
            for i = nb:-1:1
                pid = robot.Bodies{i}.ParentIndex;
                if pid > 0
                    IcrbBodies{pid} = IcrbBodies{pid} + X{i}'*IcrbBodies{i}*X{i};
                else
                    IcrbBase = IcrbBase + X{i}'*IcrbBodies{i}*X{i};
                end
            end
        end
        
        function [pCoM, totalmass, cmm] = centroidalMomentumMatrix(robot, q)
            %centroidalMomentumMatrix Compute the centroidal momentum
            %   matrix (CMM) for robot under configuration q. CMM is the
            %   matrix that relates joint velocities to the centroidal
            %   momentum (6-by-1 vector), i.e. the aggregate momentum of all 
            %   bodies in the robot projected to the CoM frame.
            %
            %   Also, base frame (frame 0) and the CoM frame (frame G) are 
            %   always set to be parallel. 
            
            [IcrbBase, IcrbBodies, X] = robotics.manip.internal.RigidBodyTreeDynamics.compositeRigidBodyInertia(robot, q);
            
            totalmass = IcrbBase(6,6); % totalmass cannot be zero
            pCoMSkew = IcrbBase(1:3, 4:6)/totalmass;
            pCoM = [-pCoMSkew(2,3), pCoMSkew(1,3), -pCoMSkew(1,2)]';
            XG = [eye(3), zeros(3); pCoMSkew, eye(3)]; % spatial transform from CoM frame to base
            
            nb = robot.NumBodies;
            Xtree = cell(1,nb); % spatial transform from CoM to body i
            vNum = robot.VelocityNumber;
            
            for k = 1:nb % required by codegen
                Xtree{k} = eye(6);
            end
            
            if nargout > 2
                cmm = zeros(6, vNum);

                for i = 1:nb
                    body = robot.Bodies{i};
                    pid = body.ParentIndex;
                    if pid == 0
                        Xtree{i} = X{i}*XG; 
                    else
                        Xtree{i} = X{i}*Xtree{pid}; 
                    end
                    a = robot.VelocityDoFMap(i,:);
                    if a(1) <= a(2)
                        S = body.Joint.MotionSubspace;
                        TDHOffset = body.Joint.ChildToJointTransform; % an offset is needed for DH robots
                        XDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TDHOffset));
                        cmm(:, a(1):a(2)) = Xtree{i}'*IcrbBodies{i}*XDHOffset*S;
                    end
                end
            end
            
        end
    
           
        function qddot = forwardDynamicsAB(robot, q, qdot, tau, fext)
            %forwardDynamicsAB Given the robot state (joint positions and
            %   velocities), joint input (joint torques), and the
            %   external forces on each body, compute the resultant joint
            %   accelerations using Articulated Body Algorithm
            
            %vel0 = zeros(6,1);
            a0 = [zeros(3,1); - robot.Gravity'];
            
            nb = robot.NumBodies;
            
            vel = zeros(6, nb); % spatial velocity for each body
            acc = zeros(6, nb); % spatial acceleration for each body
            qddot = zeros(robot.VelocityNumber, 1);
            cori = zeros(6, nb); % spatial accelerations due to coriolis effect for each body
            I_A = cell(1, nb); % articulated-body inertia for each body
            p_A = zeros(6, nb); % bias force for each body
            X = cell(1,nb); % spatial transform from body parent(i) to body i
            Xtree = cell(1,nb); % spatial transform from body i to base
            
            for i = 1:nb % pass 1, outward
                
                body = robot.Bodies{i};
                
                vJ = zeros(6,1); % velocity across joint j
                if ~strcmp(body.JointInternal.Type, 'fixed')
                    idxp = robot.PositionDoFMap(i,:);
                    qi = q(idxp(1):idxp(2));
                    T = body.JointInternal.transformBodyToParent(qi);
                    
                    idxv = robot.VelocityDoFMap(i,:);
                    qdi = qdot(idxv(1):idxv(2));
                    
                    TDHOffset = body.Joint.ChildToJointTransform; % an offset is needed when ChildToJointTransform is not identity (i.e. when rigid body tree is created from DH parameters)
                    XDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TDHOffset));
                    
                    vJ = XDHOffset*body.Joint.MotionSubspace*qdi;
                else
                    T = body.JointInternal.transformBodyToParent(0);
                end
                
                Tinv = robotics.manip.internal.tforminv(T);
                X{i} = robotics.manip.internal.tformToSpatialXform(Tinv); % body parent(i) to body i
                
                pid = body.ParentIndex;
                if pid == 0
                    vel(:,i) = vJ;
                    Xtree{i} = robotics.manip.internal.tformToSpatialXform(T);
                else
                    vel(:,i) = X{i}*vel(:, pid) + vJ;
                    Xtree{i} = Xtree{pid}*robotics.manip.internal.tformToSpatialXform(T);
                end

                cori(:,i) = robotics.manip.internal.crossMotion(vel(:,i), vJ);
                I_A{i} = body.SpatialInertia;
                p_A(:,i) = robotics.manip.internal.crossForce(vel(:,i), I_A{i}*vel(:,i)) ...
                           - Xtree{i}'*fext(:,i);
            end
            
            for i = nb:-1:1 % pass 2, inward
                body = robot.Bodies{i};
                pid = body.ParentIndex;
                
                if pid > 0 
                    % propagating articulated body inertia across joint i
                    
                    if ~strcmp(body.Joint.Type, 'fixed') % if the joint is either revolute or prismatic
                        TDHOffset = body.Joint.ChildToJointTransform;
                        XDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TDHOffset));
                    
                        S = XDHOffset*body.Joint.MotionSubspace;
                        I = I_A{i};
                        U = I*S;
                        D = S'*U;
                        Ia = I - U*(1/D)*U'; % Ia is I_A{i} being propagated across (revolute) joint i
                        p = p_A(:,i);
                        idxv = robot.VelocityDoFMap(i, :);
                        pa = p + Ia*cori(:,i) + U*(1/D)*(tau(idxv(1):idxv(2)) - S'*p); % pa is p_A{i} being propagated across (revolute) joint i

                        I_A{pid} = I_A{pid} + X{i}'*Ia*X{i};
                        p_A(:,pid) = p_A(:,pid) + X{i}'*pa;

                    else
                        I_A{pid} = I_A{pid} + X{i}'*I_A{i}*X{i};
                        p_A(:,pid) = p_A(:,pid) + X{i}'*p_A(:,i);                         
                    end
                end
                    
            end
            
            for i = 1:nb % pass 3, outward again

                body = robot.Bodies{i};
                pid = body.ParentIndex;
                                
                if pid > 0
                    acc(:,i) = X{i}*acc(:,pid) + cori(:,i);
                else
                    acc(:,i) = X{i}*a0 + cori(:,i);
                end
                I = I_A{i};
                p = p_A(:,i);
                idxv = robot.VelocityDoFMap(i,:);
                
                if ~strcmp(body.Joint.Type, 'fixed')
                    TDHOffset = body.Joint.ChildToJointTransform;
                    XDHOffset = robotics.manip.internal.tformToSpatialXform(robotics.manip.internal.tforminv(TDHOffset));
                    S = XDHOffset*body.JointInternal.MotionSubspace;
                    qdd = (1/(S'*I*S))*(tau(idxv(1)) - S'*p - S'*I*acc(:,i));
                    qddot(idxv(1):idxv(2)) = qdd;
                    acc(:,i) = acc(:,i) + S*qdd;
                end                
                
            end
            
        end

        function qddot = forwardDynamicsCRB(robot, q, qdot, tau, fext)
            %forwardDynamicsAB Given the robot state (joint positions and
            %   velocities), joint input (joint torques), and the
            %   external forces on each body, compute the resultant joint
            %   accelerations using Composite Rigid Body Algorithm  
            
            [H, lambda] = robotics.manip.internal.RigidBodyTreeDynamics.massMatrix(robot, q);
            
            vNum = robot.VelocityNumber;
            bias = robotics.manip.internal.RigidBodyTreeDynamics.inverseDynamics(robot, q, qdot, zeros(vNum, 1), fext);
            
            y = tau - bias;
            L = robotics.manip.internal.specialCholesky(H, lambda);
            
            % specialized in-situ backward substitution
            % L^(-T)
            for i = vNum:-1:1
                y(i) = y(i)/L(i,i);
                j = lambda(i);
                while j>0
                    y(j) = y(j) - y(i)*L(i,j); 
                    j = lambda(j);
                end
            end
            
            % L^(-1)
            for i = 1:vNum
                j = lambda(i);
                while j>0
                    y(i) = y(i) - L(i,j)*y(j);
                    j = lambda(j);
                end
                y(i) = y(i)/L(i,i);
            end
            qddot = y;
            
        end
    end
    
end

