classdef TimedElasticBandCarGraph < handle & nav.algs.internal.InternalAccess
%TimedElasticBandCarGraph creates a graph for Timed Elastic Band optimizer
%for Car-like robots.
%   It contains the information about the graph which contains metadata for
%   the graph and necessary information required for the computation of
%   errors, Jacobians, gradients, and Hessians. For Timed Elastic band
%   formulation lot of the graph information is directly dependent on input
%   path, hence, some of that information is not stored. For e.g. motion
%   edges are always between consecutive poses, making storing of motion
%   edges redundant.
%
%   This graph computes errors, cost, gradients, hessians while considering
%   12 types of edges, where 1 is primary objective of the cost function
%   and other 11 are considered as soft-constraints added to cost function.
%       1. Time component of the cost function.
%       2. Non-holonomic motion edge constraints
%       3. Min Turning radius edge constraints
%       4. Velocity edge constraints
%       5. Angular Velocity edge constraints
%       6. Acceleration edge constraints
%       7. Angular Acceleration edge constraints
%       8. Acceleration at Start edge constraints
%       9. Angular Acceleration at Start edge constraints
%       10. Acceleration at End edge constraints
%       11. Angular Acceleration at End edge constraints
%       12. Obstacle edge constraints
%
% [1] C. Rosmann, W. Feiten, T. Wosch, F. Hoffmann and T. Bertram:
%     Efficient trajectory optimization using a sparse model. Proc. IEEE
%     European Conference on Mobile Robots, Spain, Barcelona, 2013, pp.
%     138–143.
% [2] C. Rosmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory
%     Optimization and Control for Car-Like Robots, IEEE/RSJ International
%     Conference on Intelligent Robots and Systems (IROS), Vancouver, BC,
%     Canada, Sept. 2017.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties (GetAccess = public, SetAccess=private)
        % Path after optimization
        Path

        % DeltaT after optimization
        DeltaT

        % Parameters required for computation of error for e.g. Safety Margin
        Params

        % Positions of all obstacles
        ObstPos
    end

    properties (Dependent=true, GetAccess = public, SetAccess=private)
        % Linear and Angular Velocity after optimization
        Velocity

        % Linear and Angular Acceleration after optimization
        Acceleration
    end

    properties (Access = ?nav.algs.internal.InternalAccess)
        % Start pose (first pose in path). This is stored to reconstruct path.
        % Start is a fixed node and is not a part of state vector, but is
        % needed to compute edge errors for Start node. [1 x 2]
        Start

        % Goal pose (last pose in path). This is stored to reconstruct path.
        % Goal is a fixed node and is not a part of state vector, but is
        % needed to compute edge errors for Goal node. [1 x 2]
        Goal

        % Index where DeltaT elements start in state vector. [Scalar]
        DeltaTXIdxStart

        % [PoseIdx ObstIdx] list of edges representing which obstacles are
        % influencing which poses. [M x 2]
        ObstEdges

        % convenience property for size(ObstEdges,1). [Scalar]
        NumMotionEdges

        % Dimension of state vector for optimization
        Xdim

    end

    properties (Access = private)
        % Next few properties are values used in computation of errors. These
        % are computed in motionMetrics and used in individual error functions.
        % Number of rows for these properties depend on the number of
        % poses in the path. Their sizes are mentioned in their
        % description.
        % N = Number of poses in the Path
        % M = Number of obstacle edges.

        % Time taken to travel between pose_k and pose_k+1. This is used
        % in computation of time error and velocity. [N-1 x 1]
        DeltaT1

        % Value for Non-Holonomic Motion for consecutive pose pair, this is
        % used for computation of Non-holonomic motion error. [N-1 x 1]
        NhMotion

        % Turning radius for consecutive pose pair, this is used for rho error
        % computation. [N-1 x 1]
        Rho21

        % Linear and Angular Velocity between consecutive pose pair calculated
        % using the consecutive poses and corresponding deltaT, this is used for
        % velocity error computation. [N-1 x 2]
        Vel21

        % Linear and Angular Acceleration between three consecutive pose,
        % calculated using the three poses and corresponding deltaT1 and deltaT2,
        % this is used for acceleration error computation. It should be [N-2 x
        % 2], but to enable vectorization we pad it with [0 0] in last row.
        % [N-1 x 2].
        Accel

        % Linear and Angular Acceleration for start of the path. [1x2]
        StartAccel

        % Linear and Angular Acceleration for end of the path. [1x2]
        EndAccel

        % Error for this call to cost. This is reused in error, gradient, and
        % hessian. [(11*(N-1) + M) x 1]
        Err

        % Jacobian for this call to cost. This is reused in gradient, and
        % hessian. [(11*(N-1) + M) x Xdim]
        Jacobian

        % Block diagonal weight matrix for this graph. This is reused in cost,
        % gradient, and hessian. [NumElem in X x NumElem in X]
        Weight
    end

    properties (Constant)
        % Dimension of the state nodes in the graph. We are considering SE2 for
        % now, hence it's 3
        PoseNodeDim = 3;

        % Maximum number of elements an edge can have. Each state node has 3
        % elements and deltaT node has 1 element. Acceleration edge has max
        % elements, 3 state nodes (3*3) and 2 deltaT nodes (2*1), totaling 9+2=11
        MaxMotionInput = 11;

        % Step for Numerical Jacobian
        NumJacStep = 1e-9;
    end

    methods (Access = public)

        function obj = TimedElasticBandCarGraph(params, obsts)
            %TimedElasticBandCarGraph constructs the object using parameters and obstacle list
            obj.Params = params;
            obj.ObstPos = obsts;

            % Init private properties
            obj.Start = zeros(1,obj.PoseNodeDim);
            obj.Goal = obj.Start;
            obj.DeltaTXIdxStart = 0;
            obj.Xdim = 0;
            obj.NumMotionEdges = 0;
            maxWsz = min(20000, params.MaxPathStates * size(obsts, 1)); 
            %Changing it to approx (1/100000*intmax),
            %values greater than 20000 practically not possible as
            %computation time will be in hours.
            obj.Weight = sparse(maxWsz, maxWsz);
        end

        function build(obj, pth, obstWeightMultiplier)
        %build calculates edges and meta information for the graph.
        %   build(obj, pth) takes poses in pth to compute metadata for
        %   the graph like dimension of stateVector, Start and Goal pose,
        %   etc... It also creates the obstacle edges for the scenario using
        %   pth and ObstPos property in obj. This function should be called
        %   before static functions can be called.

            % In TEB Graph we have two fixed nodes, start and goal. They are
            % static throughout optimization but are needed for error
            % computation of edge constraints involving these nodes. Save them
            % to variable.
            obj.Start = pth(1,1:obj.PoseNodeDim);
            obj.Goal = pth(end,1:obj.PoseNodeDim);

            % Since the state vector "X" consists of pose(dim=3) and
            % deltaT(dim=1), save the boundary location. Remove start and goal
            % from index computation as they are fixed nodes.
            numPose = size(pth,1);
            obj.DeltaTXIdxStart = (numPose - 2) * obj.PoseNodeDim + 1;

            % Number of elements in the state vector are 3*num of pose + num of deltaT. Assuming poses are SE2.
            % (numPose-2) because start and goal are fixed
            % (numPose-1) because num of deltaT will always be 1 less than num of pose
            obj.Xdim = (numPose-2) * obj.PoseNodeDim + (numPose - 1);

            % For a path with 10 poses, there will be 9 non-holonomic edges, 9
            % rho edges, 9 velocity edges, and 8 acceleration edge.
            % Acceleration edge is padded to enable vectorization of jacobian
            % and error computation, hence it's considered same as other motion
            % edges.
            obj.NumMotionEdges = numPose - 1;

            obj.ObstEdges = [];
            if size(obj.ObstPos, 1)
                % Initialize with inf to ensure obstacles not in considerations
                % get infinity value.
                distMat = inf(size(pth,1), size(obj.ObstPos,1)); % NumPose x NumObst
                % Compute "Obst Edges" by computing distance in a vectorized form.
                % Only compute distance for movable nodes, rest all would
                % be inf so that they are "cutoff"
                distMat(2:end-1, :) = poseToObstacleDistance(obj, pth(2:end-1, :));
                % Criteria to make sure obstacle has effect on poses close to it.
                inclusionDist = obj.Params.ObstacleInclusionDistance;
                % Criteria to make obstacle out of consideration for the current path.
                cutoffDist = obj.Params.ObstacleCutOffDistance;

                % Figure out subscripts for close obstacles
                % Get their index first to keep them out of contention while
                % figuring out closes obstacle on left and right in the middle
                % zone.
                closeIdx = distMat < inclusionDist;

                % Figure out subscripts for far obstacles
                farIdx = distMat > cutoffDist;

                % Set the close and far dist to inf to remove from further
                % processing of figuring out closest obstacle on left and right
                % in the middle zone
                disableIdx = or(closeIdx, farIdx);
                remainingDist = distMat;
                remainingDist(disableIdx) = inf;

                % Find closest left and closest right obstacles for each edge
                % Ignore first and last pose as they are fixed, so start from 2
                % and at end-1
                % Calculate d_k, the vector from current pose to next pose
                % to be used for cross product further down the line.
                delX = pth(2:end,1) - pth(1:end-1,1);
                delY = pth(2:end,2) - pth(1:end-1,2);
                % Iterate over poses
                middleIdx = false(size(distMat));
                leftObstIds = false(1, size(middleIdx,2));
                rightObstIds = false(1, size(middleIdx,2));
                for i=2:(size(pth,1)-1)
                    % Get distance to obstacles for current pose
                    obstDist = remainingDist(i,:);
                    % Find indices of obstacles in the "middle zone", not too
                    % far and not too close.
                    remainingObstIdx = obstDist~=inf;
                    % Move on to the next pose in no obstacle in the middle zone
                    if isempty(remainingObstIdx)
                        continue;
                    end

                    % Compute sin(theta) between "pose to obstacle" vector and
                    % "pose to next pose" vector (d_k).
                    % Using cross product equation for third component.
                    % Obstacles on left will have positive sin(theta) and right ones
                    % will have negative. Straight ahead ones (sin(theta)=0) are
                    % considered as left.
                    obstPos = obj.ObstPos(remainingObstIdx,1:2);
                    pose2obst = obstPos - ...
                        repmat(pth(i,1:2), size(obstPos,1), 1); % repmat for codegen
                    % sin(theta) = 3rd element of d_k x pose2Obst
                    pxo = delX(i) .* pose2obst(:,2) - delY(i) .* pose2obst(:,1);

                    % Add the closest obstacle for this pose on left and right
                    % to the ObstEdges list.

                    % Get the obstacles on left with 0+ sin(theta) in the
                    % remaining obstacles in middle zone.
                    leftObstIds(:) = false; % Wipe the slate clear.
                    leftObstIds(remainingObstIdx==1) = (pxo >= 0); %Assign values at right indices
                    if any(leftObstIds)
                        % If any obstacles on left then find the closest one.
                        [~, minId] = min(obstDist(leftObstIds));
                        % Add to edge list
                        idx =  find(leftObstIds);
                        middleIdx(i,idx(minId)) = true;
                    end

                    % Get the obstacles on right with negative sin(theta) in the
                    % remaining obstacles in middle zone.
                    % rightObstIds = remainingObstIdx(pxo < 0);
                    rightObstIds(:) = false; % Wipe the slate clear.
                    rightObstIds(remainingObstIdx==1) = (pxo < 0);  %Assign values at right indices
                    if any(rightObstIds)
                        % If any obstacles on right then find the closest one.
                        [~, minId] = min(obstDist(rightObstIds));
                        % Add to edge list
                        idx =  find(rightObstIds);
                        middleIdx(i,idx(minId)) = true;
                    end
                end
                % Extract the pose and obst id for making the edge.
                [poseId, obstId] = find(or(closeIdx,middleIdx));
                % Save the edge list for use in computation for errors for
                % obstacle edge constraints.
                obj.ObstEdges = [poseId, obstId];
            end

            % Build the diagonal matrix for Weight, this allows for H=J'WJ.
            % W is a diagonal matrix with 11*N + M number of rows and columns.
            % N is the number of motion edges which is one less than number of
            % poses, and M is number of obstacle edges.
            % Placement of weights is such that all the same type of weights
            % are together. Weights for types of constraints come one after
            % other in the order as seen in function call to blkdiag.
            N = obj.NumMotionEdges;
            M = size(obj.ObstEdges, 1);
            Wtime = obj.Params.WeightTime * eye(N);
            Wmotion = obj.Params.WeightSmoothness * eye(N);
            Wrho = obj.Params.WeightMinTurningRadius * eye(N);
            Wvel = obj.Params.WeightVelocity * eye(N);
            Wangvel = obj.Params.WeightAngularVelocity * eye(N);
            Waccel = obj.Params.WeightAcceleration * eye(N);
            Wangaccel = obj.Params.WeightAngularAcceleration * eye(N);
            Wobst = obstWeightMultiplier * obj.Params.WeightObstacles * eye(M);
            % 7*N for Wtime to Wangaccel and M for Wobst.
            coder.internal.errorIf(7*N+M > intmax, 'nav:navalgs:optimizepath:NumPoseTooLarge');
            W = blkdiag(Wtime, Wmotion, Wrho, Wvel, Wangvel, ...
                Waccel, Wangaccel, Waccel, Wangaccel, Waccel, Wangaccel, Wobst);
            obj.Weight = sparse(W);
        end

    end

    methods (Static)

        function [en, ev] = error(~, obj)
        %error Output the stored edge errors and weighted errors computed in last
        %    call to cost

            % In input "pth" and "deltaT" have 1 more row at the end, hence sizes
            % would for both would be +1 of original
            ev = obj.Err;
            wev = obj.Weight * ev;
            en = sqrt(full(sum(wev.*wev)));
        end

        function [c, W, J, obj] = cost(X, obj)
        %cost Calculates the weighted squared-error and jacobian for the
        %     time cost and constraints.
        %
        %   [c, W, J, obj] = cost(X, obj) computes errors and jacobian for
        %   the graph conceptually described by information in obj. c is
        %   computed as weighted squared-error and is a scalar. W is a
        %   block diagonal matrix containing weights for all 12 types of
        %   constraints and is a square matrix of 11*NumMotionEdges +
        %   Obstacle Edges. J is Jacobian for the 12 types of constraints
        %   in the "graph" and is matrix with 11*NumMotionEdges +
        %   NumObstacleEdges rows and number of elements in X as column.

            % rearrange the state vector(X) to be useful for vectorized
            % computation of metric for constraints, like velocity
            ip = x2input(obj, X);

            % Get all errors related to motion
            % (:,1) - time
            % (:,2) - non-holonomic motion
            % (:,3) - rho
            % (:,4) - linear velocity
            % (:,5) - angular velocity
            % (:,6) - linear accel
            % (:,7) - angular accel
            emotion = combinedMotionError(obj, ip);

            % Save the errors for reuse in error and gradient. This should be a
            % column vector with each constraint vertically stacked.
            obj.Err = emotion(:);

            % Append obstacle error only if there are obstacles
            anyObstacles = ~isempty(obj.ObstEdges);
            pth = [];
            if anyObstacles
                % Start should be the first row of the first two columns
                pth = [ip(:,1:2); obj.Goal(1, 1:2)];
                eobst = obstacleError(obj, pth); % Mx1
                fullErr = [obj.Err; eobst(:)];
                obj.Err = fullErr(:);
            end

            % Cost computation
            c = full(obj.Err' * obj.Weight * obj.Err);

            % Computation of Jacobian is expensive, do it only if it's asked for.
            if nargout > 1
                % Output the weight matrix computed while building the "graph"
                W = obj.Weight;

                % First calculate Jacobian for motion edges
                Jmotion = combinedMotionJacobian(obj, ip);
                J = Jmotion;
                % Append obstacle jacobian only if there are any
                if anyObstacles
                    Jobst = obstacleJacobian(obj, pth);
                    J = [J; Jobst];
                end

                % Save it for reuse in gradient and hessian.
                obj.Jacobian = J;
            end
        end

        function g = gradient(~, obj)
        %gradient Output the gradient (J'We) from the stored edge errors and
        %         Jacobian from last call to cost
            we = obj.Weight * obj.Err;
            grad = obj.Jacobian' * we;
            g = full(grad);
        end

        function H = hessian(~, obj)
        %hessian Output the Hessian (J'WJ) from the stored Jacobian from last call
        %        to cost
            H = obj.Jacobian' * obj.Weight * obj.Jacobian;
        end

    end

    methods (Access = private)

        function d = poseToObstacleDistance(obj, pth)
        %poseToObstacleDistance Calculates the distance of each row in pth with
        %each row in obj.ObstPos in a vectorized form.
            pthPos = pth(:, 1:2);
            obstPos = obj.ObstPos;

            % d² = ||x||² + ||y||² - 2xyᵀ
            x2 = sum(pthPos.^2, 2);
            y2 = sum(obstPos.^2, 2);
            xty2 = 2 .* pthPos * obstPos';
            d = sqrt(repmat(x2, 1, size(xty2, 2)) + repmat(y2', size(xty2, 1), 1) - xty2);
        end

        function e = combinedMotionError(obj, inp)
        %combinedMotionError calls functions for common and individual error
        %constraints and collects them in a matrix

            % Compute metrics(measurement) for motion related constraints,
            % these values will be used to compute the errors. Quantities
            % like Rho, Velocity, acceleration etc. are stored for use in
            % the corresponding error computation function, like rhoError.
            motionMetrics(obj, inp);

            % Compute individual errors.
            eTime = timeError(obj);
            eNHMotion = nonHolonomicMotionError(obj);
            eRho = rhoError(obj);
            eVel = velocityError(obj); % NumMotionEdge x 2
            eAccel = accelerationError(obj); % NumMotionEdge x 2
            eAccelS = sparse(obj.NumMotionEdges, 2);
            eAccelS(1,:) = accelerationStartError(obj);
            eAccelE = sparse(obj.NumMotionEdges, 2);
            eAccelE(end,:) = accelerationEndError(obj);

            % 9*NumMotionEdge x 1
            e = [eTime eNHMotion eRho eVel eAccel eAccelS eAccelE];
        end

        function e = obstacleError(obj, pth)
        %obstacleError calculates the safety margin error for all obstacle edges.

            % This is also a soft constraint, from obstEdges it finds the
            % relationship between the obstacles and poses, if an edge
            % exists it computes the distance between the pair. Error is
            % zero if the distance is greater than "ObstacleSafetyMargin"
            % specified by the user. If the distance is less than the
            % safety margin then the difference between value and margin is
            % the  error

            % Get poses of obstacles which are "active" have an influence on
            % the path. Active obstacles are the ones which lie in the
            % close or the middle zone.
            activePthIdx = obj.ObstEdges(:,1);
            activePthPos = pth(activePthIdx, 1:2);
            activeObstIdx = obj.ObstEdges(:,2);
            activeObstPos = obj.ObstPos(activeObstIdx, :);

            % activePathPos and activeObstPos would be of the same
            % size(=NumObstEdges) as the index repeat in ObstEdges, hence
            % the position also repeats.
            d = vecnorm(activePthPos - activeObstPos, 2, 2);
            safetyMargin = obj.Params.ObstacleSafetyMargin;
            e = safetyMargin - min(safetyMargin, d);
        end

        function J = combinedMotionJacobian(obj, inp)
        %combinedMotionJacobian calculates 'middle' numerical Jacobian for all
        %motion related constraints(i.e. no obstacles).

            %1. It iterates over the maximum input elements for any given
            %   edge elements can have. It uses the vectorized error
            %   computation function `combinedMotionError` to compute
            %   errors for forward and backwards step.
            %2. Once the jacobians are calculated we need to rearrange it
            %   such that errors and jacobians are getting added. For e.g.
            %   Node 2 is part of edge 1 and edge 2, so values
            %   corresponding to Node 2 are in first and second row. we
            %   shift and reshape the jacobian s.t. while multiplication
            %   with Weight matrix they get summed up. See,
            %   `arrangeJacobians` for more details.

            % Shorthand for small step to take in input forwards and
            % backwards
            h = obj.NumJacStep;

            % Allocate and Initialize Jacobian for each motion constraint.
            % Using sparse matrices as most of the values, especially in
            % later iteration would be zeros. Each row corresponds to one
            % edge in the graph, each column represents the input for the
            % edge. So, in essence number of rows = number of outputs, and
            % number of columns = number of inputs.
            timeJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            nhmotionJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            rhoJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            velJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            angvelJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            accelJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            angaccelJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            accelSJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            angaccelSJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            accelEJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);
            angaccelEJ = sparse(obj.NumMotionEdges, obj.MaxMotionInput);

            % Iterate over the max input elements any edge can have, that's
            % acceleration for us. Acceleration edge connects three pose nodes
            % and 2 deltaT nodes. Assuming SE2 pose, 3 pose nodes is 9 and 2 deltaT
            % nodes is 2, so total 11 input elements.
            % inputs: [x1 y1 yaw1 x2 y2 yaw2 x3 y3 yaw3 deltaT1 deltaT2]
            % time edges are only impacted by deltaT1
            % non-holonomic motion, Rho, Linear and Angular velocities are only
            % impacted by [x1 y1 yaw1,x2 y2 yaw2, deltaT1]
            % Linear and Angular accelerations are impacted by all 11
            % Linear and Angular accelerations for start and end are only
            % impacted by elements impacting velocities.
            for elemIdx = 1:obj.MaxMotionInput
                % Take a step forward and calculate error
                inpF = inp;
                inpF(:,elemIdx) = inpF(:,elemIdx) + h;
                emotionF = combinedMotionError(obj, inpF);

                % Take a step back and calculate error
                inpB = inp;
                inpB(:,elemIdx) = inpB(:,elemIdx) - h;
                emotionB = combinedMotionError(obj, inpB);

                % jacobian = dely/delx
                jac = (emotionF-emotionB)./(2*h);

                % Split the combined jacobian into each constraint.
                timeJ(:,elemIdx) = jac(:,1); %#ok<*SPRIX>
                nhmotionJ(:,elemIdx) = jac(:,2);
                rhoJ(:,elemIdx) = jac(:,3);
                velJ(:,elemIdx) = jac(:,4);
                angvelJ(:,elemIdx) = jac(:,5);
                accelJ(:,elemIdx) = jac(:,6);
                angaccelJ(:,elemIdx) = jac(:,7);
                accelSJ(:,elemIdx) = jac(:,8);
                angaccelSJ(:,elemIdx) = jac(:,9);
                accelEJ(:,elemIdx) = jac(:,10);
                angaccelEJ(:,elemIdx) = jac(:,11);
            end

            % Rearrange the jacobian to get [NumMotionEdges x Xdim] matrix.
            % Roughly it changes columns to diagonals, it also shifts them to
            % ensure values impact the right elements in stateVector.
            jacTime = arrangeJacobians(obj, timeJ);
            jacNHMotion = arrangeJacobians(obj, nhmotionJ);
            jacRho = arrangeJacobians(obj, rhoJ);
            jacVel = arrangeJacobians(obj, velJ);
            jacAngVel = arrangeJacobians(obj, angvelJ);
            jacAccel = arrangeJacobians(obj, accelJ);
            jacAngAccel = arrangeJacobians(obj, angaccelJ);

            % Rearrange jacobian for accel start and accel end separately.
            % Accel start edge only affects second pose and first deltaT and
            % Accel end only affects penultimate pose and last deltaT.
            xdim = obj.Xdim;

            % Allocate a matrix for acceleration constraint at start with same
            % "size" as other motion constraints for easy clubbing.
            jacAccelS = sparse(obj.NumMotionEdges, xdim);
            % Impact on second pose, hence row 1. Only x2,y2,yaw2 impact it, hence
            % column 4:6 are extracted
            jacAccelS(1, 1:3) = accelSJ(1, 4:6);
            % Impact on 1st deltaT, hence row 1. Only deltaT1 impacts it, hence
            % column 10 is extracted
            jacAccelS(1, obj.DeltaTXIdxStart) = accelSJ(1, 10);

            % Same for Angular acceleration at start
            jacAngAccelS = sparse(obj.NumMotionEdges, xdim);
            jacAngAccelS(1, 1:3) = angaccelSJ(1, 4:6);
            jacAngAccelS(1, obj.DeltaTXIdxStart) = angaccelSJ(1, 10);

            % Calculate index positions for penultimate pose, in graph it's
            % penultimate but in state it's the last three in pose section,
            % which is just before the deltaT section. Assuming SE2 pose, we
            % subtract 3(PoseNodeDim) from deltaT start index to get start of
            % penultimate pose index and end index is the one just before(-1)
            % deltaT starts.
            n1PoseRange = obj.DeltaTXIdxStart-obj.PoseNodeDim : obj.DeltaTXIdxStart-1;
            % Allocate a matrix for acceleration constraint at end with same
            % "size" as other motion constraints for easy clubbing.
            jacAccelE = sparse(obj.NumMotionEdges, xdim);
            % Impact on second last pose, hence row end(last pose is fixed). Only
            % x1,y1,yaw1 impact it, hence column 1:3 are extracted
            jacAccelE(end, n1PoseRange) = accelEJ(end, 1:3);
            % Impact on last deltaT, hence row end. Only deltaT1 impacts it, hence
            % column 10 is extracted
            jacAccelE(end, end) = accelEJ(end, 10);

            % Same for Angular acceleration at end.
            jacAngAccelE = sparse(obj.NumMotionEdges, xdim);
            jacAngAccelE(end, n1PoseRange) = angaccelEJ(end, 1:3);
            jacAngAccelE(end, end) = angaccelEJ(end, 10);

            % Stack them vertically. This allows multiplication with Weight
            % matrix in the right way.
            J = [jacTime; jacNHMotion; jacRho; jacVel; jacAngVel; ...
                jacAccel; jacAngAccel; jacAccelS; jacAngAccelS; ...
                jacAccelE; jacAngAccelE];
        end

        function J = obstacleJacobian(obj, pth)
        %obstacleJacobian computes 'middle' numerical jacobian for all
        %obstacle edges.
            % Get x,y for all poses
            inp = pth(:,1:2);
            h = obj.NumJacStep;
            obstEdges = obj.ObstEdges; %convenience

            % Take a step forward and compute error. For x.
            inpF = inp;
            inpF(:,1) = inpF(:,1) + h;
            exF = obstacleError(obj, inpF);

            % Take a step back and compute error. For x.
            inpB = inp;
            inpB(:,1) = inpB(:,1) - h;
            exB = obstacleError(obj, inpB);

            % Take a step forward and compute error. For y.
            inpF = inp;
            inpF(:,2) = inpF(:,2) + h;
            eyF = obstacleError(obj, inpF);

            % Take a step back and compute error. For y.
            inpB = inp;
            inpB(:,2) = inpB(:,2) - h;
            eyB = obstacleError(obj, inpB);

            numObstEdge = size(obstEdges,1);
            % Jacobian for all edges as rows(output) and two
            % column(inputs:x,y).
            jac = sparse(numObstEdge, 2);
            jac(:,1) = (exF - exB) ./ (2*h);
            jac(:,2) = (eyF - eyB) ./ (2*h);

            xdim = obj.Xdim;
            % Rearrange jacobian s.t. H = J'WJ, for new format row is
            % dictated by Obstacle edge id and column is dictated by pose
            % index in state vector(xIdx).
            % xIdx for 2nd pose => (2-1)*3 - (3-1) == 1*3 - 2 == 1, as
            % first pose is fixed 2nd would have 1st index. For first pose
            % the value is negative (as expected), because pose index 1
            % shouldn't be in edge list as obj.build won't add edges for
            % fixed nodes.
            xIdx = (obstEdges(:,1) - 1)*obj.PoseNodeDim - (obj.PoseNodeDim - 1);
            r = [(1:numObstEdge)'; (1:numObstEdge)']; % [Obst Indices; Obst Indices], row num
            c = [xIdx; xIdx+1]; % [X; Y] indices, col num
            % As most obstacles would be outside of safety margin sparse is a
            % good option for it.
            J = sparse(r, c, jac(:), numObstEdge, xdim);
        end

        function motionMetrics(obj, inp)
            %motionMetrics calculates the values used for motion related
            %errors in vectorized form

            % All calculations are done in one function as each constraint term
            % builds on the previous one. Accel uses velocity, velocity uses
            % Rho uses delBeta and dkNorm for computation.
            % Splitting it up leads to lot of data passing and/or re-computation.

            % Give input columns names. In paper terminology s_k = [x1, y1,
            % yaw1], s_k+1 = [x2, y2, yaw2], s_k+2 = [x3, y3, yaw3], deltaT_k =
            % deltaT1, and deltaT_k+1 = deltaT2
            x1 = inp(:,1);
            y1 = inp(:,2);
            yaw1 = inp(:,3);
            x2 = inp(:,4);
            y2 = inp(:,5);
            yaw2 = inp(:,6);
            x3 = inp(:,7);
            y3 = inp(:,8);
            yaw3 = inp(:,9);
            deltaT1 = inp(:,10);
            deltaT2 = inp(:,11);

            % Pre-compute for Non-Holonomic motion and velocity
            cosBeta1 = cos(yaw1);
            sinBeta1 = sin(yaw1);
            cosBeta2 = cos(yaw2);
            sinBeta2 = sin(yaw2);

            % Pre-computation for Non-Holonomic motion, Rho, and Velocity
            delX21 = x2 - x1;
            delY21 = y2 - y1;
            delX32 = x3 - x2;
            delY32 = y3 - y2;
            % Pre-compute for Rho and Velocity
            % To avoid delBeta = 0 condition(straight path), add epsilon.
            delBeta21 = (yaw2 - yaw1) + 2 * sqrt(eps);
            delBeta32 = (yaw3 - yaw2) + 2 * sqrt(eps);

            % Pre-compute for Rho and Velocity
            dkNorm21 = sqrt(delX21.^2 + delY21.^2);
            dkNorm32 = sqrt(delX32.^2 + delY32.^2);

            % Optimal time constraint
            obj.DeltaT1 = deltaT1;

            % Non-Holonomic motion calculations
            obj.NhMotion = abs(delY21 .* (cosBeta1 + cosBeta2) ...
                - delX21 .* (sinBeta1 + sinBeta2)); % eq (4)

            % Turning radius calculations
            obj.Rho21 = abs(dkNorm21 ./ (2 * sin(delBeta21 ./ 2))); % eq (5)
            rho32 = abs(dkNorm32 ./ (2 * sin(delBeta32 ./ 2)));

            % Velocity calculations.
            % If editing this, edit lines below for computation of vel32
            d = abs(obj.Rho21 .* delBeta21);
            kappa = 100;
            dotDkQk = delX21 .* cosBeta1 ...
                + delY21 .* sinBeta1;
            numerator = kappa .* dotDkQk;
            gamma = numerator./(1 + abs(numerator)); % eq (8)

            % Compute for Velocity and pre-compute for acceleration
            obj.Vel21 = zeros(numel(deltaT1), 2);
            obj.Vel21(:,1) = gamma .* (d./deltaT1); % Linear Velocity % eq(7)
            obj.Vel21(:,2) = delBeta21./deltaT1; % Angular Velocity

            % acceleration calculations for s_K+1 and s_k+2. Ideally we want to
            % shift Vel21 and save this computation, but this is required for
            % perturbation of x3,y3,yaw3 to compute jacobian w.r.t s_k+2
            % If editing this, edit lines above for computation of Vel21
            d = abs(rho32 .* delBeta32);
            kappa = 100;
            dotDkQk = delX32 .* cosBeta2 ...
                + delY32 .* sinBeta2;
            numerator = kappa .* dotDkQk;
            gamma = numerator./(1 + abs(numerator));

            vel32 = zeros(numel(deltaT2), 2);
            vel32(:,1) = gamma .* (d./deltaT2);
            vel32(:,2) = delBeta32./deltaT2;

            obj.Accel = zeros(numel(deltaT2), 2);
            deltaT12 = deltaT1 + deltaT2;
            obj.Accel(:,1) = 2*(vel32(:,1) - obj.Vel21(:,1))./deltaT12; % Linear Acceleration % eq (9)
            obj.Accel(:,2) = 2*(vel32(:,2) - obj.Vel21(:,2))./deltaT12; % Angular Acceleration

            % Formula is (v_1 - vel_start)/deltaT_1, but since velStart is
            % assumed to be 0 we directly compute v_1/deltaT_1
            obj.StartAccel = obj.Vel21(1,:)./deltaT1(1);

            % Formula is (vel_end - v_(n-1))/deltaT_(n-1), but since velEnd is
            % assumed to be 0 we directly compute -v_(n-1)/deltaT_(n-1)
            obj.EndAccel = -obj.Vel21(end,:)/deltaT1(end);

        end

        function e = timeError(obj)
        %timeError returns the error for the time component of cost
        %function.

            % Travel time, i.e. deltaT is being minimized, without any
            % constraints such as velocity the ideal value for optimized
            % path would be to traverse it in 0s. So, error = deltaT - 0,
            % i.e. error = deltaT
            e = obj.DeltaT1;
        end

        function e = nonHolonomicMotionError(obj)
        %nonHolonomicMotionError calculates the non-holonomic motion error
        %for path

            % This is a soft constraint, but for this the error is computed
            % from zero as well, which implies that ideally incoming angle
            % difference and outgoing angle difference are the same, which
            % is the case for a circle, which is what we want. What's angle
            % difference? best to read the paper[2] and focus on Fig. 1 and
            % equation 3 and 4.
            e = obj.NhMotion;
        end

        function e = rhoError(obj)
        %rhoError calculates the error for min turning radius constraint

            % This is a soft-constraint, the ideal value is determined by
            % the user provided parameter "MinTurningRadius". Any value
            % above the min value is acceptable, hence the error is zero,
            % as soon as the turning radius goes below the min, the
            % difference between value and min is taken as error.
            rhoMin = obj.Params.MinTurningRadius;
            e = rhoMin - min(rhoMin, obj.Rho21);
        end

        function e = velocityError(obj)
        %velocityError calculates the error for velocity constraint

            % This is a soft-constraint, the ideal value is determined by
            % the user provided parameter "MaxVelocity" and
            % "MaxAngularVelocity". Any value below the reference
            % value is acceptable, hence the error is zero, as soon as the
            % velocity or angular velocity goes above the reference, the
            % difference between value and reference is taken as error.
            e = zeros(obj.NumMotionEdges, 2);
            refVel = obj.Params.MaxVelocity;
            refAngVel = obj.Params.MaxAngularVelocity;
            e(:,1) = max(refVel, abs(obj.Vel21(:,1))) - refVel;
            e(:,2) = max(refAngVel, abs(obj.Vel21(:,2))) - refAngVel;
        end

        function e = accelerationError(obj)
        %accelerationError calculates the error for acceleration constraint

            % This is a soft-constraint, the ideal value is determined by
            % the user provided parameter "MaxAcceleration" and
            % "MaxAngularAcceleration". Any value below the reference
            % value is acceptable, hence the error is zero, as soon as the
            % acceleration or angular acceleration goes above the
            % reference, the difference between value and reference is
            % taken as error.
            e = zeros(obj.NumMotionEdges, 2);
            refAccel = obj.Params.MaxAcceleration;
            refAngAccel = obj.Params.MaxAngularAcceleration;
            e(:,1) = max(refAccel, abs(obj.Accel(:,1))) - refAccel;
            e(:,2) = max(refAngAccel, abs(obj.Accel(:,2))) - refAngAccel;
            e(end, :) = 0;
        end

        function e = accelerationStartError(obj)
        %accelerationStartError calculates the error for starting
        %acceleration constraint

            % This is a soft-constraint, the ideal value is determined by
            % the user provided parameter "MaxAcceleration" and
            % "MaxAngularAcceleration". Any value below the reference
            % value is acceptable, hence the error is zero, as soon as the
            % acceleration or angular acceleration goes above the
            % reference, the difference between value and reference is
            % taken as error.
            % This is different from accelerationError as this tries to
            % compute the acceleration assuming the start velocity as 0.
            % Acceleration is computed with 3 pose nodes, so first
            % acceleration edge considers pose1,pose2,pose3, it takes the
            % difference of velocities between pose1 to pose2 and pose2 to
            % pose 3. In acceleration start difference is taken between
            % velocity between pose1 and pose2 and a start velocity,
            % assumed to be 0.
            e = zeros(1,2);
            refAccel = obj.Params.MaxAcceleration;
            refAngAccel = obj.Params.MaxAngularAcceleration;
            e(1) = max(refAccel, abs(obj.StartAccel(:,1))) - refAccel;
            e(2) = max(refAngAccel, abs(obj.StartAccel(:,2))) - refAngAccel;
        end

        function e = accelerationEndError(obj)
        %accelerationEndError calculates the error for ending acceleration
        %constraint
            % Same as accel start, accept it's for end, that is velocity
            % between (n-1)th and (n)th pose is considered with an end
            % velocity, assumed to be 0.
            e = zeros(1,2);
            refAccel = obj.Params.MaxAcceleration;
            refAngAccel = obj.Params.MaxAngularAcceleration;
            e(1) = max(refAccel, abs(obj.EndAccel(:,1))) - refAccel;
            e(2) = max(refAngAccel, abs(obj.EndAccel(:,2))) - refAngAccel;
        end

        function J = arrangeJacobians(~, jac)
        %arrangeJacobians arranges motion jacobian to [NumMotionEdges x Xdim]
        %to enable J'WJ or J'We

        % Input jacobians are [NumEdges x MaxMotionInputs], we need to have
        % jacobians as [NumEdges x Xdim], as g = J'We needs to be [Xdim x
        % 1] and H = J'WJ need to [Xdim x  Xdim].
        % This rearranging enable summation of values corresponding to same
        % node from all it's edges and produce g and H of desired size.
        % For e.g. Third pose node(n2) is a part of acceleration edge
        % thrice, once for first edge(e1) as the terminal node and once for
        % second edge(e2) as middle node, and once for third edge(e3) as
        % starting node. Jacobian values corresponding to n1 from e1 are in
        % first row, from e2 are in second row, and from e3 in third row.
        %
        % We need to distribute jacobians horizontally so that they
        % are multiplied with the right element in error vector. For this
        % we convert column vectors in jacobians to diagonals in output
        % matrix.
        %
        % We need to also shift jacobians for e2 and e3 to bring it to same
        % level as e1, this leads to all of them outputted to indices
        % corresponding to n2 in g and H. For this while diagonalization we
        % make the elements for e1 as main diagonal, e2 as 1 shifted
        % diagonal, and e3 as 2 shifted diagonal.
        % NB: If a = [N-by-1], diag(a) is NxN, diag(a,1) is N+1 x N+1,
        % similarly for diag(a,2) is N+2 x N+2. To allow addition of these
        % three matrices we pad a accordingly, s.t. diag([a;0;0]) is N+2 x
        % N+2 while a being the main diagonal, diag([a;0],1) is N+2 x N+2.
        % After addition we remove the 2 padded rows.

            % Jacobian matrix for x* Pad jacobian values for x1 to create a
            % compatible size matrix for
            % addition after diagonalization. x1 is going to be main diagonal
            % Reshape so codegen recognizes T1 and T2 as column vector
            T1 = reshape([jac(:,1); 0; 0], [], 1); % x1
            T2 = reshape([jac(:,4); 0], [], 1); % x2
            T3 = jac(:,7); % x3
            % Main diagonal + 1-shifted diagonal + 2-shifted diagonal
            M = diag(T1) + diag(T2, 1) + diag(T3, 2); % x1 + x2 + x3

            % Jacobian matrix for y*. Similar technique of shifting and
            % diagonalization as x*
            % disp('---------------------N-------------------')
            T1 = reshape([jac(:,2); 0; 0], [], 1); % y1
            T2 = reshape([jac(:,5); 0], [], 1); % y2
            T3 = jac(:,8); % y3
            N = diag(T1) + diag(T2, 1) + diag(T3, 2); % y1 + y2 + y3

            % Jacobian matrix for yaw*. Similar technique of shifting and
            % diagonalization as x*
            T1 = reshape([jac(:,3); 0; 0], [], 1); % yaw1
            T2 = reshape([jac(:,6); 0], [], 1); % yaw2
            T3 = jac(:,9); % yaw3
            O = diag(T1) + diag(T2, 1) + diag(T3, 2); % yaw1 + yaw2 + yaw3

            % Jacobian matrix for deltaT*. Similar technique of shifting and
            % diagonalization as x*
            T2 = reshape([jac(:,10); 0], [], 1); % deltaT
            T3 = jac(:,11); % deltaT1
            P = diag(T2) + diag(T3, 1);  % deltaT + deltaT1

            % Make the rearranged matrices sparse.
            [M, N, O, P] = deal(sparse(M), sparse(N), sparse(O), sparse(P));

            % Remove last two rows. There rows correspond to the padded rows above in T1 T2.
            M = M(1:end-2,:);
            N = N(1:end-2,:);
            O = O(1:end-2,:);
            P = P(1:end-1,:);

            % E.g.
            % path: [x1,x2,x3,x4]
            % x5 is dummy
            % [x1,x2,x3]
            % [x2,x3,x4]
            % [x3,x4,x5]
            % We want jacobian only for [x2,x3,x4]

            % Remove 1st & last two elements. Last column corresponds to dummy
            % state. First and penultimate columns are for the fixed nodes
            % (first and last pose in path).
            M = M(:, 2:end-2);
            N = N(:, 2:end-2);
            O = O(:, 2:end-2);
            P = P(:, 1:end-1);

            % If we do [M,N,O] we get the following ordering
            % [x1,x2,x3,y1,y2,y3,yaw1,yaw2,yaw3]
            % Interleave matrices to obtain the desired ordering
            % [x1,y1,yaw1,x2,y2,yaw2,x3,y3,yaw3]
            M_ = M';
            N_ = N';
            O_ = O';
            J = reshape([M_(:) N_(:) O_(:)]',3*size(M_,1), [])';
            J = [J, P];

        end

        function input = x2input(obj, X)
            %x2input converts X of solver into input for error and Jacobian
            %calculation

            % In input "pth" and "deltaT" have 1 more row at the end, hence sizes
            % would for both would be +1 of original. This is done to include
            % acceleration in vectorization.
            pth = x2path(obj, X);
            deltaT = x2DeltaT(obj, X);
            obj.Path = pth;
            obj.DeltaT = deltaT;

            % Add dummy values at the end to enable vectorization of
            % Acceleration constraints
            pth = [pth; pth(end,:)];
            deltaT = [deltaT; deltaT(end)];

            % Will have MaxEdgeElems(11) column
            input = [pth(1:end-2, :) pth(2:end-1, :) pth(3:end, :) ...
                deltaT(1:end-1) deltaT(2:end)];
        end

    end

    methods

        function value = get.Velocity(obj)
            value = obj.Vel21;
        end

        function value = get.Acceleration(obj)
            value = [obj.StartAccel; obj.Accel(1:end-1, :); obj.EndAccel];
        end

        function pth = x2path(obj, X, poseEndXIdx)
            %x2path splits X of solver to path nodes

            if nargin <= 2
                poseEndXIdx = obj.DeltaTXIdxStart-1;
            end
            % As first and last nodes are fixed they don't get included in x,
            % hence include them in the path.
            pth = [obj.Start;...
                reshape(X(1:poseEndXIdx), 3, [])';...
                obj.Goal];
        end

        function deltaT = x2DeltaT(obj, X)
            %x2DeltaT splits X of solver to deltaT nodes

            deltaT = X(obj.DeltaTXIdxStart:end);
        end

    end

end
