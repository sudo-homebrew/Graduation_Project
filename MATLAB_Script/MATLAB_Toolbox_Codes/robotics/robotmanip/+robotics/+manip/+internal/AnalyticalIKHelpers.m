classdef AnalyticalIKHelpers < robotics.manip.internal.InternalAccess
    % This class is for internal use only and may be removed in a future release
    
    %AnalyticalIKHelpers Collection of utility methods for use with analytical IK
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    methods (Static)
        
        function tree = buildNewChainFromKinematicGroup(treeInternal, kinPath)
            %buildNewChainFromKinematicGroup Build rigidBodyTree from a vector of body indices
            %   This method builds a chain from a vector of body indices
            %   that define a kinematic path. The kinematic path represents
            %   the shortest path from one body to another; hence it
            %   contains the joints that would change position in an
            %   inverse kinematics call. This function builds a new rigid
            %   body tree from those indices, which can subsequently be
            %   used to generate inverse kinematics solutions.
            
            % reorder affected body list based on existing indices
            newRobot = robotics.manip.internal.RigidBodyTree(numel(kinPath), 'column');
            
            % Get the name of the first body in the chain and
            % assign it to the base
            pid = kinPath(1);
            if pid > 0
                parent = treeInternal.Bodies{pid};
            else
                parent = treeInternal.Base;
            end
            newRobot.BaseName = parent.Name;
            
            for i = 2:numel(kinPath)
                % For each element, get the ID of the parent (pid) and the
                % ID of the body itself (bid)
                pid = kinPath(i-1);
                bid = kinPath(i);
                if pid > 0
                    parent = treeInternal.Bodies{pid};
                else
                    parent = treeInternal.Base;
                end
                newRobot.addBody(treeInternal.Bodies{bid}, parent.Name);
            end
            
            % Get rigidBodyTree from treeInternal
            tree = rigidBodyTree(newRobot);
        end
        
        function solDetails = identifyRobotClassification(tree, diffTolerance)
            %identifyRobotClassification Identify the kinematic structure of the robot for IK
            %   Given a rigidBodyTree, this method determines the
            %   high-level classification. That information can then be
            %   used by other analysis tools to determine whether we can
            %   compute a solution for the rigidBodyTree. Here it is
            %   assumed that the input tree is a serial chain with no
            %   branches or leaf bodies that has already been validated by
            %   a user-facing function. The method outputs a structure
            %   containing classification details. The structure has the
            %   following fields:
            %      IsJointRevolute                - A 1xN vector indicating
            %                                       whether the ith joint
            %                                       is revolute
            %
            %      IsIntersectingAxesMidpoint     - A 1xN vector indicating
            %                                       whether the ith joint
            %                                       is the midpoint of a
            %                                       set of three
            %                                       intersecting axes
            %
            %      MidpointAxisIntersections      - A 2x3xN matrix that
            %                                       stores the measured
            %                                       joint intersections.
            %                                       For the ith joint
            %                                       (specified in the 3rd
            %                                       dimension), this
            %                                       property returns a 2x3
            %                                       matrix where the first
            %                                       row is the intersection
            %                                       of the previous axis
            %                                       with the current joint
            %                                       axis, and the second
            %                                       row is the intersection
            %                                       of the current joint
            %                                       axis with the next
            %                                       joint axis. All
            %                                       intersections are in
            %                                       the ith joint frame.
            %                                       Axes that don't
            %                                       intersect return NaNs.
            
            import robotics.manip.internal.AnalyticalIKHelpers
            
            % Degrees of freedom
            nDoF = tree.TreeInternal.VelocityNumber;
            
            % Joint type is an array of logicals indicating whether the
            % joint is revolute. It is indexed by the bodyIndex.
            isBodyJointRevolute = cellfun(@(body)(strcmpi(body.Joint.Type, 'revolute')), tree.Bodies);
            
            % Define identification tolerance
            if nargin < 2
                diffTolerance = sqrt(eps);
            end
            
            % Initialize output struct
            solDetails = struct();
            solDetails.IsJointRevolute = false(1,nDoF);
            solDetails.IsIntersectingAxesMidpoint = false(1,nDoF);
            solDetails.MidpointAxisIntersections = NaN(2,3,nDoF);
            
            % Initialize a vector to track the moving joint type
            isMovingJointRevolute = zeros(1,nDoF);
            
            % Move through all the non-fixed joints looking at all
            % consecutive sets of three to see if (1) they are all revolute
            % and (2) their axes intersect. Use the center joint in every
            % set of three ("currJtIdx") as the anchor / index.
            for currJointIdx = 2:nDoF-1
                prevJointIdx = currJointIdx-1;
                nextJointIdx = currJointIdx+1;
                
                % Get the bodies associated with the three consecutive
                % non-fixed joints
                prevJointBodyIdx = AnalyticalIKHelpers.getBodyIndexFromJointIndex(tree, prevJointIdx);
                currJointBodyIdx = AnalyticalIKHelpers.getBodyIndexFromJointIndex(tree, currJointIdx);
                nextJointBodyIdx = AnalyticalIKHelpers.getBodyIndexFromJointIndex(tree, nextJointIdx);
                
                % Check the joint types and return if any of the joints is
                % not revolute
                isMovingJointRevolute([prevJointIdx currJointIdx nextJointIdx]) = isBodyJointRevolute([prevJointBodyIdx currJointBodyIdx nextJointBodyIdx]);
                if any(~isBodyJointRevolute([prevJointBodyIdx currJointBodyIdx nextJointBodyIdx]))
                    % Only care about intersections of revolute joint axes
                    solDetails.IntersectionPoints(currJointIdx) = NaN;
                    continue;
                end
                
                % Find intersection between axes in two frames
                [prevCurrHasIntersection, prevCurrIntersectPt] = AnalyticalIKHelpers.findJointAxesIntersectionPoint(currJointBodyIdx, prevJointBodyIdx, tree, diffTolerance);
                [nextCurrHasIntersection, nextCurrIntersectPt] = AnalyticalIKHelpers.findJointAxesIntersectionPoint(currJointBodyIdx, nextJointBodyIdx, tree, diffTolerance);
                
                % Check if both axes intersect at the same point, all in
                % the current body frame
                if prevCurrHasIntersection && nextCurrHasIntersection
                    % Check whether the intersection points are the same
                    isSharedIntersection = all(abs(prevCurrIntersectPt - nextCurrIntersectPt) < diffTolerance);
                else
                    isSharedIntersection = false;
                end
                
                % Log data in output struct
                solDetails.IsIntersectingAxesMidpoint(currJointIdx) = isSharedIntersection;
                solDetails.MidpointAxisIntersections(:,:,currJointIdx) = [prevCurrIntersectPt(:)'; nextCurrIntersectPt(:)'];
                solDetails.IsJointRevolute = isMovingJointRevolute;
                solDetails.JointPositionLimits = tree.TreeInternal.JointPositionLimits;
            end
        end
        
        function [isSupported, lastThreeAxes] = assessLastThreeJointsConfigurationForIKSupport(dhParams, tol)
            %assessLastThreeJointsConfigurationForIKSupport Get the last three joint axes from the DH parameters and check whether they are supported
            % Analyze the last three axes to get their direction
            % relative to the axis of joint 4. This is used for the
            % orientation computation; when these axes differ, Euler
            % angles are used to compute to joint axis. For that to
            % work, the last three joints must be rotated relative to
            % each other by angles that are multiples of 90 degrees,
            % and the last three axes must intersect. The latter
            % statement is assumed true, all trees passed to this
            % method are already known to have three intersecting axes
            % at the end.
            
            import robotics.manip.internal.AnalyticalIKHelpers
            
            % Build a kinematically compatible DH robot using the
            % derived parameters and use that to determine the relative
            % orientations of the last tree axes. Using a reference
            % robot helps to avoid transform complexity
            dhTree = robotics.manip.internal.robotplant.revoluteDHCompatibleRobot(dhParams);
            dhTreeTFTree = dhTree.TreeInternal.forwardKinematics(zeros(dhTree.TreeInternal.VelocityNumber, 1));
            
            % Get the orientation of the last three axes by multiplying
            % the local Z axes of each frame by the transform from
            % the joint 4 to that frame.
            lastThreeAxes = repmat([0 0 1], 3, 1); %Initialize to all Z, in line
            lastThreeAxes(2,:) = [0 0 1]*tform2rotm(AnalyticalIKHelpers.getJointTransform(dhTree, 4, 5, dhTreeTFTree));
            lastThreeAxes(3,:) = [0 0 1]*tform2rotm(AnalyticalIKHelpers.getJointTransform(dhTree, 4, 6, dhTreeTFTree));
            
            % Verify that the last values are all multiples of 90 degrees
            % and round the axes to zero or one
            for i = 1:numel(lastThreeAxes)
                if isEqualWithinTolerance(lastThreeAxes(i), 0, tol)
                    lastThreeAxes(i) = 0;
                elseif isEqualWithinTolerance(abs(lastThreeAxes(i)), 1, tol)
                    lastThreeAxes(i) = sign(lastThreeAxes(i));
                else
                    % If it's not 1 or zero, cannot solve
                    lastThreeAxes(i) = NaN;
                end
            end
            isSupported = ~any(any(isnan(lastThreeAxes)));
            
            % Only support the ZYZ format, which allows the position
            % and orientation to be completely decoupled in the
            % joint-limit-free case because the any orientation is
            % possible with the last three joints. ZZZ, ZZY, and ZYY
            % axes orientations are not supported.
            isSupported = isSupported && isEqualWithinTolerance(abs(lastThreeAxes), [0 0 1; 0 1 0; 0 0 1], tol);
            
            if ~isSupported
                lastThreeAxes = NaN(3,3);
            end
        end
        
        function bodyIndex = getBodyIndexFromJointIndex(tree, jointIndex)
            %getBodyIndexFromJointIndex Returns the body index associated with a specific joint
            %   Given the joint index (i.e., the index of the joint in the
            %   configuration vector), this returns the associated
            %   rigidBodyObject's index.
            
            
            % Map the joint index to the body index (e.g. bodies with fixed
            % joints are not included. The Position DoF Map maps the body
            % indices to joint index and behavior, where the ith row of the
            % map corresponds to the ith body, and the columns represent
            % the matching joint index. Values of 0 and -1 indicate a fixed
            % joint.
            bodyIndex = find(tree.TreeInternal.PositionDoFMap(:,2) == jointIndex, 1);
        end       
        
        function [filePath, fnName] = generateIKcode(solutionDetails, fileName, idTolerance, solTolerance)
            
            import robotics.manip.internal.AnalyticalIKHelpers
            
            % Get the DH Parameters & Z-screw data
            dhParams = solutionDetails.SolutionMethodDetails.DHParams;
            zscrews = solutionDetails.SolutionMethodDetails.ZScrews;

            % Find the translation of the end effector relative to
            % the joint where the last three axes intersect
            dhTree = robotics.manip.internal.robotplant.revoluteDHCompatibleRobot(dhParams);
            dhEEToJt5 = getTransform(dhTree, zeros(size(dhTree.homeConfiguration)), dhTree.BodyNames{5}, dhTree.BodyNames{end});
            [lastThreeAxesSign, lastThreeAxesSequenceStr, hasMultipleRotationSolns] = AnalyticalIKHelpers.getLastThreeAxesInfo(solutionDetails.SolutionMethodDetails.LastThreeAxes);

            % Define code parameters
            h1Line = 'Function for generating closed-form inverse kinematics solutions to the DH robot given by the parameters specified below';
            dimensionSpec = ['dhParams = ' mat2str(dhParams) ';'];
            offsetSpec = ['thetaOffsets = ' mat2str(zscrews(1:6,2)') ';'];
            jointLimSpec = ['jointLimits = ' mat2str(solutionDetails.ClassificationDetails.JointPositionLimits) ';'];
            jointTypeSpec = ['isJointRevolute = ' mat2str(solutionDetails.ClassificationDetails.IsJointRevolute) ';'];

            % Define key DH-dependent subroutines
            firstThreeJointsFcnSignature = 'solveFirstThreeDHJoints(jt5Pose(1:3,4), dhParams)';
            firstThreeJointsFcn = AnalyticalIKHelpers.generateFirstThreeJointsFcn('solveFirstThreeDHJoints', dhParams, idTolerance, solTolerance);
            joint4PoseFcn = AnalyticalIKHelpers.generateFirstThreeDHJointsPose(dhParams);
            
            % Define rotation matrix subroutine used to get last three joint angles 
            axesSignSpec = ['lastThreeAxesSign = ' mat2str(lastThreeAxesSign) ';'];
            rotationMatrixToAxisAnglesFcnName = sprintf('convertRotationTo%sAxesAngles', lastThreeAxesSequenceStr);
            
            % Define subroutine to compute equality within a set tolerance
            isEqualWithToleranceFcn = AnalyticalIKHelpers.generateIsEqualWithinToleranceFcn(solTolerance);
            
            % Get the function name from the filename
            [filePath, fnName, ~] = fileparts(fileName);
            
            % Only support generation to a local directory
            if ~isempty(filePath)
                fileInputArgName = 'functionName';
                robotics.manip.internal.error('rigidbodytree:OnlyLocalFilesSupported', fileInputArgName);
            end
            
            % Create file using MATLAB Function generator
            ikFcn = sigutils.internal.emission.MatlabFunctionGenerator( fnName, {'eeTform', 'enforceJointLimits', 'sortByDistance', 'referenceConfig'}, 'qOpts' );
            ikFcn.EndOfFileMarker = false;
            
            % Specify file location
            if ~isempty(filePath)
                ikFcn.Path = filePath;
            end
            
            % Add auto-generated subroutines as local functions
            ikFcn.addLocalFunction(firstThreeJointsFcn);
            ikFcn.addLocalFunction(joint4PoseFcn);
            ikFcn.addLocalFunction(isEqualWithToleranceFcn);
            
            % Add description & specifications
            ikFcn.H1Line = h1Line;
            ikFcn.addCode(' ');
            ikFcn.addCode(dimensionSpec);
            ikFcn.addCode(offsetSpec);
            ikFcn.addCode(axesSignSpec);
            ikFcn.addCode(jointLimSpec);
            ikFcn.addCode(jointTypeSpec); % Even though currently all joints are revolute, it's important to pass this info in case we add support in the future
            ikFcn.addCode(' ');
            
            % If joint limits are respected, they must be known during
            % computation for cases where there are multiple plausible
            % solution, i.e. when joint are co-axial and therefore in
            % gimbal lock. In those cases, there are many possible
            % solutions, but arbitrary decisions that occur without
            % knowledge of the joint limits can ultimately be infeasible if
            % joint limits are only applied at the end. Instead, a set of
            % joint limits is used with the following modifications:
            %
            %    1) Since the IK computations do not consider theta offsets
            %       (these are reapplied at the end), the joint limits must
            %       be shifted.
            %
            %   2) Since the joint limits are always passed into functions
            %      that use them (for efficiency), if joint limits are not
            %      respected, they must be overridden by setting all values
            %      to [-inf inf]
            %
            % At the end of the IK computation, the joint limits are
            % applied to all joints using a helper that also takes into
            % account periodicity.
            ikFcn.addCode('% Compute the shifted joint limits, which are the limits during solution, where theta offsets are not yet in play');
            ikFcn.addCode('shiftedJointLimits = jointLimits + repmat(thetaOffsets(:),1,2);');
            ikFcn.addCode(' ');
            
            % Convert the transform to one that maps the DH end effector to
            % its base. The world pose describes the position of the end
            % effector in the world frame. If the DH robot's origin is not
            % at the world origin or is rotated so the first axis is about
            % an axis other than world Z, that information is contained in
            % the worldToDHBase transform. The orientation of the actual
            % end effector may differ by a rotation about Z from that of
            % the DH transform, as captured in the last ZScrew. The inverse
            % transforms are computed here during generation to minimize
            % extra computation in the generated function.
            worldToBaseTform = solutionDetails.SolutionMethodDetails.WorldToDHBase;
            dhEEToActEETform = solutionDetails.SolutionMethodDetails.F6ToEETform;
            ikFcn.addCode('% Convert the end effector pose in the global frame to the end effector described by the DH parameters relative to the DH-described origin')
            ikFcn.addCode(['baseToWorldTform = ' mat2str(robotics.manip.internal.tforminv(worldToBaseTform)) ';']);
            ikFcn.addCode(['actEEToDhEETform = ' mat2str(robotics.manip.internal.tforminv(dhEEToActEETform)) ';']);
            ikFcn.addCode('eePose = baseToWorldTform*eeTform*actEEToDhEETform;');
            ikFcn.addCode(' ');
            
            % Parse optional inputs
            ikFcn.addCode('% Parse optional inputs')
            ikFcn.addCode('narginchk(1,4);');
            
            % Reference config is zero by default
            ikFcn.addCode('if nargin < 4')
            ikFcn.addCode('referenceConfig = zeros(1,6);');
            ikFcn.addCode('end')
            
            % Distance-based sorting is off by default
            ikFcn.addCode('if nargin < 3')
            ikFcn.addCode('sortByDistance = false;');
            ikFcn.addCode('end')
            
            % Joint limits are enforced by default
            ikFcn.addCode('if nargin < 2')
            ikFcn.addCode('enforceJointLimits = true;');
            ikFcn.addCode('end')
            ikFcn.addCode(' ');
            
            ikFcn.addCode('% If joint limits are not enforced, set the shifted joint limits to have infinite range');
            ikFcn.addCode('if ~enforceJointLimits');
            ikFcn.addCode('shiftedJointLimits = repmat([-inf inf], size(jointLimits, 1), 1);');
            ikFcn.addCode('end');
            
            % Compute the transform that maps the end effector to the
            % intersection of joints. This is a constant translation in the
            % local frame (relative to the end effector), as long as the
            % last three axes are intersecting. For the transform T that
            % maps the end effector, EE, to joint 5, ee_T_J5 should satisfy
            % w_T_J5 = w_T_ee*ee_T_J5', where w_T_J5 is the position of
            % joint 5 in the world frame, and w_T_ee is the position of the
            % end effector in the world frame. Again, only translation
            % matters; the compete pose of joint 5 will differ between
            % joint angle solutions because the orientations are different.
            ikFcn.addCode('% Map the desired end effector pose to the pose of the central intersecting joint.');
            ikFcn.addCode(sprintf('eeToJt5 = %s;', mat2str(dhEEToJt5)));
            ikFcn.addCode('jt5Pose = eePose*eeToJt5;');
            ikFcn.addCode(' ');
            ikFcn.addCode('% Solve for the position of the first three joints from the pose of joint 5');
            ikFcn.addCode('q123Opts = %s;', firstThreeJointsFcnSignature);
            ikFcn.addCode(' ');
            
            % Initialize the output for the last three axes. In general,
            % there is just one solution (gimbal lock cases where there are
            % infinite solutions use just one default case), but in the ZYZ
            % case, every position solution has at least two valid
            % orientation solutions. Initialize the outputs depending on
            % the case using the hasMultipleRotationSolns flag
            ikFcn.addCode('% Solve for the positions of the intersecting axes');
            if hasMultipleRotationSolns
                ikFcn.addCode('% For each position solution, this configuration of the last three axes produces at least two possible orientation solutions');
                ikFcn.addCode('numRotationSolns = 2;');
                ikFcn.addCode('q456Opts = zeros(numRotationSolns*size(q123Opts,1), size(q123Opts,2));');
            else
                ikFcn.addCode('% For each position solution, this configuration of the last three axes produces at least one possible orientation solution');
                ikFcn.addCode('numRotationSolns = 1;');
                ikFcn.addCode('q456Opts = zeros(size(q123Opts));');
            end
            ikFcn.addCode(' ');
            
            % Make sure the axes are correctly mapped
            ikFcn.addCode(['% The next step seeks to find the orientation, which is entirely governed' ...
                ' by the last three joints. This means that rotation from the fourth joint' ...
                ' to the end effector can be mapped to three rotations in-place about the' ...
                ' fifth joint. Since the rotations are in-place, they can be defined' ...
                ' relative to the fourth joint axes, assuming a fixed pose rotation at the' ...
                ' end to align with the end effector. The fixed rotation is found using the' ...
                ' DH parameters, and corresponds to the rotation of the end effector' ...
                ' relative to the fourth joint when the last three joints are all zero.']);
            ikFcn.addCode('eeFixedAlpha = dhParams(4,2) + dhParams(5,2) + dhParams(6,2);');
            ikFcn.addCode('eeFixedRotation = [1 0 0; 0 cos(eeFixedAlpha) -sin(eeFixedAlpha); 0 sin(eeFixedAlpha) cos(eeFixedAlpha)];');
            
            % Start the for loop
            ikFcn.addCode('for jtIdx = 1:size(q123Opts,1)');
            
            % Call the generated joint 4 pose function and make sure it
            % gets added to the generated code
            ikFcn.addCode('% Get the position of the fourth joint at its zero position when the first three joints are positioned for IK');
            ikFcn.addCode(sprintf('jt4ZeroPose = %s(q123Opts(jtIdx,:));', joint4PoseFcn.Name));
            ikFcn.addCode(' ');
            ikFcn.addCode('% Compute the rotation matrix needed to get to the end');
            ikFcn.addCode('% The orientation of the end effector in the world frame can be written:');
            ikFcn.addCode('%    eeRot = jt4ZeroRot*(Rotation about axes 4-6)*eeFixedRotation');
            ikFcn.addCode('% Then the goal is to solve for the rotation about the axes and relate them to he known form from the DH parameters, if a valid solution exists:');
            ikFcn.addCode('%    (Rotation about axes 4-6) = jt4ZeroRot''*eeRot*eeFixedRotation''');
            ikFcn.addCode('jt4ToEERot = jt4ZeroPose(1:3,1:3)''*eePose(1:3,1:3)*eeFixedRotation'';');
            ikFcn.addCode(' ');
            
            % For the ZYZ case, which has at least two solutions, assign
            % both outputs. For all other cases, assign only one output.
            if hasMultipleRotationSolns
                ikFcn.addCode('% This orientation produces at least two configurations for every solution, when joint limits allow');
                ikFcn.addCode(sprintf('orientationSolns = %s(jt4ToEERot, lastThreeAxesSign, shiftedJointLimits(4:6,:));', rotationMatrixToAxisAnglesFcnName));
                ikFcn.addCode('q456Opts(jtIdx,:) = orientationSolns(1,:);');
                ikFcn.addCode('q456Opts(jtIdx + size(q123Opts,1),:) = orientationSolns(2,:);');
            else
                ikFcn.addCode('% This orientation produces at least one configuration for every solution, when joint limits allow');
                ikFcn.addCode(sprintf('q456Opts(jtIdx,:) = %s(jt4ToEERot, lastThreeAxesSign, shiftedJointLimits(4:6,:));', rotationMatrixToAxisAnglesFcnName));
            end
            ikFcn.addCode(' ');
            
            % Offset the solved joints by the theta offsets from the
            % z-screw
            ikFcn.addCode('% Offset theta to reflect the source robot configuration');
            ikFcn.addCode('q123Opts(jtIdx,:) = q123Opts(jtIdx,:) - thetaOffsets(1:3);');
            ikFcn.addCode('q456Opts(jtIdx,:) = q456Opts(jtIdx,:) - thetaOffsets(4:6);');
            if hasMultipleRotationSolns
                ikFcn.addCode('q456Opts(jtIdx + size(q123Opts,1),:) = q456Opts(jtIdx + size(q123Opts,1),:) - thetaOffsets(4:6);');
            end
            ikFcn.addCode(' ');
            
            % Apply joint limits
            ikFcn.addCode('% Remove solutions that violate joint limits');
            ikFcn.addCode('if enforceJointLimits');
            ikFcn.addCode('q123Opts(jtIdx,:) = applyJointLimits(q123Opts(jtIdx,:), jointLimits(1:3,:), isJointRevolute(1:3));');
            ikFcn.addCode('q456Opts(jtIdx,:) = applyJointLimits(q456Opts(jtIdx,:), jointLimits(4:6,:), isJointRevolute(1:3));');
            if hasMultipleRotationSolns
                ikFcn.addCode('q456Opts(jtIdx + size(q123Opts,1),:) = applyJointLimits(q456Opts(jtIdx + size(q123Opts,1),:), jointLimits(4:6,:), isJointRevolute(1:3));');
            end
            ikFcn.addCode('end');
            ikFcn.addCode(' ');
            
            % End the for loop
            ikFcn.addCode('end');
            ikFcn.addCode(' ');
                        
            % Combine the two parts and eliminate Nan-rows
            ikFcn.addCode('% Filter out any remaining rows with NaNs in them by getting the index of the valid rows and only assembling those in the final output');
            ikFcn.addCode('allSolnOpts = [repmat(q123Opts, numRotationSolns, 1) q456Opts];');
            ikFcn.addCode('isValidRowIdx = all(~isnan(allSolnOpts),2);');
            ikFcn.addCode('qOptsAllSolns = allSolnOpts(isValidRowIdx,:);');
            ikFcn.addCode(' ');
            
            % Remove periodic duplicates. This is done by first creating a
            % copy of the solutions that wraps them to pi (so that
            % identical entries look the same), and then rounding them
            % within the tolerance (so they show up as equal entries). Note
            % that round uses the single input format, as the multi-input
            % format does not support code generation.
            ikFcn.addCode('% Create a copy of the solutions that wraps all revolute joints to pi, then round within solution tolerance.');
            ikFcn.addCode(sprintf('qOptsWrappedAndRounded = round(robotics.internal.wrapToPi(qOptsAllSolns)*%e)/%e;', 1/solTolerance, 1/solTolerance));
            ikFcn.addCode(' ');
            
            % It's important to index into the original set of solutions
            % (Rather than just using the output of the "unique" function)
            % because those may have been wrapped to specifically consider
            % joint limits, whereas the re-wrapping, while geometrically
            % equivalent, could violate those constraints.
            ikFcn.addCode('% Find the indices of all unique values after wrapping to pi');
            ikFcn.addCode('[~, isUniqueValidRowIdx] = unique(qOptsWrappedAndRounded, ''rows'');');
            ikFcn.addCode(' ');
            ikFcn.addCode('% Select only unique solutions from the original set of solutions');
            ikFcn.addCode('qOpts = qOptsAllSolns(sort(isUniqueValidRowIdx),:);');
            
            % Add an optional distance metric sort
            ikFcn.addCode('% Sort results using a distance metric');
            ikFcn.addCode('if sortByDistance');
            ikFcn.addCode('qOpts = sortByEuclideanDistance(qOpts, referenceConfig(:)'', isJointRevolute);');
            ikFcn.addCode('end');
            ikFcn.addCode(' ');
            
            % Add in the helper functions
            ikFcn.addCode('    % Helper functions');
            internalHelperFcnNames = {'sortByEuclideanDistance', 'applyJointLimits', rotationMatrixToAxisAnglesFcnName, 'distributeRotationOverJoints'};
            
            for i = 1:numel(internalHelperFcnNames)
                AnalyticalIKHelpers.addHelperFunctionToGeneratedFunction(ikFcn, internalHelperFcnNames{i});
            end
            
            % Write the generated function to file
            ikFcn.writeFile(false);
        end
        
    end
    
    %% Helper methods
    
    methods (Static, Access = ?robotics.manip.internal.InternalAccess)
        function addHelperFunctionToGeneratedFunction(fcnGeneratorObj, helperName)
            %addHelperFunctionToGeneratedFunction Add a helper function saved as a file to the main function generator as a nested function
            %   The generated code references several helper files that are
            %   source from the robotics.manip.internal.analyticalIK.*
            %   directory. Since the function generator doesn't have any
            %   way to copy existing files to functions, this method
            %   instead finds those files, copies them to text, and adds
            %   them to the generated code as nested functions.
            
            % Find the file on the path
            helperFcnPath = which(strcat('robotics.manip.internal.analyticalIK.', helperName));

            % Get the function contents and copy it into a string
            helperText = fileread(helperFcnPath);

            % Add function contents to the generated object as a nested
            % function
            fcnGeneratorObj.addCode(helperText);
            fcnGeneratorObj.addCode(' ');
        end
        
        function [hasIntersection, intersectionPt] = findJointAxesIntersectionPoint(body1Idx, body2Idx, tree, tol)
            %findJointAxesIntersectionPoint Determine whether two axes intersect
            %   This method checks whether the axes from two joints
            %   intersect and returns the point of intersection in the
            %   frame of the first body, body1. The methods accepts the
            %   indices of the two bodies, as well as the rigid body tree
            %   and a tolerance.
            
            % Get body1 and body2 from the tree using the indices
            body1 = tree.Bodies{body1Idx};
            body2 = tree.Bodies{body2Idx};
            
            % Get a frame that positions the body2 joint relative to the
            % body1 joint
            jt1Idx = tree.TreeInternal.PositionDoFMap(body1Idx,2);
            jt2Idx = tree.TreeInternal.PositionDoFMap(body2Idx,2);
            tfTree = tree.TreeInternal.forwardKinematics(zeros(tree.TreeInternal.VelocityNumber, 1));
            T12 = robotics.manip.internal.AnalyticalIKHelpers.getJointTransform(tree, jt2Idx, jt1Idx, tfTree);
            
            % Compute the location of body1 and body2 axis in the body1
            % reference frame
            body1AxisInBody1Frame = body1.Joint.JointAxis(:);
            body2AxisInBody1Frame = T12(1:3,1:3)*body2.Joint.JointAxis(:);
            
            % To find intersections of two lines, we need to define those
            % lines using (1) the vector that defines the line in space and
            % (2) a point on the line to anchor it in 3-space.
            pointOnBody1Axis = [0; 0; 0]; % Since this is the reference frame, the axis runs through its origin
            pointOnBody2Axis = T12(1:3,4);
            
            % Compute the intersection of two lines
            [hasIntersection, intersectionPt] = robotics.manip.internal.AnalyticalIKHelpers.findIntersectionOfTwoLines(...
                pointOnBody1Axis, body1AxisInBody1Frame, pointOnBody2Axis, body2AxisInBody1Frame, tol);
        end
        
        function [hasIntersection, intersectionPt] = findIntersectionOfTwoLines(ptOnLine1, line1Vec, ptOnLine2, line2Vec, tol)
            %findLinesIntersectionPoint Find the intersection of two lines
            %   This method finds the intersection of two lines given
            %   vectors defining them and points on the lines. The method
            %   works by first checking if the lines are parallel. If not,
            %   a vector solution is used to find an intersection. Invalid
            %   results (returned when the lines are skew lines) are thrown
            %   out. The method then returns whether there is an
            %   intersection, and what that point is. If there is no
            %   intersection, the intersection point is returned as a 3x1
            %   NaN vector.
            
            % Initialize output and key variables
            hasIntersection = 0; %#ok<NASGU>
            intersectionPt = NaN(3,1);
            lineBtwnPts = ptOnLine2 - ptOnLine1;
            
            % Check whether the lines are parallel
            if(all(abs(cross(line1Vec, line2Vec)) <= tol))
                % The lines are parallel, so they only intersect if they
                % pass through the same points
                if(all(abs(cross(line1Vec, lineBtwnPts)) <= tol))
                    % If the cross product of the vector that defines line
                    % 1 and the vector between the two points is zero, then
                    % the lines are the same
                    hasIntersection = 2;
                    intersectionPt = ptOnLine1;
                else
                    hasIntersection = 0;
                end
            else
                % First, find the point of intersection using a vector
                % formulation. This approach defines a triangle with three
                % sides, where the line for each side is defined by a
                % vector and starting point:
                %    - Side 1: line1Vec, ptOnLine1:    point on line 1 to intersection
                %    - Side 2: line2Vec, ptOnLine2:    point on line 2 to intersection)
                %    - Side 3: lineBtwnPts, ptOnLine1: point on line 1 to point on line 2
                % The length of lineBtwnPts is known, as are the unit
                % directions of line1Vec and line2Vec, which means that the
                % angles between the lines are known. Then the goal is to
                % find the length of Side 1, so that the intersection point
                % can be formulated as:
                %    ptOnLine1 + direction*(Length of Side 1)*line1UnitVec
                % This is achieved using the law of Sines:
                %   L1/sin(theta1) = L3/sin(theta3)
                % where L1 and L3 are the lengths of sides 1 and 3,
                % respectively, and theta2 and theta3 are the angles across
                % from sides 1 and 3, respectively:
                %
                %            PtOnLine2
                %              /\
                %          L3 /  \L2
                %            /    \
                %  PtOnLine1/______\_______
                %              L1   \Intersection Point
                %
                % This leads to L1 = L3*sin(theta1)/sin(theta3).
                % Furthermore, since the vectors defining the directions of
                % L1 and L2 are unit vectors v1 and v2, the right hand side
                % can be found exclusively using cross products:
                %    ||v2 x L3|| = v2*L3*sin(theta1) = L3*sin(theta1)
                %    ||v2 x v3|| = v2*v3*sin(theta3) = sin(theta3)
                %
                % This formulation always returns a result, but if the
                % lines are skew, the result will be invalid (it won't
                % actually fall on both lines)
                
                % Ensure that the vectors that define the direction of
                % line1 and line2 are unit vectors
                line1Vec = line1Vec/norm(line1Vec);
                line2Vec = line2Vec/norm(line2Vec);
                
                % Take the cross products to of the vectors adjacent to the
                % angles needed for the computation
                v2CrossL3 = (cross(line2Vec, lineBtwnPts));
                v2Crossv1 = (cross(line2Vec, line1Vec));
                
                % Get the direction of the intersection point relative to
                % the offset from the relative orientations of the cross
                % products
                lineDir = sign(dot(v2CrossL3, v2Crossv1));
                line1Mag = norm(v2CrossL3)/norm(v2Crossv1);
                intersectionPt = ptOnLine1 + lineDir*line1Mag*line1Vec;
                
                % Check whether the intersection point actually falls on
                % both lines. For each line, verify that the cross product
                % of the line connecting the intersection point to a point
                % on line 1 with line 1 returns a value that is effectively
                % zero.
                line1PtCheck = all(abs(cross((intersectionPt - ptOnLine1), line1Vec)) <= tol);
                line2PtCheck = all(abs(cross((intersectionPt - ptOnLine2), line2Vec)) <= tol);
                
                % If the point does not exist one of the two lines, then
                % the lines are skew lines and do not intersect. Otherwise,
                % the intersect at the specified point.
                if ~line1PtCheck || ~line2PtCheck
                    % Lines are askew
                    hasIntersection = 0;
                    intersectionPt = NaN(3,1);
                else
                    % Lines intersect at a common point
                    hasIntersection = 1;
                end
            end
        end
        
        function tform = getJointTransform(tree, mesJtIdx, refJtIdx, tfTree)
            %getJointTransform Get the position of the measured joint frame relative to the reference joint frame
            %   This method computes the position of two joints relative to
            %   each other. The joints are specified by their joint index:
            %      - mesJtIdx specifies the index of the measurement joint
            %      - refJtIdx specifies the  index of the reference joint
            %   The main need for the method comes in differentiating
            %   between joints where the body frame is at the proximal end
            %   of the body (collocated with the joint) and those where it
            %   is at the distal end. This method ensure that those details
            %   don't matter, and the transform is always computed for the
            %   joint frame (i.e. a frame that is collocated with the
            %   joint).
            
            import robotics.manip.internal.AnalyticalIKHelpers
            
            if mesJtIdx > 0
                mesBodyIdx = AnalyticalIKHelpers.getBodyIndexFromJointIndex(tree, mesJtIdx);
                
                % Move the joint by the inverse of the child to joint
                % transform. If the joint was created without DH, this will
                % have no impact (the childToJoint will be I), but if it
                % was created with DH, this ensures that the body frame
                % coincides with the joint frame.
                mesInvChildToJoint = robotics.manip.internal.tforminv(tree.Bodies{mesBodyIdx}.Joint.ChildToJointTransform);
                mesJointBodyTForm = tfTree{mesBodyIdx}*mesInvChildToJoint;
            else
                mesJointBodyTForm = eye(4);
            end
            
            if refJtIdx > 0
                refBodyIdx = AnalyticalIKHelpers.getBodyIndexFromJointIndex(tree, refJtIdx);
                
                % Move the joint by the inverse of the child to joint
                % transform. If the joint was created without DH, this will
                % have no impact (the childToJoint will be I), but if it
                % was created with DH, this ensures that the body frame
                % coincides with the joint frame.
                refInvChildToJoint = robotics.manip.internal.tforminv(tree.Bodies{refBodyIdx}.Joint.ChildToJointTransform);
                refJointBodyTForm = tfTree{refBodyIdx}*refInvChildToJoint;
            else
                refJointBodyTForm = eye(4);
            end
            
            % Get the transform that converts the reference frame to the
            % measured frame
            tform = robotics.manip.internal.tforminv(refJointBodyTForm)*(mesJointBodyTForm);
        end
        
        function firstThreeJointsFcn = generateFirstThreeJointsFcn(fcnName, dhParams, idTolerance, solTolerance)
            %generateFirstThreeJointsFcn Generate solveFirstThreeDHJoints helper function
            %   This method creates a function generator object containing
            %   the solveFirstThreeDHJoints helper function. This helper
            %   function solves for the first three joints, which govern
            %   position. This can then be added to the main IK function as
            %   a local function. This method generates the function;
            %   details of what the function does and how it works may be
            %   found in the generated help text below.
            
            import robotics.manip.internal.AnalyticalIKHelpers
            
            % Extract decision variables from the dh parameters
            a1 = dhParams(1,1);
            alpha1 = dhParams(1,2);
            
            % Add H1 line and help text for the generated file
            firstThreeJointsFcn = sigutils.internal.emission.MatlabFunctionGenerator( fcnName, {'jt5Pos', 'dhParams'}, 'outputThetas' );
            firstThreeJointsFcn.H1Line = 'Solve for the first three joint angles of a DH-parameterized robot';
            firstThreeJointsFcn.addCode('%   This function computes the first three joint angles of a robot');
            firstThreeJointsFcn.addCode('%   parameterized using Denavit-Hartenberg parameters. The function accepts');
            firstThreeJointsFcn.addCode('%   a matrix of the fixed DH parameters, as well as the position of the');
            firstThreeJointsFcn.addCode('%   fifth joint. The matrix of DH parameters is of size 6x4 for the 6');
            firstThreeJointsFcn.addCode('%   non-fixed joints, where each row has the order [a alpha d 0], where a');
            firstThreeJointsFcn.addCode('%   is a translation along x, alpha is a rotation about x, and d is a');
            firstThreeJointsFcn.addCode('%   translation along z. The last value, which typically refers to theta');
            firstThreeJointsFcn.addCode('%   (the rotation about z) for that joint, is not yet known; this function');
            firstThreeJointsFcn.addCode('%   will solve for theta for the first three joints. When a robot has the');
            firstThreeJointsFcn.addCode('%   last three axes intersecting, the position and orientation of the end');
            firstThreeJointsFcn.addCode('%   effector can be split up: the position is entirely determined by the');
            firstThreeJointsFcn.addCode('%   first three joints, while the orientation is governed by the last three');
            firstThreeJointsFcn.addCode('%   joints (provided the first three are known). Furthermore, the position');
            firstThreeJointsFcn.addCode('%   of any end effector can be related to the position of the fifth joint');
            firstThreeJointsFcn.addCode('%   frame, which corresponds to the joint frame at the midpoint of the');
            firstThreeJointsFcn.addCode('%   three intersecting joints. This frame will have the same position in');
            firstThreeJointsFcn.addCode('%   any valid solution to the particular IK problem (though its orientation');
            firstThreeJointsFcn.addCode('%   may differ), and its translation relative to the base frame is entirely');
            firstThreeJointsFcn.addCode('%   defined by the first three joints. This function solves for those first');
            firstThreeJointsFcn.addCode('%   three joints given the position of the joint 5 frame relative to the');
            firstThreeJointsFcn.addCode('%   base. This solution method and notation follows Chp.3 of Pieper''s 1968');
            firstThreeJointsFcn.addCode('%   thesis, but adds two corrections, as well as minor notation changes and');
            firstThreeJointsFcn.addCode('%   the addition of constraints to ensure only feasible solutions are');
            firstThreeJointsFcn.addCode('%   output:');
            firstThreeJointsFcn.addCode('%');
            firstThreeJointsFcn.addCode('%   Pieper, D. The Kinematics Of Manipulators Under Computer Control.');
            firstThreeJointsFcn.addCode('%   Stanford University (1968).');
            firstThreeJointsFcn.addCode(' ');
            
            firstThreeJointsFcn.addCode('% Extract DH parameters from matrix');
            firstThreeJointsFcn.addCode('[a1, a2, a3] = deal(dhParams(1,1), dhParams(2,1), dhParams(3,1));');
            firstThreeJointsFcn.addCode('[alpha1, alpha2, alpha3] = deal(dhParams(1,2), dhParams(2,2), dhParams(3,2));');
            firstThreeJointsFcn.addCode(' ');
 
            firstThreeJointsFcn.addCode('% Note that Pieper uses "s" instead of "d" in his solutions');
            firstThreeJointsFcn.addCode('[d1, d2, d3, d4] = deal(dhParams(1,3), dhParams(2,3), dhParams(3,3), dhParams(4,3));');
            firstThreeJointsFcn.addCode(' ');
 
            firstThreeJointsFcn.addCode('% Three variables derived from jt5Pos');
            firstThreeJointsFcn.addCode('z3 = jt5Pos(3);');
            firstThreeJointsFcn.addCode('R3 = jt5Pos(1)^2 + jt5Pos(2)^2 + (jt5Pos(3) - d1)^2;');
            firstThreeJointsFcn.addCode('z = z3 - d1;');
            firstThreeJointsFcn.addCode(' ');
            
            % Solve for theta3. This requires solving for h (see comments
            % below), but the exact approach is dependent on the DH
            % parameters
            firstThreeJointsFcn.addCode('% The first step is to solve for theta3. This is achieved by eliminating');
            firstThreeJointsFcn.addCode('% theta1 and theta2 through a number of substitutions and sum-of-squares');
            firstThreeJointsFcn.addCode('% operations. The resultant equation for theta3, is a function of');
            firstThreeJointsFcn.addCode('% sin(theta3) and cos(theta3), but this can be further mapped to a');
            firstThreeJointsFcn.addCode('% polynomial in h, where h = tan(theta3/2). This substitutions is made');
            firstThreeJointsFcn.addCode('% possible by the Weierstrass transformation, which maps sin(theta3) to');
            firstThreeJointsFcn.addCode('% 2*h/(1 + h^2) and cos(theta3) to (1-h^2)/(1+h^2). The goal is then to');
            firstThreeJointsFcn.addCode('% solve the polynomial for h and map the solutions back to theta3.');
            firstThreeJointsFcn.addCode(' ');
            if isEqualWithinTolerance(a1, 0, idTolerance)
                % If a1 is zero, the solution comes from applying the transformation to
                % the equation R3 = F3 and solving for h. This produces a quadratic in
                % h, and will have two solutions.
                solveHFcnName = 'solveForHCaseZeroA1';
                firstThreeJointsFcn.addCode('% Since a1 = 0, the solution arises from R3 = F3, which produces a quadratic in h')
                firstThreeJointsFcn.addCode('[hSolns, ~, hasPiSoln] = solveForHCaseZeroA1(R3, a2, a3, alpha2, alpha3, d2, d3, d4);');

            elseif isEqualWithinTolerance(sin(alpha1), 0, idTolerance)
                % If sin(alpha1) is zero, the solution comes from applying the
                % transformation to z3 + s1 = F4 and solving for h. This produces a
                % quadratic in h, and will have two solutions.
                solveHFcnName = 'solveForHCaseZeroSinAlpha1';
                firstThreeJointsFcn.addCode('% Since sin(alpha1) = 0, the solution arises from z3 = F(4) - s1, which produces a quadratic in h')
                firstThreeJointsFcn.addCode('[hSolns, ~, hasPiSoln] = solveForHCaseZeroSinAlpha1(a3, alpha1, alpha2, alpha3, d2, d3, d4, z);');

            else
                % If neither a1 nor sin(alpha1) are zero, the solution is found using a
                % sum of squares to eliminate theta2. This produces a quartic
                % polynomial in h and will have four solutions.
                solveHFcnName = 'solveForHGeneralCase';
                firstThreeJointsFcn.addCode('% As a1 and sin(alpha1) are both nonzero, use sum of squares to eliminate theta2, which produces a quartic in h')
                firstThreeJointsFcn.addCode('[hSolns, ~, hasPiSoln] = solveForHGeneralCase(R3, z, a1, a2, a3, alpha1, alpha2, alpha3, d2, d3, d4);');
            end
            
            % In total, there are up to 16 solutions for a 6-DoF robot
            % (treating configurations where two or more joints form a
            % null-space with infinite possibilities as a single case).
            % Additionally, the number of solutions grows by joint: theta3
            % can have up to 8 solutions and theta2 can have another two
            % options. For each theta2/theta3 combo, there is one valid
            % theta1.
            firstThreeJointsFcn.addCode('% Initialize the matrix of possible solutions');
            firstThreeJointsFcn.addCode('possThetas = zeros(16,3);');
            
            % As we add values, if an item is imaginary or otherwise
            % invalid, it will be replaced with a NaN value. At the end,
            % all rows with NaN values are discarded so only valid
            % solutions remain.
            firstThreeJointsFcn.addCode('% After all solutions are processed, rows with NaNs will be removed');
            
            % Some solutions only replace a subset of the 16 solutions, so
            % the whole last column is first initialized to NaN to ensure
            % the rows that haven't been adjusted will be thrown out.
            firstThreeJointsFcn.addCode('% Initialize theta3 to NaN and replace based on actual solutions');
            firstThreeJointsFcn.addCode('possThetas(:,3) = NaN;');
            
            % Solve for theta3 from the values of h computed in the
            % polynomial above. Use a for-loop to ensure all values are
            % addressed.
            firstThreeJointsFcn.addCode('for hIdx = 1:numel(hSolns)'); % Start for-hIdx loop
            firstThreeJointsFcn.addCode('% Ensure only real solutions to h are converted');
            firstThreeJointsFcn.addCode('h3 = replaceImagWithNaN(hSolns(hIdx));');
            firstThreeJointsFcn.addCode('if isnan(h3)');
            firstThreeJointsFcn.addCode('% When h3 is imaginary, theta3 = NaN');
            firstThreeJointsFcn.addCode('possThetas(hIdx,3) = NaN;');
            firstThreeJointsFcn.addCode('possThetas(4+hIdx,3) = NaN;');
            firstThreeJointsFcn.addCode('else');
            firstThreeJointsFcn.addCode('% When h3 is real, there are two possible equivalent values of theta3');
            firstThreeJointsFcn.addCode('possThetas(hIdx,3) = 2*atan2(h3,1);')
            firstThreeJointsFcn.addCode('possThetas(4+hIdx,3) = 2*atan2(-h3,-1);');
            firstThreeJointsFcn.addCode('end');
            firstThreeJointsFcn.addCode('end'); % End for-hIdx loop
            
            % If theta3 = pi is a solution, it will not be found using the
            % Weierstrass substitution since tan(theta/2) is undefined when
            % theta = pi. This is checked by an output of the "solveForH.."
            % functions.
            firstThreeJointsFcn.addCode('if hasPiSoln')
            firstThreeJointsFcn.addCode('possThetas(numel(hSolns)+1,3) = pi;');
            firstThreeJointsFcn.addCode('end');
            
            % Next, loop over the 8 possible solutions to theta3 and solve
            % for theta2 (and ultimately theta 1 in an inner loop).
            firstThreeJointsFcn.addCode('for theta3Idx = 1:8'); % Start for-theta3Idx loop
            
            % Make sure any rows where theta3 is not a valid, real number
            % are automatically populated with NaNs and skipped over. These
            % will be discarded at the end.
            firstThreeJointsFcn.addCode('% If theta3 is NaN or imaginary, replace whole row with NaNs and skip to next row');
            firstThreeJointsFcn.addCode('theta3 = replaceImagWithNaN(possThetas(theta3Idx,3));');
            firstThreeJointsFcn.addCode('if isnan(possThetas(theta3Idx,3))');
            firstThreeJointsFcn.addCode('possThetas(theta3Idx,:) = [NaN NaN NaN];');
            firstThreeJointsFcn.addCode('continue');
            firstThreeJointsFcn.addCode('end');
            firstThreeJointsFcn.addCode(' ');
            
            % Compute key subexpressions
            firstThreeJointsFcn.addCode('% Compute key subexpressions f1 to f3 and F1 to F4, which are functions of theta3');
            firstThreeJointsFcn.addCode('f = computef13SupportingEquations(a3, alpha3, d3, d4, theta3);');
            firstThreeJointsFcn.addCode('F = computeF14SupportingEquations(a1, a2, alpha1, alpha2, f(1), f(2), f(3), d2);');
            
            firstThreeJointsFcn.addCode('% Compute theta2. The exact approach depends on the DH');
            firstThreeJointsFcn.addCode('% parameters, but the gist is the same: since the equations');
            firstThreeJointsFcn.addCode('% output multiple solutions, but some are actually just results');
            firstThreeJointsFcn.addCode('% of the sum of squares, i.e., they solve the local problem,');
            firstThreeJointsFcn.addCode('% but do not actually solve the overlying problem. Rather than');
            firstThreeJointsFcn.addCode('% compute all solutions and filter at the end, we filter here');
            firstThreeJointsFcn.addCode('% by always solving using two different equations. Then we');
            firstThreeJointsFcn.addCode('% choose only the solution that satisfies both equations.');
            firstThreeJointsFcn.addCode(' ');
            
            if ~isEqualWithinTolerance(a1, 0, idTolerance) && ~isEqualWithinTolerance(sin(alpha1), 0, idTolerance)
                firstThreeJointsFcn.addCode('% Since a1 and sin(alpha1) are both nonzero, solve for theta2 using equation 3.25 and 3.26');
                firstThreeJointsFcn.addCode('theta2Opts = solveTrigEquations(F(1)*2*a1, F(2)*2*a1, R3 - F(3));');                     % Pieper equation 3.25
                firstThreeJointsFcn.addCode('theta2Constraint = solveTrigEquations(-F(2)*sin(alpha1), F(1)*sin(alpha1), z - F(4));'); % Pieper equation 3.26
                firstThreeJointsFcn.addCode(' ');
                
                firstThreeJointsFcn.addCode('% Choose the solution(s) that solve both equations');
                firstThreeJointsFcn.addCode(sprintf('theta2 = chooseCorrectSolution(theta2Opts, theta2Constraint, %e);', solTolerance));
                firstThreeJointsFcn.addCode(' ');
                
                % Set a flag that is true when theta1 and theta2 are
                % in-line. In that case, an infinite number of solutions is
                % possible, and the solution must be specially handled
                isTheta1Theta2Complement = false;
        
            elseif isEqualWithinTolerance(a1, 0, idTolerance) && ~isEqualWithinTolerance(sin(alpha1), 0, idTolerance)
                firstThreeJointsFcn.addCode('% Since a1 is zero and sin(alpha1) is nonzero, solve for theta2 using equation 3.26 and the third element of 3.20');
                firstThreeJointsFcn.addCode('theta2Opts = solveTrigEquations(-F(2)*sin(alpha1), F(1)*sin(alpha1), z - F(4));'); % Pieper equation 3.26
                firstThreeJointsFcn.addCode(' ');
                
                firstThreeJointsFcn.addCode('rhsConst = jt5Pos(3) - d1 - cos(alpha1)*(sin(alpha2)*f(2) + cos(alpha2)*f(3) + d2);');
                firstThreeJointsFcn.addCode('lhsCosCoeff = -sin(alpha1)*(-cos(alpha2)*f(2) + sin(alpha2)*f(3));');
                firstThreeJointsFcn.addCode('lhsSinCoeff = sin(alpha1)*(a2 + f(1));');
                firstThreeJointsFcn.addCode('theta2Constraint = solveTrigEquations(lhsCosCoeff, lhsSinCoeff, rhsConst);'); % Pieper equation 3.20 third element
                firstThreeJointsFcn.addCode(' ');
                
                firstThreeJointsFcn.addCode('% Choose the solution(s) that solve both equations');
                firstThreeJointsFcn.addCode(sprintf('theta2 = chooseCorrectSolution(theta2Opts, theta2Constraint, %e);', solTolerance));
                firstThreeJointsFcn.addCode(' ');
                
                % Set a flag that is true when theta1 and theta2 are
                % in-line. In that case, an infinite number of solutions is
                % possible, and the solution must be specially handled
                isTheta1Theta2Complement = false;
                
            elseif ~isEqualWithinTolerance(a1, 0, idTolerance) && isEqualWithinTolerance(sin(alpha1), 0, idTolerance)
                firstThreeJointsFcn.addCode('% Since a1 is nonzero and sin(alpha1) is zero, solve for theta2 using equation 3.25 and 3.24');
                
                firstThreeJointsFcn.addCode('theta2Opts = solveTrigEquations(F(1)*2*a1, F(2)*2*a1, R3 - F(3));');                 % Pieper equation 3.25
                firstThreeJointsFcn.addCode(' ');
                
                firstThreeJointsFcn.addCode('rhsConst = R3 - (f(1)^2 + f(2)^2 + f(3)^2 + a1^2 +a2^2 + d2^2 + 2*a2*f(1) + 2*d2*(sin(alpha2)*f(2) + cos(alpha2)*f(3)));');
                firstThreeJointsFcn.addCode('lhsCosCoeff = 2*a1*(a2 + f(1));');
                firstThreeJointsFcn.addCode('lhsSinCoeff = 2*a1*(-cos(alpha2)*f(2) + sin(alpha2)*f(3));');
                firstThreeJointsFcn.addCode('theta2Constraint = solveTrigEquations(lhsCosCoeff, lhsSinCoeff, rhsConst);');  % Pieper equation 3.24
                firstThreeJointsFcn.addCode(' ');
                
                firstThreeJointsFcn.addCode('% Choose the solution(s) that solve both equations');
                firstThreeJointsFcn.addCode(sprintf('theta2 = chooseCorrectSolution(theta2Opts, theta2Constraint, %e);', solTolerance));
                firstThreeJointsFcn.addCode(' ');
                
                % Set a flag that is true when theta1 and theta2 are
                % in-line. In that case, an infinite number of solutions is
                % possible, and the solution must be specially handled
                isTheta1Theta2Complement = false;
            else
                % If sin(alpha1) and a1 are both zero, then theta1 and theta2 form
                % a null space and share an infinite combination of values. In that
                % case, assume that theta1 is zero and solve for theta2
                firstThreeJointsFcn.addCode('% Since a1 and sin(alpha1) are both zero, theta1 and theta2 are co-axial');
                firstThreeJointsFcn.addCode('% In this case, an infinite combination of solutions are possible, so one');
                firstThreeJointsFcn.addCode('% of the values must be fixed so the other can be solved for. Therefore,  ');
                firstThreeJointsFcn.addCode('% assume that theta1 is zero and solve for theta2 using the first two ');
                firstThreeJointsFcn.addCode('% elements of equation 3.20.');
                
                % Solve 3.20 (Solving for x)
                firstThreeJointsFcn.addCode('theta2Opts = solveTrigEquations(a2 + f(1), -cos(alpha2)*f(1) + sin(alpha2)*f(3), jt5Pos(1));');        % Pieper equation 3.20 (first element)
                firstThreeJointsFcn.addCode('theta2Constraint = solveTrigEquations(cos(alpha2)*f(1) - sin(alpha2)*f(3), a2 + f(1), jt5Pos(2));');	% Pieper equation 3.20 (second element)
                firstThreeJointsFcn.addCode(sprintf('theta2 = chooseCorrectSolution(theta2Opts, theta2Constraint, %e);', solTolerance));
                firstThreeJointsFcn.addCode(' ');
                
                % Set a flag that is true when theta1 and theta2 are
                % in-line. In that case, an infinite number of solutions is
                % possible, and the solution must be specially handled
                isTheta1Theta2Complement = true;
            end
            
            % Update table of all possible theta values and filter out
            % invalid values
            firstThreeJointsFcn.addCode('% Theta2 is a 2-element vector with up to two valid solutions (invalid');
            firstThreeJointsFcn.addCode('% solutions are represented by NaNs). Iterate over the possible values');
            firstThreeJointsFcn.addCode('% and add the second solution set in the latter half of the matrix (so');
            firstThreeJointsFcn.addCode('% they aren''t overwritten by subsequent loops).');
            
            % Start an inner for-loop to solve for theta1 for all possible
            % theta2 values
            firstThreeJointsFcn.addCode('for theta2Idx = 1:2');
            firstThreeJointsFcn.addCode('% Update the local index so it''s reflective of the indexed value of theta2');
            firstThreeJointsFcn.addCode('solIdx = theta3Idx + 8*(theta2Idx-1);');
            firstThreeJointsFcn.addCode(' ');

            firstThreeJointsFcn.addCode('% Update the value of theta3 in case it was previously set to NaN,');
            firstThreeJointsFcn.addCode('% and replace any invalid values of theta2 with NaN');
            firstThreeJointsFcn.addCode('possThetas(solIdx,3) = theta3;');
            firstThreeJointsFcn.addCode('possThetas(solIdx,2) = replaceImagWithNaN(theta2(theta2Idx));');
            firstThreeJointsFcn.addCode(' ');

            firstThreeJointsFcn.addCode('% If any of the joint variables in NaN, replace it and all the');
            firstThreeJointsFcn.addCode('% remaining joints to solve with NaNs and move on to the next loop');
            firstThreeJointsFcn.addCode('if isnan(possThetas(solIdx, 2))');
            firstThreeJointsFcn.addCode('possThetas(solIdx, 1:2) = [NaN NaN];');
            firstThreeJointsFcn.addCode('continue;');
            firstThreeJointsFcn.addCode('end');
            firstThreeJointsFcn.addCode(' ');
            
            % Complete the solution
            if isTheta1Theta2Complement
                firstThreeJointsFcn.addCode('% Since theta1 is in line with theta2, theta2 was computed assuming theta1 = 0');
                firstThreeJointsFcn.addCode('% As long as theta2 is non-nan, theta1 = 0. Otherwise, theta1 = NaN.');
                firstThreeJointsFcn.addCode('if ~isnan(theta2(theta2Idx))');
                firstThreeJointsFcn.addCode('possThetas(solIdx,1) = 0;');
                firstThreeJointsFcn.addCode('else');
                firstThreeJointsFcn.addCode('possThetas(solIdx,1) = nan;');
                firstThreeJointsFcn.addCode('end');
                firstThreeJointsFcn.addCode(' ');
            else
                firstThreeJointsFcn.addCode('% Compute theta1 from the first two elements of eq 3.20');
                firstThreeJointsFcn.addCode('g = computeg12SupportingEquations(a1, a2, alpha1, alpha2, f(1), f(2), f(3), d2, theta2(theta2Idx));');
                firstThreeJointsFcn.addCode('theta1Opts = solveTrigEquations(g(1), g(2), jt5Pos(1));');
                firstThreeJointsFcn.addCode('theta1Constraint = solveTrigEquations(-g(2), g(1), jt5Pos(2));');
                firstThreeJointsFcn.addCode(sprintf('theta1Opts = chooseCorrectSolution(theta1Opts, theta1Constraint, %e);', solTolerance));
                firstThreeJointsFcn.addCode(' ');
                
                firstThreeJointsFcn.addCode('% Since theta1 is the last value that is solved for, only one');
                firstThreeJointsFcn.addCode('% of the solutions will be valid, and chooseCorrectSolution');
                firstThreeJointsFcn.addCode('% sorts the results so that if there is only one solution, it');
                firstThreeJointsFcn.addCode('% is always the first element (and the other element is nan)');
                firstThreeJointsFcn.addCode('theta1 = theta1Opts(1);');
                firstThreeJointsFcn.addCode(' ');
                
                firstThreeJointsFcn.addCode('% Update the array of possible theta values');
                firstThreeJointsFcn.addCode('possThetas(solIdx,1) = replaceImagWithNaN(theta1);');
                firstThreeJointsFcn.addCode(' ');
            end
            
            % Close inner theta1 for-loop 
            firstThreeJointsFcn.addCode('end');
            firstThreeJointsFcn.addCode(' ');
            
            % Close theta2 for-loop 
            firstThreeJointsFcn.addCode('end');
            firstThreeJointsFcn.addCode(' ');
    
            % Eliminate the rows containing NaN values
            firstThreeJointsFcn.addCode('% Now we are left with an 8x3 matrix where some values are NaN. The');
            firstThreeJointsFcn.addCode('% function will only output the rows where all elements are non-NaN.');
            firstThreeJointsFcn.addCode('outputThetas = possThetas(all(~isnan(possThetas),2),:);');
            
            % Add the necessary nested functions from files
            firstThreeJointsFcn.addCode('% Helper functions');
            internalHelperFcnNames = {'computef13SupportingEquations', 'computeF14SupportingEquations', 'computeg12SupportingEquations', ...
                'solveTrigEquations', 'chooseCorrectSolution', 'replaceImagWithNaN', solveHFcnName};
            if strcmp(solveHFcnName, 'solveForHGeneralCase')
                internalHelperFcnNames = [internalHelperFcnNames {'solveQuarticPolynomial'}];
            end
            for i = 1:numel(internalHelperFcnNames)
                AnalyticalIKHelpers.addHelperFunctionToGeneratedFunction(firstThreeJointsFcn, internalHelperFcnNames{i});
            end
            
        end
        
        function firstThreeJointsPoseFcn = generateFirstThreeDHJointsPose(dhParams)
            %generateFirstThreeDHJointsPose Generate function for computing the pose of joint 4
            %   This function generator object defines the subroutine
            %   "getJoint4PoseFromDH", which defines the pose of the fourth
            %   joint frame given theta1, theta2, and theta3 when theta4 is
            %   zero. The subroutine accepts q123, a 3-element vector
            %   containing [theta1 theta2 theta3] and the DH-parameters for
            %   the robot in standard 6x4 form where each row has the
            %   elements [a alpha d 0]. The function then defines the
            %   product of the first three DH matrices, which define the
            %   joint 4 pose when theta4 = 0. 
            
            firstThreeJointsPoseFcn = sigutils.internal.emission.MatlabFunctionGenerator( 'getJoint4PoseFromDH', 'q123', 'jt4Pose' );
            firstThreeJointsPoseFcn.H1Line = 'Get the pose of the fourth joint when the first three joints set to q123 and joint4 angle is zero';
            firstThreeJointsPoseFcn.addCode(' ');
            firstThreeJointsPoseFcn.addCode(['dhParams = ' mat2str(dhParams) ';']);
            firstThreeJointsPoseFcn.addCode(' ');
            
            firstThreeJointsPoseFcn.addCode('% Initialize output');
            firstThreeJointsPoseFcn.addCode('jt4Pose = eye(4);');
            
            % Assemble the transformation matrices. Inside a for-loop, each
            % complete DH matrix is split into two terms, Ttheta =
            % (rotation about z of theta), which represents the portion of
            % the DH matrix that is variable in theta, and TFixed =
            % (translation of a along x * translation of d along z), which
            % represents the portion of the DH matrix that is constant
            % since the DH parameters are known at function generation time.
            % rotation about x of alpha). Each subsequent DH matrix is then
            % post-multiplied, so that the output of the for-loop at the end
            % is the pose of joint 4.
            firstThreeJointsPoseFcn.addCode('for i = 1:3');
            % Extract DH Parameters
            firstThreeJointsPoseFcn.addCode('a = dhParams(i,1);');
            firstThreeJointsPoseFcn.addCode('alpha = dhParams(i,2);');
            firstThreeJointsPoseFcn.addCode('d = dhParams(i,3);');
            firstThreeJointsPoseFcn.addCode('theta = q123(i);');
            firstThreeJointsPoseFcn.addCode(' ');
            
            % Assemble each transform
            firstThreeJointsPoseFcn.addCode('Ttheta = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];');
            firstThreeJointsPoseFcn.addCode('TFixed = [1 0 0 a; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) d; 0 0 0 1];');
            firstThreeJointsPoseFcn.addCode(' ');
            
            % Compute the new output
            firstThreeJointsPoseFcn.addCode('jt4Pose = jt4Pose*Ttheta*TFixed;');
            
            % Close the for-loop
            firstThreeJointsPoseFcn.addCode('end');
            firstThreeJointsPoseFcn.addCode(' ');
        end
        
        function fcnGenerator = generateIsEqualWithinToleranceFcn(tolerance)
            %generateIsEqualWithinToleranceFcn Generate a function that checks for equality within a fixed tolerance
            %   The generated code needs to check equality a few times. In
            %   those cases, a tolerance is used, though the tolerance is
            %   fixed and defined during generation. This method creates
            %   the helper function used to check for equivalence and
            %   hard-codes the tolerance value in the generates function.
            
            fcnGenerator = sigutils.internal.emission.MatlabFunctionGenerator( 'isEqualWithinTolerance', {'mat1', 'mat2'}, 'isEquiv' );
            fcnGenerator.H1Line = 'Check if two matrices are equal within a set tolerance';
            fcnGenerator.addCode('%   This is a convenience function designed for inputs with up to two');
            fcnGenerator.addCode('%   dimensions. If the input has 3+ dimensions, a non-scalar output will be');
            fcnGenerator.addCode('%   returned.');
            fcnGenerator.addCode(' ');
            
            fcnGenerator.addCode(sprintf('tol = %e;', tolerance));
            fcnGenerator.addCode('diffMat = abs(mat1 - mat2);');
            fcnGenerator.addCode('isEquiv = all(all(diffMat < tol));');
            fcnGenerator.addCode(' ');
            
        end
       
        function [lastThreeAxesSign, lastThreeAxesSequenceStr, hasMultipleRotationSolns] = getLastThreeAxesInfo(lastThreeAxes)
            %getLastThreeAxesInfoFromDH Get info related to the orientation computation of the last three axes
            %   This function accepts a 3x3 matrix indicating the
            %   orientations of the last three axes (where the ith row
            %   corresponds to the ith axis), and converts that to a string
            %   format and an associated sign. The string format takes the
            %   form 'LLL' where the ith letter L is either X, Y, or Z,
            %   indicating that the ith axis is along that joint axis. The
            %   associated vector of signs indicates whether the axis is
            %   positive or negative. Finally, the hasMultipleRotationSolns
            %   output is a boolean flag that is true when a rotation
            %   solution can produce more than one set of solutions for
            %   cases other than those in gimbal lock. For the current axis
            %   set, this is limited the ZYZ case, which always produces at
            %   least two solutions.
            
            %Initialize outputs
            lastThreeAxesSign = [1 1 1];
            lastThreeAxesSequenceStr = 'ZZZ';
            axesStringOpts = {'X' 'Y', 'Z'};
            
            for i = 1:3
                axisValue = lastThreeAxes(i,:);
                lastThreeAxesSign(i) = axisValue(axisValue~=0);
                lastThreeAxesSequenceStr(i) = axesStringOpts{axisValue~=0};
            end
            
            % Set a flag that indicates whether the corresponding
            % orientation solution can have more than one value for cases
            % where the total number of solutions is finite
            hasMultipleRotationSolns = strcmp(lastThreeAxesSequenceStr, 'ZYZ');
        end
    end
end

function isEquiv = isEqualWithinTolerance(mat1, mat2, tol)
%isEqualWithinTolerance Check if two matrices are equal within a set tolerance

diffMat = abs(mat1 - mat2);
isEquiv = all(all(diffMat < tol));

end
