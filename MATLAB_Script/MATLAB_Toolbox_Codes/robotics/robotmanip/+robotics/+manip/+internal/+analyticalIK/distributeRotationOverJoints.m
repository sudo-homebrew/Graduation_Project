function jointAngles = distributeRotationOverJoints(totalRotation, axesSigns, jointLim)
%distributeRotationOverJoints Distribute a rotation over several in-line revolute joints
%   When revolute joints are co-axial, the total rotation can be distributed
%   over the joints in a number of ways. This function assigns a default
%   behavior that respects the joint limits. For the case where no joint
%   limits are required, they should be provided as infinite, i.e [-inf
%   inf] for each joint. The default behaviors are as follows:
%
%      - If any joint limits have a range of at minimum 2*pi, all total 
%        rotation amounts are achievable and the rotation is distributed
%        evenly over the joints with infinite range, assuming the other
%        joints are centered within their range.
%
%      - If all joints have finite ranges with total range less than 2*pi, 
%        some total rotation amounts may not be feasible and the rotation
%        is distributed as much as possible on the distal joints (unused
%        more proximal joints are centered). If the solution is infeasible,
%        a NaN-vector is returned.
%
%   The goal of these distributions is to favor solutions that are
%   efficient. This function accepts the total rotation (in radians) to be
%   divided over N joints, the signs of those N joints (whether rotation is
%   positive or negative about the axes), and the joint limits, given as an
%   Nx2 matrix.
%   
%   If joint limits are ignored, they can be provided as infinite; the
%   behavior is equivalent. This function returns an N-element row vector.
 
%   Copyright 2020 The MathWorks, Inc.
 
% Get the total number of joints from the joint limit input
N = size(jointLim, 1);
 
% Initialize the output
jointAngles = zeros(1,N);
 
% Remap the joint limits to fit the assumption that all axes are positive.
% Since the joint limits can contain infinite elements, it is necessary to
% use element-wise multiplication, as matrix multiplication can result in
% NaNs when it causes sums of infinities.
jointLim = repmat(axesSigns(:),1,2).*jointLim;
        
% Re-order the joint limits to ensure the lower limit always comes first
% (in case the of a sign flip in the previous line)
jointLim = sort(jointLim,2);
 
% Determine the total ranges of each joint. Since all joints are revolute,
% a range of 2*pi or greater is equivalent to an infinite range as the IK
% problem does not distinguish between periodic equivalents. Note that a
% downstream helper in the IK solution, applyJointLimits, includes a check
% that wraps periodic equivalents back to their values given the joint
% limits.
jointRange = jointLim(:,2) - jointLim(:,1);
isRevJointFullRange = (jointRange > 2*pi);
for limIdx = 1:size(jointRange,1)
    % Use a tolerance check on the equality. Since isEqualWithinTolerance
    % returns a scalar, it is necessary to do this inside a for-loop
    isRevJointFullRange(limIdx) = isRevJointFullRange(limIdx) || isEqualWithinTolerance(jointRange(limIdx), 2*pi);
end
 
% There are two primary cases: when some joints have full range, any
% solution is feasible and the variable values are distributed over these
% joints. When all of the joints have range of less than 2*pi, the problem
% is more complex, as some solutions may not be feasible.
if any(isRevJointFullRange)
    % If any of the joint have infinite ranges, use that to distribute the
    % total rotation. First, place the joints with finite ranges in the
    % middle of their respective ranges, then distribute the remaining
    % joint rotation evenly over the joints with at least 2*pi range.
    jointIdxVector = 1:N;
    jointsWithIncompleteRange = jointIdxVector(~isRevJointFullRange);
    for i = 1:numel(jointsWithIncompleteRange)
        jointIdx = jointsWithIncompleteRange(i);
        jointAngles(jointIdx) = sum(jointLim(jointIdx,:))/2;
    end
    
    % Compute the remaining rotation and wrap it to the interval [-pi, pi],
    % then distribute over the joints with complete range
    wrappedRemainder = robotics.internal.wrapToPi(totalRotation - sum(jointAngles));
    jointsWithCompleteRange = jointIdxVector(isRevJointFullRange);
    for j = 1:numel(jointsWithCompleteRange)
        jointIdx = jointsWithCompleteRange(j);
        jointAngles(jointIdx) = wrappedRemainder/numel(jointsWithCompleteRange);
    end
    
else
    % Use an algorithm that favors loading distal joints, which are
    % typically easier to change: first set all the joints to their
    % mid-range values. Then iterate over the joints from first to last,
    % moving each joint up or down based on the difference in the current
    % total from the desired total, until the desired total is reached.
    % This is essentially a cascaded bang-bang controller.
    
    % Initialize the joint angles to their mid-range values
    jointAngles(:) = (sum(jointLim,2)/2)';
    
    % Iterate over the joints, using a feedback law to move them closer
    % to the desired total
    jointIdxVector = N:-1:1;
    wrappedTotalRotation = robotics.internal.wrapToPi(totalRotation);
    for jointIdx = 1:numel(jointIdxVector)
        diffRotation = robotics.internal.wrapToPi(wrappedTotalRotation - sum(jointAngles));
        jointAngles(jointIdx) = jointAngles(jointIdx) + sign(diffRotation)*min(abs(diffRotation), jointRange(jointIdx)/2);
    end
        
    % Check if the sum of the joint angles reaches the desired total. If
    % not, the solution is infeasible and a vector of NaNs is returned.
    if ~isEqualWithinTolerance(robotics.internal.wrapToPi(sum(jointAngles)), wrappedTotalRotation)
        jointAngles = nan(size(jointAngles));
        return;
    end
end
 
% Factor back in the axes signs. Since all valid joint angles are finite,
% matrix multiplication is the most efficient approach.
jointAngles = jointAngles*diag(axesSigns);
 
end