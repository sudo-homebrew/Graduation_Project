 function [actAngles, jointsInGimbalLock] = convertRotationToZYZAxesAngles(tgtRotation, axesSign, jointLim)
%convertRotationToZYZAxesAngles Convert desired orientation to rotation about Z-Y-Z
%   This function is used to three angles of rotation corresponding to
%   consecutive joint angles whose joint axes intersect at a single, common
%   point. This function addresses the case where the first joint rotation
%   is about Z, and the subsequent angles, in order and defined relative to
%   the first joint frame, are about Y and then Z. The function accepts the
%   rotation of the last joint relative to the origin of the first one, as
%   well as the sign of each axes. The second output indicates joints that
%   are in gimbal lock, where 1 indicates that they are, and zero indicates
%   that they are not. When joints are in gimbal lock, the affected joint
%   axes align and an infinite combination of joint angles are possible (as
%   long as the total rotation still matches the target). The default
%   assumption is that rotation is divided over the joint along the
%   affected axis.

%   Copyright 2020 The MathWorks, Inc.

eulAngles = rotm2eul(tgtRotation, 'ZYZ');
 
% The jointsInGimalLock variable indicates redundant joints, i.e. joints
% that complement each other and can have an infinite pair of values in the
% directJointAngleMaps output. Initialize this value to zeros (no joints in
% gimbal lock). This is a flag that is consistent across rotation functions
% and may be used by the caller function.
jointsInGimbalLock = [0 0 0];

% When the middle angle is zero, the first and last joints are co-axial,
% meaning there are an infinite set of solutions. Use a helper function to
% distribute the values consistently given knowledge of the joint limits.
if isEqualWithinTolerance(eulAngles(2), 0)
    
    newTgtRotation = tgtRotation;
    newTgtRotation(1:2,3) = 0;
    newTgtRotation(3,1:2) = 0;
    newTgtRotation(3,3) = 1;
    eulAngles = rotm2eul(newTgtRotation, 'ZYZ');

    variableJtIdx = [1 3];
    jointsInGimbalLock(variableJtIdx) = [1 1];
    totalRotation = sum(eulAngles(variableJtIdx));
    eulAngles(variableJtIdx) = distributeRotationOverJoints(totalRotation, axesSign(variableJtIdx), jointLim(variableJtIdx,:));
    
    % In this case the alternate Euler angles aren't required, as they will
    % also result in a set of co-axial joints. However, to ensure codegen
    % compatibility, the size must stay the same Therefore, return a set of
    % NaN angles (so the second solution may be thrown out). Note that the
    % axes sign is handled inside the distributeRotationOverJoints helper
    % function.
    actAngles = [eulAngles; nan(1,3)];
else
    % For finite solutions when the middle angle is non-zero, there are two possible solutions to this problem
    % that can be derived from the first solution set
    eulAltUnwrapped = eulAngles;
    eulAltUnwrapped(:,2) = -eulAltUnwrapped(:,2);
    eulAltUnwrapped = eulAltUnwrapped + pi;
    eulAltUnwrapped(:,2) = eulAltUnwrapped(:,2) - pi;
    eulAnglesAlt = robotics.internal.wrapToPi(eulAltUnwrapped);
    
    % Output the angles given the axes signs
    actAngles = [eulAngles; eulAnglesAlt]*diag(axesSign);
end

end
