function [actAngles, jointsInGimbalLock] = convertRotationToZZZAxesAngles(tgtRotation, axesSign, jointLim)
%convertRotationToZYZAxesAngles Convert desired orientation to rotation about Z-Z-Z
%   This function is used to three angles of rotation corresponding to
%   consecutive joint angles whose joint axes intersect at a single, common
%   point. This function addresses the case where the first joint rotation
%   is about Z, and the subsequent angles, in order and defined relative to
%   the first joint frame, are also about Z. The function accepts the
%   rotation of the last joint relative to the origin of the first one, as
%   well as the sign of each axes. The second output indicates joints that
%   are in gimbal lock, where 1 indicates that they are, and zero indicates
%   that they are not. When joints are in gimbal lock, the affected joint
%   axes align and an infinite combination of joint angles are possible (as
%   long as the total rotation still matches the target). The default
%   assumption is that rotation is divided over the joint along the
%   affected axis. For ZZZ, since the joints are coaxial, all the joints
%   are always in gimbal lock.
 
%   Copyright 2020 The MathWorks, Inc.
 
% Indicate redundant joints, i.e. joints that complement each other and can
% have an infinite pair of values in the directJointAngleMaps output. This
% is a flag that is consistent across rotation functions and may be used by
% the caller function.
jointsInGimbalLock = [1 1 1];
 
% Convert to axis-angle format (note that this will only output angles on
% the interval [-pi, pi])
axangRot = rotm2axang(tgtRotation);
 
% Check if the axis is Z and determine the cumulative rotation
if isEqualWithinTolerance(abs(axangRot(1:3)), [0 0 1]) 
    % If the axis is the Z axis, the total rotation is determined by the
    % sign and angle of the axis-angle output
    totalRotation = axangRot(4)*sign(axangRot(3));
    
elseif isEqualWithinTolerance(abs(axangRot(4)), 0)
    % If the angle is zero, the identified axis doesn't matter (rotation of
    % 0 radians about any axis is equivalent)
    totalRotation = 0;
    
else
    % If the angle is non-zero and the axis is not the Z-axis, this
    % rotation is not feasible and a NaN vector is returned
    actAngles = nan(1,3);
    return;
end
 
% Distribute rotation over the joint axes using a helper function. This
% helper also incorporates the axes signs.
actAngles = distributeRotationOverJoints(totalRotation, axesSign, jointLim);
 
end
