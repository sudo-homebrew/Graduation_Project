function [actAngles, jointsInGimbalLock] = convertRotationToZZYAxesAngles(tgtRotation, axesSign, jointLim)
%convertRotationToZZYAxesAngles Convert desired orientation to rotation about Z-Z-Y
%   This function is used to three angles of rotation corresponding to
%   consecutive joint angles whose joint axes intersect at a single, common
%   point. This function addresses the case where the first joint rotation
%   is about Z, and the subsequent angles, in order and defined relative to
%   the first joint frame, are about Z and then Y. The second output
%   indicates joints that are in gimbal lock, where 1 indicates that they
%   are, and zero indicates that they are not. When joints are in gimbal
%   lock, the affected joint axes align and an infinite combination of
%   joint angles are possible (as long as the total rotation still matches
%   the target). The default assumption is that rotation is divided over
%   the joint along the affected axis. For ZZY, since the first two joints
%   are coaxial, these joints are always in gimbal lock.

%   Copyright 2020 The MathWorks, Inc.

% Indicate joints in gimbal lock, i.e. joints that are co-axial and
% therefore can take on an infinite set of possible valid solutions. This
% is a flag that is consistent across rotation functions and may be used by
% the caller function.
jointsInGimbalLock = [1 1 0];

% For ZZY, rotation is reduced to two consecutive rotations: rotation about
% Z, then Y. If rotation about Z is theta and about Y is gamma, the
% resultant rotation matrix rotZ(theta)*rotY(gamma) has the following form:
% [cos(gamma)*cos(theta), -sin(theta), cos(theta)*sin(gamma)]
% [cos(gamma)*sin(theta),  cos(theta), sin(gamma)*sin(theta)]
% [          -sin(gamma),           0,            cos(gamma)]
theta = atan2(-tgtRotation(1,2), tgtRotation(2,2)); %Rotation about Z
gamma = atan2(-tgtRotation(3,1), tgtRotation(3,3)); %Rotation about Y

% Check that derived rotation matches target rotation
chkRotation = [cos(gamma)*cos(theta), -sin(theta), cos(theta)*sin(gamma); ...
               cos(gamma)*sin(theta),  cos(theta), sin(gamma)*sin(theta); ...
                         -sin(gamma),           0,            cos(gamma)];
if ~isEqualWithinTolerance(chkRotation, tgtRotation)
    % No valid solution
	actAngles = nan(1,3);
else
    % Distribute the z-axis rotation, which consists of two co-axial joints
    % (those in gimbal lock), over the joint axes using a helper function. This
    % helper also incorporates the axes signs.
    zAngles = distributeRotationOverJoints(theta, axesSign([1 2]), jointLim([1 2],:));
    
    % Factor the axes sign into the y-axis rotation
    yAngle = axesSign(3)*gamma;
    
    % Assemble the joints into an output vector
    actAngles = [zAngles(1) zAngles(2) yAngle];
end

end
