function [actAngles, jointsInGimbalLock] = convertRotationToZYYAxesAngles(tgtRotation, axesSign, jointLim)
%convertRotationToZYYAxesAngles Convert desired orientation to rotation about Z-Y-Y
%   This function is used to three angles of rotation corresponding to
%   consecutive joint angles whose joint axes intersect at a single, common
%   point. This function addresses the case where the first joint rotation
%   is about Z, and the subsequent angles, in order and defined relative to
%   the first joint frame, are about Y and then Z. The second output
%   indicates joints that are in gimbal lock, where 1 indicates that they
%   are, and zero indicates that they are not. When joints are in gimbal
%   lock, the affected joint axes align and an infinite combination of
%   joint angles are possible (as long as the total rotation still matches
%   the target). The default assumption is that rotation is divided over
%   the joint along the affected axis. For ZYY, since the last two joints
%   are coaxial, these joints are always in gimbal lock.
 
%   Copyright 2020 The MathWorks, Inc.
 
% Indicate joints in gimbal lock, i.e. joints that are co-axial and
% therefore can take on an infinite set of possible valid solutions. This
% is a flag that is consistent across rotation functions and may be used by
% the caller function.
jointsInGimbalLock = [0 1 1];
 
% Solve for rotation about Z and Y. For rotation theta about Z and gamma
% about Y, the output format will be:
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
    % Factor the axes sign into the z-axis rotation
    zAngle = axesSign(1)*theta;
    
    % Distribute the y-axis rotation, which consists of two co-axial joints
    % (those in gimbal lock), over the joint axes using a helper function. This
    % helper also incorporates the axes signs.
    yAngles = distributeRotationOverJoints(gamma, axesSign([2 3]), jointLim([2 3],:));
    
    % Assemble the joints into an output vector
    actAngles = [zAngle yAngles(1) yAngles(2)];
end
 
end
