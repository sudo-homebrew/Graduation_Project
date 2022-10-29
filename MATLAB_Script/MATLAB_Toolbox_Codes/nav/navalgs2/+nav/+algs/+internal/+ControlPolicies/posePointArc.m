function [rad,chord,thArc,S] = posePointArc(pose,pt,invertDir)
%This function is for internal use only. It may be removed in the future.

%posePointArc Calculates arc originating at pose and intersecting an xy point

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    if nargin == 2
        invertDir = false;
    end
    
    % Get pose angle
    th = pose(3);
    
    v = pt(1:2)-pose(1:2);
    
    % Transform vector pointing from pt->pose to pose frame
    R = [cos(th) -sin(th); sin(th) cos(th)];
    dv = R'*v';
    
    % Calculate chord length
    chord = norm(dv,2);
    
    % Calculate radius and arclength
    thArc = 2*atan2(dv(2),dv(1));
    rad = chord/sin(thArc/2)/2;
    if invertDir
        thArc = thArc-sign(thArc)*2*pi;
    end
    S = rad*thArc;
end
