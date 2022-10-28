function pts = evaluateArc(initPose,r,s)
%This function is for internal use only. It may be removed in the future.

%evaluateArc Generate xy points along an arc
%
%   PTS = evaluateArc(INITPOSE, R, S) Calculates xy locations on an arc 
%   which is tangent to INITPOSE. R is a signed radius, which defines the
%   curvature of the arc, and S is a signed arc-length, dictating whether
%   generated points are offset in the tangent or anti-tangent direction

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    thInit = initPose(3)-pi/2;
    R = [cos(thInit) -sin(thInit); sin(thInit) cos(thInit)];
    dTh = s/r;
    dx = cos(dTh)*r-r;
    dy = sin(dTh)*r;
    pts = initPose(1:2)'+R*[dx;dy];
end
