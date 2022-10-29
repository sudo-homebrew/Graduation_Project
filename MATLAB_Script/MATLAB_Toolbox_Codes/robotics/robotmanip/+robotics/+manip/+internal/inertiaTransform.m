function Io = inertiaTransform( Ii, mass, R, p )
%This function is for internal use only. It may be removed in the future.

%INERTIATRANSFORM This function transforms the 3x3 inertia matrix Ii to
%   the target frame. The current inertia frame (i) is related to the
%   target frame (o) through (R,p) pair. 
%   
%   R:   The orientation of frame i in frame o (o_R_i)
%   p:   The position of the origin of frame i, relative to frame o

%   Copyright 2017 The MathWorks, Inc.

%#codegen

sp = robotics.manip.internal.skew(p);
Io = R*Ii*R' + mass*(sp*sp');

end

