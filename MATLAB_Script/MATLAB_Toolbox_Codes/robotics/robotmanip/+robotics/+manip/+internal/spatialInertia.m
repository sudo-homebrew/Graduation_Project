function spatialI = spatialInertia( mass, com, inertia )
%This function is for internal use only. It may be removed in the future.

%SPATIALINERTIA Form the 6x6 spatial inertia matrix
%   mass - rigid body mass
%   com - center of mass of the rigid body relative to body frame
%   inertia - 3x3 inertia tensor of the rigid body relative to body frame

%   Copyright 2016 The MathWorks, Inc.

%#codegen
    sc = robotics.manip.internal.skew(com);
    spatialI = [inertia, mass*sc; mass*sc', mass*eye(3)];

end

