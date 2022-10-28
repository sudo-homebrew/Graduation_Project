function Ivec = flattenInertia( Imat )
%This function is for internal use only. It may be removed in the future.

%FLATTENINERTIA This function extracts the independent terms from a 3x3
%   inertia matrix and flatten them into a 1x6 vector with the order 
%   required by rigidBody class, i.e. [Ixx Iyy Izz Iyz Ixz Ixy]

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen

Ivec = [Imat(1,1), Imat(2,2), Imat(3,3), Imat(2,3), Imat(1,3), Imat(1,2)];

end

