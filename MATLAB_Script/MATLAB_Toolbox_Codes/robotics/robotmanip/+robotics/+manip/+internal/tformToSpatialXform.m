function X = tformToSpatialXform(T)
%This class is for internal use only. It may be removed in the future.

%TFORMTOSPATIALTFORM convert homogeneous transformation (4x4) to its adjoint
%   spatial transformation (6x6). 

%   Copyright 2016 The MathWorks, Inc.

%#codegen
    
    R = T(1:3,1:3);
    p = T(1:3, 4);
    X = [R, zeros(3); robotics.manip.internal.skew(p)*R, R];

end



