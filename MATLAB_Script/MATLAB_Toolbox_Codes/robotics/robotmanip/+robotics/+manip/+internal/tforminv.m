function Tinv = tforminv(T)
%This function is for internal use only. It may be removed in the future.

%tforminv Efficient inverse of a homogeneous transform.

%   Copyright 2016 The MathWorks, Inc.

%#codegen

R = T(1:3,1:3)';
p = -R*T(1:3,4);
Tinv = [R,    p; ...
       [0 0 0 1]];