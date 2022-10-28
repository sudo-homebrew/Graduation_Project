function dims = constaccdims(x)
% This function is for internal use and may be removed or modified later.
%constaccdims determines the dimension of constant acceleration motion model

%   Copyright 2018 The MathWorks, Inc.

%#codegen

dims = size(x,1)/3;
end