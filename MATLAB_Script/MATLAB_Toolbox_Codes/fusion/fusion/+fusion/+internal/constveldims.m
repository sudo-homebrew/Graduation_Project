function dims = constveldims(x)
% This function is for internal use and may be removed or modified later.
%constveldims determines the dimension of constant velocity motion model

% Copyright 2018 The MathWorks, Inc.

%#codegen

dims = size(x,1)/2;
end