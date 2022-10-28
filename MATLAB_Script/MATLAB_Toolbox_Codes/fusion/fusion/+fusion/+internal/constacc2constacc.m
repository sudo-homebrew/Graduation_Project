function x = constacc2constacc(x1, x2)
% This function is for internal use and may be removed or modified later.
%constacc2constacc function to convert state or stateCovariance.
%   x = constacc2constacc(x1, x2) converts the state or stateCovariance.
%   x1 - specifies state or stateCovariance corresponding to constacc
%   x2 - specifies state or stateCovariance corresponding to constacc
%   x1 - a L-by-1 real vector or L-by-L real matrix
%   x2 – a M-by-1 real vector or M-by-M real matrix.
%   x  - a M-by-1 real vector or M-by-M real matrix
%
%   Example 1 - Converting state from constant acceleration to constant
%   acceleration 
%   --------------------------------------------------------------------
%   %The input parameters for constacc2constacc are
%   x1 = [1;2;3;4;5;6;7;8;9];
%   x2 = [0;0;0;0;0;0;0;0;0];
%   x  = constacc2constacc(x1, x2)
%

%   Copyright 2018 The MathWorks, Inc.

%#codegen
x2 = zeros(size(x2),'like',x2);
x  = fusion.internal.same2same(x1, x2);
end

