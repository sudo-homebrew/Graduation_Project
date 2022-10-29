function x = constacc2constvel(x1, x2)
% This function is for internal use and may be removed or modified later.
%constacc2constvel function to convert state or stateCovariance.
%   x = constacc2constvel(x1, x2) converts the state or stateCovariance.
%   x1 - specifies state or stateCovariance corresponding to constacc
%   x2 - specifies state or stateCovariance corresponding to constvel
%   x1 - a L-by-1 real vector or L-by-L real matrix
%   x2 – a M-by-1 real vector or M-by-M real matrix.
%   x  - a M-by-1 real vector or M-by-M real matrix
%
%   Example 1 - Converting state from constant acceleration to constant
%   velocity 
%   --------------------------------------------------------------------
%   %The input parameters for constacc2constvel are
%   x1 = [1;2;3;4;5;6;7;8;9];
%   x2 = [0;0;0;0;0;0];
%   x  = constacc2constvel(x1, x2)
%

%   Copyright 2018 The MathWorks, Inc.

%#codegen

dims1 = fusion.internal.constaccdims(x1);
dims2 = fusion.internal.constveldims(x2);
dims = min(dims1, dims2);
x2 = zeros(size(x2),'like',x2);

doCov = ~isvector(x1);
if doCov
    x = zeros(size(x2),'like',x2);
else
    x = x2;
end
for r = 1:dims
    indr = (1:2)+2*(r-1);
    indr1 = (1:2)+3*(r-1);
    if doCov
        x(indr,indr) = x1(indr1,indr1);
    else
        x(indr) = x1(indr1);
    end
    
    % Off diagonal terms
    if doCov
        for c = 1:dims
            indc1 = (1:2)+3*(c-1);
            indc = (1:2)+2*(c-1);
            x(indr,indc) = x1(indr1,indc1);
        end
    end
end

% Augment with diagonal blocks from higher dimension state
if doCov
    iAug = 2*dims+1:2*dims2;
    x(iAug,iAug) = x2(iAug,iAug);
    x = fusion.internal.fixPZeros(x);
end
end
