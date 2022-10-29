function x = same2same(x1, x2)
% This function is for internal use and may be removed or modified later.
%same2same converts the state or stateCovariance of same motion model 
%   same2same(x1,x2) will work for constvel2constvel,constacc2constacc,
%   constturn2constturn Model conversion functions.To convert the state or 
%   stateCovariance of same or different dimensions.
%   x1 - a L-by-1 real vector or L-by-L real matrix
%   x2 – a M-by-1 real vector or M-by-M real matrix.
%

% Copyright 2018 The MathWorks, Inc.

%#codegen

num1 = size(x1,1);
num2 = size(x2,1);
num = min(num1, num2);

doCov = ~isvector(x1);
if doCov
    x = zeros(size(x2),'like',x2);
else
    x = x2;
end

if doCov
    x(1:num, 1:num) = x1(1:num, 1:num);
else
    x(1:num) = x1(1:num);
end

% Augment with diagonal block from higher dimension state
if doCov
    iAug = num+1:num2;
    x(iAug,iAug)= x2(iAug, iAug);
end
end