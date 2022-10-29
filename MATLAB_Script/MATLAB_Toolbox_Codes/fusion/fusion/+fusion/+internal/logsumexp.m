function s = logsumexp(x)
% This is an internal function and may be removed or modified in a future
% release. 

% Copyright 2021 The MathWorks, Inc.

% A function to compute log(sum(exp(x))) without overflow/underflow.
%
% x is a vector

%#codegen
xmax = max(x);
xdiff = x - xmax;
s = xmax + log(sum(exp(xdiff)));

end