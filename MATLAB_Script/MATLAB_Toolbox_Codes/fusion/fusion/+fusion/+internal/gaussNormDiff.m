function normDiff = gaussNormDiff(m,P,vsFirst)
% This is an internal function and may be modified or removed in a future
% release.
% This function computes the normalized distance between Gaussian distributions.
% normDiff is a NxN matrix.
% m is PxN vector representing the mean of each Gaussian component.
% P is PxPxN matrix representing covariance of each Gaussian component.
% vsFirst is a flag. If true, return only one row of comparison vs. 1st set
% of m and P. 

% References:
% TODO

% Copyright 2018 The MathWorks, Inc.

%#codegen

n = size(m,2);
dataType = class(m);

if nargin==2
    vsFirst = false;
end

if vsFirst
    nRows = 1;
else
    nRows = n;
end
normDiff = zeros(nRows,n,dataType);

for i = 1:nRows
    normDiff(i,:) = calcOneRow(m,P,i,dataType);
end
end

function oneRow = calcOneRow(m,P,i,dataType)
e = bsxfun(@minus,m(:,i),m);
S = bsxfun(@plus, P(:,:,i),P);
n = size(m,2);
oneRow = zeros(1,n,dataType);
for i = 1:n
    e1 = e(:,i);
    oneRow(i) = e1' / S(:,:,i) * e1 + log(det(S(:,:,i)));
end
end