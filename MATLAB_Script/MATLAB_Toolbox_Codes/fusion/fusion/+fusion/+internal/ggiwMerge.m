function [wMerged,xMerged,Pmerged,alphaMerged,betaMerged,nuMerged,VMerged,labelMerged]= ggiwMerge(w,x,P,alpha,beta,nu,V,labels)
% This is an internal function and may be modified or removed in a future release.

% [wMerged,xMerged,Pmerged,alphaMerged,betaMerged,vMerged,VMerged,labelMerged]
% = ggiwMerge(w,x,P,alpha,beta,v,V,labels) merges multiple GGIW states to a
% single ggiw state.
% Inputs:
%   w: weights of each component
%   x: gaussian state
%   P: gaussian state covariances
%   alpha: shapes
%   beta: rates;
%   v: dof
%   V: scale matrices
% Outputs:
%   Same convention as input, merged states

% Copyright 2018 The MathWorks, Inc.

%#codegen
n = numel(w);
classToUse = class(x);

%% Merging Gaussian components
wMerged = sum(w);
wRow = reshape(w,1,n);
xMerged = sum(bsxfun(@times,wRow,x),2)/wMerged;
wPage = reshape(w,1,1,n);
Pmerged = sum(bsxfun(@times,wPage,P),3)/wMerged;
for i = 1:n
   e = x(:,i) - xMerged;
   Pmerged = Pmerged + w(i)*(e*e')/wMerged;
end

%% Merging IW components
if nargin > 3
% Instead of using optimized value of v, a weighted mean is used.
nuMerged = sum(w(:).*nu(:))/wMerged;
d = size(V,1);
vMin = cast(2*d + 3,classToUse);
nuMerged = max(vMin,nuMerged);

% Merge scale matrices
VMerged1 = wMerged*(nuMerged - d - 1);

% Allocate memory for Vinvs
Vinv = V;
for i = 1:n
    Vinv(:,:,i) = eye(d,classToUse)/V(:,:,i);
end
wV = wRow(:).*(nu(:) - d - 1);
wVPage = reshape(wV,1,1,n);
weightedVinvSum = sum(bsxfun(@times,wVPage,Vinv),3);
VMerged = VMerged1*eye(d,classToUse)/weightedVinvSum;
end

%% Merging Gamma components
if nargin > 5
% Instead of using optimized value of alpha, a weighted mean is used.
alphaMerged = sum(w(:).*alpha(:))/wMerged;

denominator = 1/wMerged*sum(w(:).*alpha(:)./beta(:));
betaMerged = alphaMerged/denominator;

end
%% Merge labels. Pick the max-weight label
if nargin > 7
    [~,i] = max(w);
    labelMerged = labels(i);
else
    labelMerged = uint32(0);
end

end