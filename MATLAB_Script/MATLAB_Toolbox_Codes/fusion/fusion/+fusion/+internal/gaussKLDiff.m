function klDiff = gaussKLDiff(m,P)
% This is an internal function and may be modified or removed in a future
% release.
% This function computes the inter KL-diff between Gaussian distributions.
% kldiff is a NxN matrix.
% m is PxN vector representing the mean of each Gaussian component.
% P is PxPxN matrix representing covariance of each Gaussian component.

% References:
% Granström, Karl, and Umut Orguner. "On the reduction of Gaussian inverse 
% Wishart mixtures." Information Fusion (FUSION), 2012 15th International 
% Conference on. IEEE, 2012.

% Copyright 2018 The MathWorks, Inc.

%#codegen

n = size(m,2);
dataType = class(m);
klDiff = zeros(n,n,dataType);

for i = 1:n
    for j = i+1:n
        m1 = m(:,i);
        m2 = m(:,j);
        P1 = P(:,:,i);
        P2 = P(:,:,j);
        val = singleKlDiff(m1,m2,P1,P2);
        klDiff(i,j) = val;
        klDiff(j,i) = val;
    end
end

end
function diff = singleKlDiff(m1,m2,P1,P2)
    e = m1 - m2;
    nx = size(m1,1);
    % KL-diff = KlDiv(dist2,dist1) + KLDiv(dist1,dist2)
    diff = 0.5*(trace(P2\P1 + P1\P2) + e'/P2*e + e'/P1*e) - nx;
end
