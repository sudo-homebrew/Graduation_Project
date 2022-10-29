function klDiff = iwKLDiff(nu,V)
% This is an internal function and may be modified or removed in a future
% release. This function computes the inter KL-diff between Inverse-Wishart
% (IW) distributions. 
% kldiff is a NxN matrix. 
% nu is Nx1 vector representing the degrees of freedom of each IW component. 
% V is dxdxN matrix representing scale matrix of each IW component.

% Copyright 2018 The MathWorks, Inc.

% References:
% Granström, Karl, and Umut Orguner. "On the reduction of Gaussian inverse 
% Wishart mixtures." Information Fusion (FUSION), 2012 15th International 
% Conference on. IEEE, 2012.

%#codegen
n = numel(nu);
dataType = class(nu);
klDiff = zeros(n,n,dataType);

for i = 1:n
    for j = i+1:n
        v1 = nu(i);
        v2 = nu(j);
        V1 = V(:,:,i);
        V2 = V(:,:,j);
        val = singleIWKlDiff(v1,v2,V1,V2);
        klDiff(i,j) = val;
        klDiff(j,i) = val;
    end
end

end
function diff = singleIWKlDiff(nu1,nu2,V1,V2)
% KL-diff is the sum of forward and backward KL-divergence
d = size(V1,1);
dTerms = cast(1:d,class(nu1));
v1D = (nu1 - dTerms - d)/2;
v2D = (nu2 - dTerms - d)/2;
t1 = 1/2*trace((2*v1D(1)*eye(d)/V1 - 2*v2D(1)*eye(d)/V2)*(V2 - V1));
t2 = (nu2-nu1)/2*(log(det(V1)) - sum(psi(0,v1D)) - log(det(V2)) + sum(psi(0,v2D)));
diff = t1 + t2;
end