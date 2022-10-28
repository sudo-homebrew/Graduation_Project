function klDiff = gammaKLDiff(alpha,beta)
% This is an internal function and may be modified or removed in a future
% release.
% This function computes the inter KL-diff between Gamma distributions.
% kldiff is a NxN matrix.
% alpha is Nx1 vector representing the shape of each Gamma component.
% beta is a Nx1 vector representing rate of each Gamma component.

% Copyright 2018 The MathWorks, Inc.

%#codegen
[Alpha1,Alpha2] = meshgrid(alpha,alpha);
[Beta1,Beta2] = meshgrid(beta,beta);
klDiff = (Alpha1 - Alpha2).*(psi(0,Alpha1) - psi(0,Alpha2) + log(Beta2) - log(Beta1)) + (Beta2 - Beta1).*(Alpha1./Beta1 - Alpha2./Beta2);

end