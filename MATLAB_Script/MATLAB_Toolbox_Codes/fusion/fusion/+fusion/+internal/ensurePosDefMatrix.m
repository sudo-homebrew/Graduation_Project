function [P, Q, D] = ensurePosDefMatrix(P)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2021 The MathWorks, Inc.

% This function ensures that an input matrix is converted to a positive
% definite matrix. The negative eigen values are replaced by eps(class(P)).
%
% P = fusion.internal.ensurePosDefMatrix(P) ensures that output P is
% positive definite. 
%
% [P, Q, D] = fusion.internal.ensurePosDefMatrix(P) provides new
% eigenvectors Q and eigenvalue matrix, D. P = Q*D*Q';

%#codegen

% Compute eigen values
[Q, D] = eig(P);

% Replace negative eigen values with eps(1)
d = diag(D);
d = max(real(d),eps(class(P))); % Q and D are complex in generated code.
D = diag(d);

% Recompute P
P = real(Q*D*Q');

end