function P = fixPZeros(P)
% This function is for internal use and may be removed or modified later.
%fixPZeros function fixes the zeros in StateCovariance diagonal.

% Copyright 2018 The MathWorks, Inc.

%#codegen
diagonal = diag(P);
needFix = (diagonal == 0);
L = 100; % Large value
diagonal = L*needFix;
P = P + diag(diagonal);
end
