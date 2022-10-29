function logLikelihood = gaussCellLikelihood(P,z,zExp,H,R,dataType,U)
% This is an internal function and may be removed or modified in a future
% release.
% This function returns the log-likelihood of n measurements (z) generated
% by an extended object with state covariance P. 
% This function can be used for two purposes.
% 1. Calculate likelihoods of measurements which are independent of extent.
% 2. Calculate likelihoods of measurements when extent is not modeled at
% all.
% P is a M-by-M matrix representing the state covariance
% z is a p-by-n matrix, p is the size of measurement, n is the number of
% measurements.
% zExp is a p-by-1 vector representing the expected measurement.
% H is p-by-M matrix representing the jacobian of expected measurement.
% R is a w-by-w-by-n matrix, representing measurement noise of each
% detection
% dataType is the class to use to perform calculations.
% U is an optional input representing the jacobian of measurement wrt the
% measurement noise. U is a p-by-w matrix. When U is no provided, p = w.

% Copyright 2018 The MathWorks, Inc.

%#codegen

hasAMN = nargin == 6;
zTotal = z(:);
numMeas = size(z,2);
zTotalExp = repmat(zExp,[numMeas 1]);
HTotal = repmat(H,[numMeas 1]);
p = size(z,1);
RTotal = zeros(p*numMeas,class(z));
for i = 1:numMeas
    index = ((i-1)*p + 1):(i*p);
    if hasAMN
        RTotal(index,index) = R(:,:,i);
    else
        RTotal(index,index) = U*R(:,:,i)*U;
    end
end
STotal = HTotal*P*HTotal' + RTotal;
rTotal = zTotal - zTotalExp;
k = numel(z);
% Inform codegen this is real.
logLikelihood = real(-1/2*(rTotal'/STotal*rTotal) - 1/2*log(det(STotal)) - k/2*cast(1.8379,dataType)); % log(2*pi) = 1.8379;

end