function loglikelihood = giwLikelihood(x,P,nu,V,z,zExp,R,posID,H,U)
% This is an internal function and may be removed in a future release.
% giwLikelihood calculates log-likelihood of measurements against GGIW 
% distributions.
% loglikelood is a Nx1 matrix, where N is number of components.
% x is PxN matrix representing Gaussian means of ggiw mixture
% P is PxPxN matrix representing Gaussian covariances of ggiw mixture
% v is N-element vector representing IW Degrees of Freedom of ggiw mixture.
% V is dxdxN matrix representing IW scale matrices of ggiw mixture
% z is SxM matrix representing the current set of measurements, where S is
% the size of measurement.
% R is a QxQxM matrix representing measurement noise of each measurement,
% where Q is the size of measurement noise.
% posID are indices of states which represent target positions
% H is a SxPxN matrix representing jacobian of measurement wrt states.
% U is a SxQxN matrix representing jacobian of measurement wrt noise.
% U is optional. 
% Not supplying U is interpreted as additive measurement noise i.e. S = Q, U = eye;

% Copyright 2018 The MathWorks, Inc.

%#codegen

if nargin < 10
    hasAMN = true;
else
    hasAMN = false;
end

numStates = size(x,2);
numMeas = size(z,2);

dataType = class(x);

loglikelihood = zeros(numStates,1,dataType);

% measurement in cell
modW = cast(numMeas,dataType);

% log(pi)
logPi = cast(1.1447,dataType);

% log(2);
logTwo = cast(0.6931,dataType);

% Extent dimensions
d = size(V,1);

% Constant across all states
t1 = d/2*(modW*logPi - log(modW)) + modW/2*(d+1)*logTwo;

% nu update
nuK = nu + modW;

% nuLin = (nu - d - 1)/2;
nuLin = (nu - d - 1)/2;
nuLinK = (nuK - d - 1)/2;

% scaling
rho = cast(0.25,dataType);

% R: measurement noise
Rmean = mean(R,3);
zMean = mean(z,2);
   
% measurement spread
meanError = bsxfun(@minus,z,zMean);
Zkw = meanError*meanError';

for i = 1:numStates
   Hi = H(:,:,i);
   Pi = P(:,:,i);
   Vi = V(:,:,i);
   
   % Jacobian of measurement wrt position
   Bkcheck = Hi(:,posID);
   
   % Measurements which have no relation with position or extent cannot be
   % evaluated against Inverse-Wishart distribution. Use a Gaussian
   % distribution governed by P. 
   nonExtent = all(abs(Bkcheck) < eps(dataType),2);
   
   % Meausurements with no relation to extent
   zNonEx = z(nonExtent,:);
   % Expected measurements
   zExpNonExtent = zExp(nonExtent,i);
   
   % Calculate likelihood using only Gaussian distribution
   if ~isempty(zNonEx)
       RnonEx = R(nonExtent,nonExtent,:);
       HnonEx = Hi(nonExtent,:);
       if ~hasAMN
           Ui = U(:,:,i);
           UnonEx = Ui(nonExtent,:);
           gaussLikelihood = fusion.internal.gaussCellLikelihood(Pi,zNonEx,zExpNonExtent,HnonEx,RnonEx,dataType,UnonEx);
       else
           gaussLikelihood = fusion.internal.gaussCellLikelihood(Pi,zNonEx,zExpNonExtent,HnonEx,RnonEx,dataType);
       end 
   else
       gaussLikelihood = cast(0,dataType);
   end
   
   % Measurements related to extent. Calculate likelihood using GIW
   % distribution.
   zExpExtent = zExp(~nonExtent,i);
   
   if ~isempty(zExpExtent)
       % Jacobian of extent measurement wrt position
       Bk = Bkcheck(~nonExtent,:);

       % Jacobian of position wrt extent measurement
       Bk2 = eye(d,dataType)/Bk;

       % Expected extent transformed into measurement space.
       Xhat = Bk*(Vi/(nu(i) - 2*d - 2))*Bk';
       % Xhat must be positive definite. For measurements with small 
       % jacobians, this can be "numerically" false
       sqrtXhat = sqrtmDiag(Xhat);
       
       % Difference between mean and expected
       zE = zExpExtent;
       error = zMean(~nonExtent) - zE;
       N = error*error';
        
       ZkwExtent = Zkw(~nonExtent,~nonExtent);
       
       % Total covariance in measurement space
       if hasAMN
           Rhat = rho*Xhat + Rmean(~nonExtent,~nonExtent);
       else   
           Ui = U(:,:,i);
           Uextent = Ui(~nonExtent,:);
           Rhat = rho*Xhat + Uextent*Rmean*Uextent';
       end
       sqrtRhat = sqrtmDiag(Rhat);
       
       % Total residual covariance in measurement space
       Hextent = Hi(~nonExtent,:);
       S = Hextent*Pi*Hextent' + Rhat/modW;
       sqrtS = sqrtmDiag(S);
       
       % Zhat and Nhat terms for scale matrix update. Transformed back into
       % extent/position space.
       sqrtXinvR = sqrtXhat/sqrtRhat;
       sqrtXinvS = sqrtXhat/sqrtS;
       
       Nhat = Bk2*(sqrtXinvS*N*sqrtXinvS')*Bk2';
       Zhat = Bk2*(sqrtXinvR*ZkwExtent*sqrtXinvR')*Bk2';
       Vk = Vi + Nhat + Zhat;
       t2 = nuLin(i)*log(det(Vi)) - nuLinK(i)*log(det(Vk));
       t3 = fusion.internal.multVarGammaln(nuLinK(i),d) - fusion.internal.multVarGammaln(nuLin(i),d);
       t4 = modW*log(det(sqrtXhat)) - (modW - 1)*log(det(sqrtRhat)) - log(det(sqrtS));
       giwLikelihood = real(t1 + t2 + t3 + t4);
   else
       giwLikelihood = cast(0,dataType);
   end
   loglikelihood(i) = giwLikelihood + gaussLikelihood;
end

end

function sqrtP = sqrtmDiag(P)
    % Compute the square root of a matrix using eigenvalue decomposition
    % and ensuring positive definiteness.
    [~,Q,D] = fusion.internal.ensurePosDefMatrix(P);
    sqrtP = real(Q*sqrt(D)*Q');
end