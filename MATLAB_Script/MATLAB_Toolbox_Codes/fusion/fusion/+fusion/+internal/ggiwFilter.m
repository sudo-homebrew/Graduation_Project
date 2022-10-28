% This is an internal class and may be removed in future release.
% ggiwFilter provides static methods for predicting, correcting and
% likelihood calculations of multiple GGIW distributions.

% Copyright 2018 The MathWorks, Inc.
classdef ggiwFilter
    %#codegen
    methods (Static)
        function [x,P,alpha,beta,nu,V] = predict(x,P,alpha,beta,nu,V,nk,f,df,M,Q,dT,tau,hasAPN)
            if nargin < 14
                hasAPN = true;
            end
            % Predict each distribution.
            [x,P] = fusion.internal.gaussEKFilter.predict(x,P,Q,f,df,hasAPN,dT);
            [alpha,beta] = fusion.internal.gammaFilter.predict(alpha,beta,nk);   
            [nu,V] = fusion.internal.iwFilter.predict(nu,V,x,tau,dT,M);
        end
        
        function [xk,Pk,alphak,betak,vk,Vk] = correct(x,P,alpha,beta,v,V,z,zExp,R,posID,H,U)
            if nargin < 12
                hasAMN = true;
            else
                hasAMN = false;
            end
            % Number of measurements
            modW = size(z,2);
            % Size of measurement
            p = size(z,1);
            % Number of states
            n = size(x,2);
            % mean of measurements
            zMean = mean(z,2);
            % mean of measurement noises
            Rmean = mean(R,3);
            % dimension of object
            d = size(V,1);
            % spread of measurements
            zE = bsxfun(@minus,z,zMean);
            Zkw = zE*zE';
            
            % Pre-allocate variables for correction of each distribution.
            dataType = class(x);
            N = zeros(d,d,n,dataType);
            Zk = zeros(d,d,n,dataType);
            S = zeros(p,p,n,dataType);
            r = zeros(p,n,dataType);
            
            % Initialize non-extent measurements
            nonExtent = false(p,1);
            
            % Correct each component
            for i = 1:n
                % Expected measurement
                zExpi = zExp(:,i);
                % Jacobian wrt sate
                Hi = H(:,:,i);
                
                % Jacobian of measurement wrt position
                Bkcheck = H(:,posID);
                
                % non-extent measurements
                nonExtent = all(abs(Bkcheck) < eps(dataType),2);
                
                % expected measurement split into two parts
                zExpExtent = zExpi(~nonExtent);
                zExpNonExtent = zExpi(nonExtent);
                
                % Correct Gaussian distribution with non-extent
                % measurements. These measurements are independent and
                % hence the correction can be sequential.
                xi = x(:,i);
                Pi = P(:,:,i);
                
                % Change the state in the input, IW calculations should be
                % performed using xi,Pi instead of xk,Pk.
                if any(nonExtent)
                    zNonExtent = z(nonExtent,:);
                    RNonExtent = R(nonExtent,nonExtent,:);
                    HnonExtent = Hi(nonExtent,:);
                    if hasAMN
                        [x(:,i),P(:,:,i)] = gaussCorrectNonExtent(xi,Pi,zNonExtent,zExpNonExtent,HnonExtent,RNonExtent);
                    else
                        Ui = U(:,:,i);
                        UnonExtent = Ui(nonExtent,:);
                        [x(:,i),P(:,:,i)] = gaussCorrectNonExtent(xi,Pi,zNonExtent,zExpNonExtent,HnonExtent,RNonExtent,UnonExtent);
                    end
                end
                
                % Jacobian of position wrt measurement
                Bk = Bkcheck(~nonExtent,:);
                % Jacobian of measurement wrt position
                Bk2 = eye(d)/Bk;
                
                % Current scale matrix
                Vi = V(:,:,i);
                % Current degrees of freedom
                vi = v(i);
                % Expected extent transformed into measurement-space.
                Xhat = Bk*(Vi/(vi - 2*d - 2))*Bk';
                % The matrix may have a tendency to have -eps eigen values.
                % Make sure sqrt is real and matrix is non-singular.
                sqrtXhat = sqrtmDiag(Xhat);
                
                % Error between mean and expected measurement
                zError = zMean(~nonExtent,:) - zExpExtent;
                r(~nonExtent,i) = zError;
                
                % Covariance spread of error
                Nz = zError*zError';
                
                % Scaling parameter to approximate Uniform distribution
                % with Gaussian.
                rho = cast(0.25,dataType);
                % Total covariance in measurement space.
                RmeanExtent = Rmean(~nonExtent,~nonExtent);
                if hasAMN
                    Rhat = rho*Xhat + RmeanExtent;
                else
                    Ui = U(:,:,i);
                    Uextent = Ui(~nonExtent,:);
                    Rhat = rho*Xhat + Uextent*RmeanExtent*Uextent';
                end
                sqrtRhat = sqrtmDiag(Rhat);
                
                % Residual covariance including contribution of
                % state-space.
                Hextent = Hi(~nonExtent,:);
                Smat = real(Hextent*Pi*Hextent' + Rhat/modW);                
                sqrtS = sqrtmDiag(Smat);
                
                S(~nonExtent,~nonExtent,i) = real(Smat);
                % N and Z terms for Inverse-Wishart distribution
                % correction.
                sqrtXinvS = sqrtXhat/sqrtS;
                N(:,:,i) = Bk2*sqrtXinvS*Nz*sqrtXinvS'*Bk2';
                
                ZkwExtent = Zkw(~nonExtent,~nonExtent); 
                sqrtXinvR = sqrtXhat/sqrtRhat;
                Zk(:,:,i) = Bk2*sqrtXinvR*ZkwExtent*sqrtXinvR'*Bk2';
            end
            % Correct Gaussian with extent residuals and covariances
            rExt = r(~nonExtent,:);
            SExt = S(~nonExtent,~nonExtent,:);
            HExt = H(~nonExtent,:,:);
            [xk,Pk] = fusion.internal.gaussEKFilter.correct(x,P,rExt,SExt,HExt);
            [alphak,betak] = fusion.internal.gammaFilter.correct(alpha,beta,modW);
            [vk,Vk] = fusion.internal.iwFilter.correct(v,V,N,Zk,modW);
        end
        
        function lhood = likelihood(x,P,v,V,z,zExp,R,posID,H,varargin)
            lhood = fusion.internal.giwLikelihood(x,P,v,V,z,zExp,R,posID,H,varargin{:});
        end
    end
end

function [xk,Pk] = gaussCorrectNonExtent(x,P,z,zExp,H,R,Ui)
hasAMN = nargin == 6;
numMeas = size(z,2);
zTotal = z(:);
zExpTotal = repmat(zExp(:),[numMeas 1]);
rTotal = zTotal - zExpTotal;
HTotal = repmat(H,[numMeas 1]);
p = size(z,1);
RTotal = zeros(p*numMeas,class(z));
for i = 1:numMeas
    index = ((i-1)*p + 1):(i*p);
    if hasAMN
        RTotal(index,index) = R(:,:,i);
    else
        RTotal(index,index) = Ui*R(:,:,i)*Ui;
    end
end
STotal = HTotal*P*HTotal' + RTotal;
[xk,Pk] = fusion.internal.gaussEKFilter.correct(x,P,rTotal,STotal,HTotal);

end

function sqrtP = sqrtmDiag(P)
    % Compute the square root of a matrix using eigenvalue decomposition
    % and ensuring positive definiteness.
    [~,Q,D] = fusion.internal.ensurePosDefMatrix(P);
    sqrtP = real(Q*sqrt(D)*Q');
end
