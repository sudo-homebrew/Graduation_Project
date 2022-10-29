function [KBEST,Su,Sy,dD12,dD21] = hinfStatic(D11,D12,D21,GMIN,GMAX,OPTS)
% Solves static H-infinity problem:
%
%    minimize  || D11 + D12 * DK / (I-D22*K) * D21 ||
%       DK
%
% The optimal value is
%
%    GOPT = max( ||(I-D12*D12+)*D11|| , ||D11*(I-D21"*D21)|| ).

%   Author(s): P. Gahinet
%   Copyright 2018 MathWorks and Musyn Inc. and 
[nE,nD] = size(D11);
nU = size(D12,2);
nY = size(D21,1);

% Scale D12/D21 to normalize their columns/rows
if OPTS.ScaleXUY
   Su = localColNorm(D12);    Su(Su==0) = 1;  Su = pow2(-round(log2(Su))).';
   Sy = localColNorm(D21.');  Sy(Sy==0) = 1;  Sy = pow2(-round(log2(Sy))).';
   D12 = D12 .* Su';
   D21 = Sy .* D21;
else
   Su = 1;  Sy = 1;
end

% Regularize D12 and D21
% Note: D12/D21 must have full column/row ranks when OPTS.TOLREG=0, 
if OPTS.TOLREG>0
   dD12 = localRegularize(D12,OPTS.TOLREG);
   dD21 = localRegularize(D21',OPTS.TOLREG)';
   D12 = [D12 ; dD12];
   D21 = [D21 , dD21];
   D11 = blkdiag(D11,zeros(size(dD12,1),size(dD21,2)));
   [nE,nD] = size(D11);
else
   dD12 = zeros(0,nU);   dD21 = zeros(nY,0);
end

% Reduce to minimizing || [X B;C D] || over X
[q12,r12] = qr(D12);
[q21,r21] = qr(D21');
D11t = q12'*D11*q21;
B = D11t(1:nU,nY+1:nD);
C = D11t(nU+1:nE,1:nY);
D = D11t(nU+1:nE,nY+1:nD);
GOPT = max(norm([B;D]),norm([C D]));

if GMAX<GOPT
   % No feasible GAM in [GMIN,GMAX]
   KBEST = [];
else
   GAM = max(GMIN,GOPT); % optimal value
   if norm(D11)<GAM
      DK = zeros(nU,nY);
   else
      DK = r12(1:nU,:)\(ltipack.parrott(B,C,D,GAM)-D11t(1:nU,1:nY))/r21(1:nY,:)';
   end
   KBEST = struct('A',[],'B',zeros(0,nY),'C',zeros(nU,0),'D',DK,'GAM',GAM,...
      'X',[],'Y',[],'Ku',zeros(nU,0),'Kw',zeros(nD,0),'Lx',zeros(0,nY),'Lu',DK);
end

%-------------------------------------------------
function cnorm = localColNorm(M)
% Computes norm of columns of M
nc = size(M,2);
cnorm = zeros(1,nc);
for ct=1:nc
   cnorm(ct) = norm(M(:,ct));
end

function dD12 = localRegularize(D12,TOLREG)
% Regularizes D12 if necessary
[nE,nU] = size(D12);
[~,S,V] = svd([D12;zeros(nU-nE,nU)]);
S = diag(S(1:nU,:))'; % row
Smin = sqrt(TOLREG);  % assumes SCALE=1
jreg = find(S < Smin);
if isempty(jreg)
   dD12 = zeros(0,nU);
else
   dD12 = Smin * V(:,jreg)';
end