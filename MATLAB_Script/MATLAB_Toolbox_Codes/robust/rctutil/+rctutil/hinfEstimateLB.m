function GAMXE = hinfEstimateLB(A,B1,B2,C1,D11,D12,Ts)
% Estimates smallest GAMMA for which Riccati solution Xoo exists.

%   Author(s): P. Gahinet
%   Copyright 2018 The MathWorks, Inc.

% Eliminate cancelling dynamics
[A,B,C1] = smreal(A,[B1 B2],C1,[]);
D = [D11 D12];
nX = size(A,1);
[nE,nD] = size(D11);
nU = size(D12,2);

% Test frequencies
if Ts==0
   s = [0 ; complex(0,logspace(-5,5,21).')];
else
   s = exp(complex(0,pi*logspace(-8,0,21).'));
end
   
% Stay away from poles of P12 (frequency response inaccurate -> 
% gamX is garbage, see CSE2)
p = eig(A);
dist = abs(s-p.')./(1+abs(s));
s = s(min(dist,[],2)>1e-3);
if Ts==0
  s = [s ; Inf];
end

GAMXE = 0;
nsv = min(nE,nU);
if nsv>0
   E = eye(nX);
   for ct=1:numel(s)
      if isinf(s(ct))
         H = D;
      else
         H = [D11 D12] + (C1 / (s(ct)*E-A)) * B;
      end
      H12 = H(:,nD+1:nD+nU);
      H12 = H12 ./ localColNorm(H12);   % invariance under U scaling
      [U,S,~] = svd(H12);
      S = [diag(S(1:nsv,1:nsv)) ; zeros(nE-nsv,1)];
      U2 = U(:,S<=1e-8*S(1));  % P12~*P12 drops numerical rank
      GAMXE = max(GAMXE,norm(U2'*H(:,1:nD)));
   end
end

%------------------------------
function cnorm = localColNorm(M)
% Computes norm of columns of M
nc = size(M,2);
cnorm = zeros(1,nc);
for ct=1:nc
   cnorm(ct) = norm(M(:,ct));
end
cnorm(cnorm==0) = 1;