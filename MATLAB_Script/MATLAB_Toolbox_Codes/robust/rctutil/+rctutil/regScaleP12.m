function [Sx,Su,dC1,dD12,INFO] = regScaleP12(A,B1,B2,C1,D11,D12,Ts,GAM,OPTS)
% Regularizes and scales P12 to minimize the sensitivity of the Riccati 
% equation for Xinf(GAM).
%
% Note: 
%   * Sensitivity is measured in terms of the eigenvalues and eigenvectors
%     of the Xinf Riccati pencil. Scaling seeks to reduce the sensitivity
%     of eigenvalues close to the stability boundary.
%   * Both the state vector x and input vector u are scaled.

%   Author(s): P. Gahinet
%   Copyright 2018 The MathWorks, Inc.
[nx,nu] = size(B2);
nd = size(B1,2);
GAM = max(1e-2,2*GAM);
B1 = B1/GAM;  D11 = D11/GAM;  % absorb GAM into B1,D11
INFO.P12 = ss(A,B2,C1,D12,Ts);

if OPTS.TOLREG==0
   % Skip regularization: Just scale and exit
   [Sx,Su] = localRicScale(A,B1,B2,C1,D11,D12,Ts);
   dC1 = zeros(0,nx);  dD12 = zeros(0,nu);
   INFO.dP12 = ss(A,B2,dC1,dD12,Ts);
   
else
   % Compute initial regularization that is independent of state scaling
   % and keeps the perturbation dP12 of P12 small
   [dC1,dD12] = localRegP12(A,B2,C1,D12,Ts);
   ne2 = size(dC1,1);
   C1R = [C1 ; dC1];  D11R = [D11 ; zeros(ne2,nd)];  D12R = [D12 ; dD12];
   if OPTS.Instrument
      % For validation
      INFO.dP12 = ss(A,B2,dC1,dD12,Ts);
      INFO.XSENS1 = hinfricsens(A,[],B1,B2,C1R,D11R,D12R,Ts,{1e-8,1e8},0,false);
   end
   
   % Scale regularized model with P12 -> [P12 ; dP12] and P11 -> [P11 ; 0]
   [Sx0,Su0] = localRicScale(A,B1,B2,C1R,D11R,D12R,Ts);
   AS = lrscale(A,Sx0,1./Sx0);
   B1S = lrscale(B1,Sx0,[]);
   B2S = lrscale(B2,Sx0,Su0);
   C1S = lrscale(C1R,[],1./Sx0);
   D12S = lrscale(D12R,[],Su0);
   if OPTS.Instrument
      INFO.XSENS1S = hinfricsens(AS,[],B1S,B2S,C1S,D11R,D12S,Ts,{1e-8,1e8},0,false);
   end
   
   % Compute minimalist regularization in scaled coordinates
   C1S = lrscale(C1,[],1./Sx0);
   D12S = lrscale(D12,[],Su0);
   [dC1,dD12] = rctutil.regP12(AS,B2S,C1S,D12S,Ts,OPTS);
   C1SR = [C1S ; dC1];
   D12SR = [D12S ; dD12];
   ne2 = size(dC1,1);
   D11SR = [D11 ; zeros(ne2,nd)];
   if OPTS.Instrument
      INFO.XSENS2 = hinfricsens(AS,[],B1S,B2S,C1SR,D11SR,D12SR,Ts,{1e-8,1e8},0,false);
      INFO.dP12R = ss(AS,lrscale(B2S,[],1./Su0),dC1,lrscale(dD12,[],1./Su0),Ts);
   end
   % undo scaling
   dC1 = lrscale(dC1,[],Sx0);
   dD12 = lrscale(dD12,[],1./Su0);
   
   % Scale again for final regularization
   [Sx,Su] = localRicScale(AS,B1S,B2S,C1SR,D11SR,D12SR,Ts);
   AS = lrscale(AS,Sx,1./Sx);
   B1S = lrscale(B1S,Sx,[]);
   B2S = lrscale(B2S,Sx,Su);
   C1S = lrscale(C1SR,[],1./Sx);
   D12S = lrscale(D12SR,[],Su);
   if OPTS.Instrument
      INFO.XSENS2S = hinfricsens(AS,[],B1S,B2S,C1S,D11SR,D12S,Ts,{1e-8,1e8},0,false);
   end
   Sx = Sx0 .* Sx;
   Su = Su0 .* Su;
end


%--------------------------------------------------
function [dC1,dD12] = localRegP12(A,B2,C1,D12,Ts)
% Initial (crude) regularization of zeros of P12 on stability boundary
[nx,nu] = size(B2);
p = eig(A);
if Ts==0
   wmax = 1 + max([0 ; abs(p)]);
else
   wn = abs(log(p));
   wmax = 0.5 + max([0 ; wn(wn<=pi)]);
end
E = eye(nx);

% Pick regularization of D12 to
% 1) prevent dynamics far beyond wmax
% 2) ensure rho(A) * ||inv([0 D12';D12 -I])|| < 1/eps where
%    rho(A) is a proxy for the best-case value of ||MX|| after scaling
if Ts==0
   s = complex(0,wmax);
else
   s = exp(complex(0,wmax));
end
delta = 1e-3 * localColNorm(D12 + C1 * ((s*E-A) \ B2));
delta = max(delta,sqrt(eps*wmax));
dD12 = [zeros(nx,nu) ; diag(delta)];

% Evaluate P12 response away from poles
if Ts==0
   s = complex(0,wmax*logspace(-8,0,9).');
else
   s = exp(complex(0,wmax*logspace(-8,0,9).'));
end
dist = abs(s-p.')./(1+abs(s));
s = s(min(dist,[],2)>1e-2);
   
tau = norm(C1,1);
if tau==0
   tau = 1;
end
delta = (1e-2*tau) * ones(1,nx);
for ct=1:numel(s)
   K = (s(ct)*E-A) \ B2;
   delta = min(delta,0.01*norm(D12+C1*K,1)./localColNorm(K'));
end
dC1 = [diag(delta) ; zeros(nu,nx)];


%--------------------------------------------------------------
function [Sx,Su,INFO] = localRicScale(A,B1,B2,C1,D11,D12,Ts)
% Scales states and controls to minimize sensitivity of X Riccati equations.
nX = size(A,1);  nX2 = 2*nX;
[nE,nD] = size(D11);
nU = size(B2,2);
m = nU+nE+nD;
E = eye(nX);

% Pick frequency grid for sensitivity minimization
cfreqs = localFreqGrid(A,Ts);

% Form pencil
if Ts==0
   MX = [zeros(nX) A B2 B1 zeros(nX,nE);A' zeros(nX,nX+nU+nD) C1';...
      B2' zeros(nU,nX+nU+nD) D12';B1' zeros(nD,nX+nU) -eye(nD) D11';...
      zeros(nE,nX) C1 D12 D11 -eye(nE)];
   NX = blkdiag([zeros(nX) E;-E' zeros(nX)],zeros(m));
else
   MX = [zeros(nX) A B2 B1 zeros(nX,nE);-E' zeros(nX,nX+nU+nD) C1';...
      zeros(nU,nX2+nU+nD) D12';zeros(nD,nX2+nU) -eye(nD) D11';...
      zeros(nE,nX) C1 D12 D11 -eye(nE)];
   NX = [[zeros(nX) E;-A' zeros(nX);-B2' zeros(nU,nX);...
      -B1' zeros(nD,nX);zeros(nE,nX2)] zeros(nX2+m,m)];
end

if Ts==0
   M1 = abs(MX);
else
   M1 = abs(MX)+abs(NX);
end

% Build term N1 = sum_k(| inv(M-s(k)*N) |)
N1 = zeros(size(M1));
for ct=1:numel(cfreqs)
   if isinf(cfreqs(ct))
      Zi = blkdiag(zeros(2*nX),ltipack.util.safeMinv(MX(nX2+1:end,nX2+1:end)));
   else
      Z = MX-cfreqs(ct)*NX;
      if Ts~=0
         Z(:,1:nX) = Z(:,1:nX)/cfreqs(ct);  % make Hermitian
         Z = (Z+Z')/2;
      end
      Zi = ltipack.util.safeMinv(Z);
   end
   % Skip frequencies near singularities (including w=Inf)
   if all(isfinite(Zi),'all')
      N1 = N1 + abs(Zi);
   end
end

% Scale
[Sx,Su] = hamgp(1,nX,nU,M1,N1);
% [Sx,Su,INFO] = hamgp(1,nX,nU,M1,N1);
% [S1,INFO1] = symgp(2*nX+nU,M1,N1);
% [S2,INFO2] = SymScale(2*nX+nU,M1,N1);
% if abs(1-INFO1.finalcost/INFO2.FinalCost)>1e-3 || norm(S1-S2)>0
%    fprintf('final cost = %.2e (symgp) vs. %.2e (SymScale)\n',INFO1.finalcost,INFO2.FinalCost)
% end
% if INFO2.FinalCost>1.1*INFO.finalcost
%    fprintf('final cost = %.2e (hamgp) < %.2e (SymScale)\n',INFO.finalcost,INFO2.FinalCost)
% end
% [Sx1,Su1,~,INFO1] = HamScale(nX,nU,M1,N1);
% gapgp = norm(log2(Sx)-log2(Sx1),1)+norm(log2(Su)-log2(Su1),1);
% fprintf('gap=%d, cost=%.2e\n',gapgp,abs(1-INFO1.FinalCost/INFO.finalcost))


function cnorm = localColNorm(M)
% Computes norm of columns of M
nc = size(M,2);
cnorm = zeros(1,nc);
for ct=1:nc
   cnorm(ct) = norm(M(:,ct));
end

function cfreqs = localFreqGrid(A,Ts)
% Generates frequency grid covering the dynamic range
p = eig(A);
if Ts==0
   wn = abs(p);
   wn = wn(wn>1e-6);
   if isempty(wn)
      fmin = 1e-6;   fmax = 1e6;
   else
      fmin = 0.1*min(wn);   fmax = 10*max(wn);
   end
else
   wn = abs(log(p));
   wn = wn(wn>1e-6 & wn<pi);
   if isempty(wn)
      fmin = 1e-6*pi;   fmax = pi/2;
   else
      fmin = 0.1*min(wn);   fmax = min(pi/2,10*max(wn));
   end
end
lfmin = log10(fmin);  lfmax = log10(fmax);
fgrid = logspace(lfmin,lfmax,ceil(lfmax-lfmin));
if Ts==0
   cfreqs = complex(0,[0 fgrid Inf]);
else
   cfreqs = exp(complex(0,[0 fgrid pi]));
end
