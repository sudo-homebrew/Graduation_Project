function [wL,wR] = findIntervalHam(A,B,C,D,Ts,VLMI,carveUB,w,dw)
% Compute interval [wL,wR] containing w where mu upper bound LMI hold
% for the constant D,G scaling Dr,Dc,Gcr,Grc. The LMI is
%    M'*Dr*M + j*(Gcr*M-M'*Grc) < ub^2*DcV+DcF
% where M = D+C*(j*w*I-A)\B. Note that the D scalings are complex in 
% general. This LMI condition is transformed to a sector bound
%     X(jw)'*J*X(jw) < 0 with J=[I 0;0 -I]
% and the associated Hamiltonian pencil is used to compute wL,wR.
% This eliminates all squaring-up occurring with the explicit Hamiltonian.
% Note that USQDC = ub^2 * DcV + DcF.

%   Copyright 1986-2020 The MathWorks, Inc.

% D,G scalings computed for frequencies [w-dw,w,w+dw] for w>0
if nargin<9 || isnan(dw)
   dw = 0.1*w;
end
Dr = VLMI.Dr;
usqDc = VLMI.DcF + carveUB^2*VLMI.DcV;
Gcr = VLMI.Gcr;

nx = size(A,1);
[ny,nu] = size(D);
% LMI is [M(s);I]' * Q * [M(s);I] < 0
Q = [Dr -1i*Gcr';1i*Gcr -usqDc];
% Q = diag(S)*QN*diag(S)
[S,QN] = ltipack.util.symscale(Q);
% QN = W1*W1'-W2*W2'
[~,W1,W2] = ltipack.getSectorData(QN,[]); 
% X(s) = [W1,W2]'*diag(1./S)*[M(s);I] = (A,BX,CX,DX)
BX = B;
aux = [W1,W2]' * (S .\ [C D;zeros(nu,nx) eye(nu)]);
CX = aux(:,1:nx);
DX = aux(:,nx+1:nx+nu);
J = diag([ones(1,size(W1,2)),-ones(1,size(W2,2))]);

% Normalize the columns of DX
sD = 1./sum(abs(DX),1);
DX = lrscale(DX,[],sD);
BX = lrscale(BX,[],sD);

% Equalize the norms of BX and CX
bn = norm(BX,1);  cn = norm(CX,1);
if bn>0 && cn>0
   tau = sqrt(bn/cn);
   BX = BX/tau;  CX = CX*tau;
end

% Form and scale Hamiltonian pencil
ta = ltipack.scalePencil(norm(A,1),norm(BX,1),1);
[alpha,beta] = ltipack.eigSHH(ta^2*A,ta^2*eye(nx),zeros(nx),zeros(nx),...
   [zeros(nu) DX';DX J],[ta*BX zeros(nx,ny+nu)],[zeros(nx,nu) ta*CX']);
ix = find(beta~=0);
eH = alpha(ix)./beta(ix);

%    HH = [ta^2*A zeros(nx) ta*BX zeros(nx,ny+nu);...
%       zeros(nx) -ta^2*A' zeros(nx,nu) ta*CX';...
%       zeros(nu,nx) -ta*BX' zeros(nu) DX';...
%       ta*CX zeros(ny+nu,nx) DX J];
%    JJ = diag([ta^2*ones(1,2*nx),zeros(1,ny+2*nu)]);
%    eH0 = eig(HH,JJ);
%    eH0 = eH0(isfinite(eH0));
%    %[dsort(eH) dsort(eH0)]

% Find eigenvalues close to jw axis
ImagThresholdRelative = 1e-6;
idx = (abs(real(eH))<ImagThresholdRelative*abs(eH));
wCand = imag(eH(idx));
wL = max( [0 ; wCand(wCand<w)] );
wR = min( [wCand(wCand>w) ; Inf] );

% Note: The Hamiltonian section can fail when the R-index is flat
% and close to 1 over a large frequency band. In such case, there is an
% infinite number of frequencies that are "quasi-crossovers", which
% manifests as clusters of eigenvalues with high sensitivity and random
% real parts. This is similar to the situation when the gain is very flat
% over a large region in the Hoo norm computation. The "Two-Cart and 
% Spring System" example is a good illustration of this issue.

% if w==0
%    X = ss(A,BX,CX,DX); sectorplot(frd(X,logspace(-6,0,500)),J)
% else
%    X = ss(A,BX,CX,DX); sectorplot(frd(X,logspace(log10(w)-1,log10(w)+1,500)),J)
% end
% set(gca,'Ylim',[.5 2])

% Validate wL
if wL==0 && w>0
   % Safeguard: wL=0 could be the result of missing the left crossover
   wTest = (w-dw) * [1 1e-1 1e-2 1e-4 1e-8 0];
   wL = wTest(1);
   for ct=1:numel(wTest)
      if localSatisfiesLMI(A,B,C,D,Ts,wTest(ct),Dr,usqDc,Gcr)
         wL = wTest(ct);
      else
         break
      end
   end
%    if wL>0
%       fprintf('wL=0 changed to wL=%.3g\n',wL)
%    end
end
while ~localSatisfiesLMI(A,B,C,D,Ts,wL,Dr,usqDc,Gcr)
   % Can get bad crossovers for the same reason we can miss crossovers
   % fprintf('wL=%.5g falsified\n',wL)
   wL = (w+wL)/2;
end   
   
% Validate wR
if wR==Inf
   % Safeguard: wR=Inf could be the result of missing the right crossover
   % Needed for "Two-Cart and Spring System"
   wTest = (w+dw) * [1 1e1 1e2 1e4 1e8 Inf];
   wR = wTest(1);
   for ct=1:numel(wTest)
      if localSatisfiesLMI(A,B,C,D,Ts,wTest(ct),Dr,usqDc,Gcr)
         wR = wTest(ct);
      else
         break
      end
   end
%    if wR<Inf
%       fprintf('wR=Inf changed to wR=%.3g\n',wR)
%    end
end
while ~localSatisfiesLMI(A,B,C,D,Ts,wR,Dr,usqDc,Gcr)
   % fprintf('wR=%.5g falsified\n',wR)
   % Note: wR cannot be Inf here (see above)
   wR = (w+wR)/2;
end   

% if wL>w-dw || wR<w+dw
%    fprintf('[w-dw,w+dw] = [%.5g,%.5g], [wL,wR] = [%.5g,%.5g]\n',w-dw,w+dw,wL,wR)
% end

%----------------------------------------------------------
function PF = localSatisfiesLMI(A,B,C,D,Ts,w,Dr,usqDc,Gcr)
% Turns true if LMI is not statisfied
zeroTol = 1e-6;
M = rctutil.freqresp(A,B,C,D,Ts,w);
T1 = M'*Dr*M;
T2 = Gcr*M;   T2 = T2-T2';
LMI = T1 + complex(-imag(T2),real(T2)) - usqDc;
eLMI = eig((LMI+LMI')/2);
PF = max(eLMI)<zeroTol*(norm(T1,1)+norm(T2,1)+norm(usqDc,1));


