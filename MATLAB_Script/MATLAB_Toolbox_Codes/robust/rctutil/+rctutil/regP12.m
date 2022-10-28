function [dC1,dD12] = regP12(A,B2,C1,D12,Ts,OPTS)
% Regularization of P12 and associated H2-type Riccati equation.
%
% Bounds M-t*N safely away from singularity on the imaginary axis or 
% unit circle and keeps the stable and anti-stable invariant subspaces 
% well separated.

%   Copyright 1986-2020 The MathWorks, Inc.
nX = size(A,1);
[nE,nU] = size(D12); 
% REVISIT: ADD DESCRIPTOR SUPPORT
E = eye(nX);
TOLREG = OPTS.TOLREG;
if TOLREG==0
   % Skip regularization
   dC1 = zeros(0,nX);   dD12 = zeros(0,nU);   return
end
if OPTS.Instrument
   if nE<nU
      fprintf('Regularizing normal-rank-deficient P12/P21\n')
   else
      fprintf('Regularizing P12/P21\n')
   end
end

% CT ONLY: Regularize D12 (Zeros of P12 or M-s*N at s=Inf)
nEr = nE;
if Ts==0
   % REVISIT: Descriptor should take into account E
   [~,S,V] = svd([D12;zeros(nU-nE,nU)]);
   S = diag(S(1:nU,:))'; % row
   % Regularize to ensure no dynamics much faster than WMAX (use a fraction
   % of the gain at WMAX rather than some multiple of WMAX which is too
   % sensitive to the rolloff slope)
   SCALE = 1+norm([A B2;C1 D12],1);
   wmax = 1+norm(A,1);
   Smin = sqrt(TOLREG*SCALE);
   if OPTS.LimitGain
      Smin = max( Smin , 1e-4 * norm(evalfr(ss(A,B2,C1,D12),1i*wmax)) );
   end
   jreg = find(S < Smin);
   if ~isempty(jreg)
      nreg = numel(jreg);
      C1 = [C1 ; zeros(nreg,nX)];
      D12 = [D12 ; Smin * V(:,jreg)'];
      %D12 = [D12 ; (V(:,jreg) .* Smin(jreg))'];
      nEr = nEr+nreg;
      if OPTS.Instrument
         fprintf('    Regularized D12/D21\n')
      end
   end
end

if OPTS.Instrument
   fprintf('    A=%.3g, C1=%.3g, B2=%.3g, D12=%.3g, B2D12+=%.3g\n',...
      norm(A), norm(C1), norm(B2), norm(D12), norm(B2*pinv(D12)));
end

% Form M-t*N and regularize at s=0 (continuous time) and z=1,-1 (discrete 
% time). This step ensures that the pencil is regular and that the 
% computed eigenvalues are meaningful.
% Note: s=Inf already regularized in previous step
%--------------------------------------------------------------------------
nX2 = 2*nX;  m = nU+nEr;   ireg = nX+1:nX2+nU;
if Ts==0
   M = [zeros(nX) A B2 zeros(nX,nEr);A' zeros(nX,nX+nU) C1';...
      B2' zeros(nU,nX+nU) D12';zeros(nEr,nX) C1 D12 -eye(nEr)];
   N = [zeros(nX) E zeros(nX,m);-E' zeros(nX,nX+m) ; zeros(m,nX2+m)];
   [DELTA,MSCALE] = localRegPert(M,ireg,TOLREG,OPTS);
   NSCALE = norm(E,1);
else
   M = [zeros(nX) A  B2 zeros(nX,nEr);-E' zeros(nX,nX+nU) C1';...
       zeros(nU,nX2+nU) D12';zeros(nEr,nX) C1 D12 -eye(nEr)];
   N = [zeros(nX) E zeros(nX,m);-A' zeros(nX,nX+m) ; ...
      -B2' zeros(nU,nX+m) ; zeros(nEr,nX2+m)];
   [V1,S1] = localRegPert(M-N,ireg,TOLREG,OPTS);
   aux = M+N;  aux(1:nX,:) = -aux(1:nX,:);  % enforce symmetry
   [V2,S2] = localRegPert(aux,ireg,TOLREG,OPTS);
   DELTA = [V1,V2];
   MSCALE = max(S1,S2);  NSCALE = MSCALE;
end
if ~isempty(DELTA)
   % Regularize M
   M(ireg,ireg) = (TOLREG*MSCALE) * (DELTA*DELTA');
   if OPTS.Instrument
      disp('    Regularized at s=0 or z=1,-1')
   end
end

% Regularize M-t*N at eigenvalues near stability boundary
%--------------------------------------------------------
RealFlag = isreal(M) && isreal(N);
[MM,NN,~,~,V,W] = qz(M,N,'complex');
V = V ./ localColNorm(V);
W = W ./ localColNorm(W);
ev = ordeig(MM,NN);
% note s=0 and z=1,-1 already taken care of
% REVISIT: Can we be more clever? In problems like DT:IH, we do a bunch of
% SVDs for eigs near unit circle to add nothing. Should we do a gamma-cut
% for P12~*P12 for gamma=1/(TOLREG*||M||) to reduce search to interval 
% [wmin,wmax] and weed out false positives
if Ts==0
   ix = find(abs(imag(ev))*NSCALE>TOLREG*MSCALE);
else
   evp = ev./abs(ev);
   ix = find(min(abs(evp-1),abs(evp+1))*NSCALE>TOLREG*MSCALE);
end
ev = ev(ix); V = V(:,ix);  W = W(:,ix);
if RealFlag
   ip = find(imag(ev)>0); 
   ev = ev(ip);  V = V(:,ip);  W = W(:,ip);
end
mod = abs(ev);
maxmod = max(mod(isfinite(mod)));
% Find eigenvalues that may become imaginary for small perturbation
% tMuSynthesisDemos:SimStab: no zeros near jw-axis but zero distribution
% drives TOLREG*cond(M-jw*N)>1 for 27<w<65 and TOLREG=1e-10
RoundOff = (TOLREG*MSCALE+mod*(eps*NSCALE)) ./ abs(diag(W'*N*V));
if Ts==0
   is = find(abs(real(ev))<min(0.1*maxmod,RoundOff));
else
   % Note: May pick z=1,-1 again
   is = find(abs(1-mod)<min(0.1,RoundOff));
end
% Regularizable eigs on stability boundary have the same left and right 
% eigenvectors and they lie in span([0;I;0]). 
Vr = [];
for ct=1:numel(is)
   ix = is(ct);
   e = ev(ix);
   if Ts==0
      sz = complex(0,imag(e));  % projection on jw axis
   else
      sz = e/abs(e);  % projection on unit circle
   end
   % Use power iter to refine estimate of smallest singular vector
   Ve = V(:,ix);
   Msz = M-sz*N;
   if Ts~=0
      Msz(1:nX,:) = Msz(1:nX,:)/sz;  % enforce symmetry
   end
   k = 0;
   while norm(Msz*Ve)>=TOLREG*MSCALE && k<3
      Ve = Msz\Ve;  Ve = Ve/norm(Ve);  k = k+1;
   end
   if norm(Msz*Ve)<TOLREG*MSCALE && norm(Ve(ireg))>0.99
      % Use singular direction
      Vr = [Vr , Ve(ireg)]; %#ok<*AGROW>
      if OPTS.Instrument
         Vr0 = localRegPert(Msz,ireg,TOLREG,OPTS);
         gap = 1-abs(Vr0'*Ve(ireg));
         fprintf('      Using eigenvector: k = %d, gap = %.3g\n',k,gap)
      end
   else
      % Use SVD in case the null space has dimension > 1
      if OPTS.Instrument
         disp('      SVD needed')
      end
      Vr = [Vr , localRegPert(Msz,ireg,TOLREG,OPTS)];
   end
end
if RealFlag
   Vr = [real(Vr) imag(Vr)];
end
if OPTS.Instrument && ~isempty(Vr)
   disp('    Regularized remaining zeros')
end
DELTA = [DELTA Vr];

% Regularize C1
if ~isempty(DELTA)
   % Extract dominant set of orthogonal directions
   [DELTA,S,~] = svd(DELTA,0);
   S = diag(S(:,1:size(S,1)));
   DELTA = sqrt(MSCALE*TOLREG) * DELTA(:,S>0.01);
   nEr = nEr+size(DELTA,2);
   C1 = [C1 ; DELTA(1:nX,:)'];
   D12 = [D12 ; DELTA(nX+1:nX+nU,:)'];
end  

% The regularized matrices are [C1;dC1] and [D12;dD12]
dC1 = C1(nE+1:nEr,:);
dD12 = D12(nE+1:nEr,:);
   

%----------------------------------------------------------
function cnorm = localColNorm(M)
% Computes norm of columns of M
nc = size(M,2);
cnorm = zeros(1,nc);
for ct=1:nc
   cnorm(ct) = norm(M(:,ct));
end

function [V,SCALE] = localRegPert(M,ireg,TOLREG,OPTS)
% Computes regularizing perturbation
[V,S] = schur(M);  % note: M must be Hermitian
S = abs(diag(S));
SCALE = max(S);
V = V(ireg,S<TOLREG*SCALE);
% Can only regularize directions of the form [0;x;u;0]
if ~isempty(V)
   % Note: All columns of V should have unit norm if P12 is fully regularizable
   [~,S,W] = svd(V,0);
   if OPTS.Instrument && size(V,2)>1
      if all(diag(S)>0.99)
         disp('*** MULTIPLICITY>1')
      else
         disp('*** MULTIPLICITY>1 WITH NON REG DIRECTIONS')
      end
   end
   V = V * W(:,diag(S)>0.99);
end
