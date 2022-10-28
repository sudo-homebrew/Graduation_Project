function LBcert = causeLowDamping(A,B,C,D,LBcert,blkData)
%   Scales the varying uncertainty DeltaV to make 
%        I-G(j*w)*Delta(t),   Delta(t) = DeltaF + t * DeltaV
%   singular for some nonnegative frequency w, or equivalently, 
%   A(Delta)-j*w*I singular where
%       A(Delta) = A + B * Delta * (I-D*Delta) \ C
%   The scaling factor t is computed as the smallest t>0 such that 
%   A(Delta(t)) has an unstable eigenvalue with nonnegative imaginary 
%   part.
%
%   This function is meant to take an almost destabilizing Delta and 
%   adjust DeltaV to cause marginal instability. The scaling t is 
%   therefore expected to be close to 1 and values far exceeding 1 
%   are ignored (lower bound is set to zero).

%   Copyright 1986-2018 The MathWorks, Inc.

% Note: Code written for Ts=0
Delta = LBcert.Delta;
[ny,nu] = size(D);
VaryRows = blkData.FVidx.VaryRows;
VaryCols = blkData.FVidx.VaryCols;
DeltaV = zeros(nu,ny);
DeltaV(VaryCols,VaryRows) = Delta(VaryCols,VaryRows);
if norm(DeltaV,1)==0
   clp = eig(A + B*Delta*((eye(ny)-D*Delta)\C));
   if all(real(clp)<0)
      % DELTA is not destabilizing. Zero out DELTA
      LBcert = struct('w',0,'LB',0,'Delta',0*Delta);
   end
   return
end
DeltaF = Delta;
DeltaF(VaryCols,VaryRows) = 0;

% Initialize search interval
clp0 = eig(A + B*Delta*((eye(ny)-D*Delta)\C));  % t=1
if all(real(clp0)<0)
   % Stable
   tStable = 1;  tUnstable = 3;  clpUnstable = [];
else
   % Unstable
   tStable = 0;  tUnstable = 1;  clpUnstable = clp0;
end

% Take into account well-posedness of the feedback loop
eV = eig(D*DeltaV,eye(ny)-D*DeltaF);
tIP = 1./real(eV(real(eV)>0 & abs(imag(eV))<1e-8,:));  % ill-posed t's
tIP = tIP(tIP>tStable & tIP<tUnstable);
if ~isempty(tIP)
   tUnstable = min(tIP);  clpUnstable = Inf;
end   
   
% Find zero crossing
while tUnstable-tStable>1e-8
   t = (tStable+tUnstable)/2;
   Delta = DeltaF+t*DeltaV;
   clp = eig(A + B*Delta*((eye(ny)-D*Delta)\C));
   if all(real(clp)<0)
      tStable = t;
   else
      tUnstable = t;  clpUnstable = clp;
   end
   if tUnstable<0.51 || tStable>1.99
      % Scaling out of range: zero out DELTA
      LBcert = struct('w',0,'LB',0,'Delta',0*Delta);
      return
   end
end

% Update LBcert
if any(isinf(clpUnstable))
   LBcert.w = Inf;
else
   % Note: Real part may not be small when wLB near Inf (near ill-posedness)
   [~,imax] = max(real(clpUnstable));
   LBcert.w = abs(imag(clpUnstable(imax)));
end
LBcert.Delta = DeltaF + tUnstable * DeltaV;
LBcert.LB = 1/(tUnstable*norm(DeltaV));