function [gap,nugap] = gapmetric(sys1,sys2,ttol)
%GAPMETRIC  Computes the gap and Vinnicombe gap metrics.
%
%    [GAP,NUGAP] = GAPMETRIC(P1,P2) calculates the gap metric and
%    the Vinnicombe variation of the gap metric, called the "nu-gap",
%    between the two linear systems P1 and P2.
%
%    [GAP,NUGAP] = GAPMETRIC(P1,P2,TOL)  specifies a relative accuracy
%    TOL for calculating these gaps. TOL must be between 0 and 1
%    and TOL=1e-3 by default.
%
%    Note that these metrics always satisfy
%        0 <= NUGAP <= GAP <= 1
%
%   See also NCFMARGIN, DISKMARGIN, LNCF, RNCF, NCFMR.

%   Copyright 1991-2011 The MathWorks, Inc.
narginchk(2,3)
if nargin<3
   ttol = 1e-3;
elseif isnumeric(ttol) && isscalar(ttol) && isreal(ttol) && ttol>0 && ttol<1
   ttol = double(ttol);
else
   warning(message('Robust:analysis:TolReset'))
   ttol = 1e-3;
end

% Validate models
try
   % Convert to state space
   sys1 = ss(sys1);
   sys2 = ss(sys2);
catch ME
   error(message('Robust:analysis:gapmetric1'))
end
try
   % Reconcile array sizes and sample times
   [sys1,sys2] = matchArraySize(sys1,sys2);
   [sys1,sys2] = matchAttributes(sys1,sys2);
catch ME
   throw(ME)
end
if hasdelay(sys1) || hasdelay(sys2)
   error(message('Robust:analysis:gapmetric2'))
elseif ~isequal(iosize(sys1),iosize(sys2))
   error(message('Robust:analysis:gapmetric3'))
end

% try
%    [A,B] = gapmetric0(sys1,sys2,ttol);
% catch
%    A = NaN; B = NaN;
% end

% Compute gaps
sz = size(sys1);
sz = [sz(3:end) 1 1];
gap = zeros(sz);
nugap = zeros(sz);
for ct=1:numel(gap)
   [isP1,P1] = isproper(subparen(sys1,{':',':',ct}));
   [isP2,P2] = isproper(subparen(sys2,{':',':',ct}));
   if ~(isP1 && isP2)
      error(message('Robust:analysis:gapmetric4'))
   end
   try
      % Compute graph symbols for P1 and P2 (right normalized coprime
      % factorizations)
      G1 = rncf(P1);
      G2 = rncf(P2);
      % Compute gaps
      gap(ct) = LOCALgap(G1,G2,ttol);
      nugap(ct) = LOCALnugap(G1,G2,P1,ttol);
   catch
      % NCF failed due to unstable pole/zero cancellations
      gap(ct) = 1;  nugap(ct) = 1;
   end
   % GAP = [abs(A(ct)-gap(ct)) abs(B(ct)-nugap(ct))]/ttol
end

%---------- LOCALNUGAP -----------------%
function gap = LOCALnugap(G1,G2,sys1,ttol)
% Compute nu-gap
gap = 1;  % default
Ts = getTs(G1);

% Compute the finite zeros of THETA=G1'*G2 and check the winding number
% condition (THETA must have the same number of RHP zeros and RHP poles,
% which is ORDER(G1) since G2 is stable)
THETA = G1'*G2;
if Ts==0 && min(svd(THETA.D))<100*eps
   % In continuous time, THETA.D must be nonsingular for det(THETA(j*Inf))
   % to be nonzero. Note that THETA.D is the inner product of the two
   % orthogonal matrices G1.D and G2.D and THETA.D = Z1'*(I+D1'*D2)*Z2 so
   % this is equivalent to I+D1'*D2 nonsingular.
   return
end
z = tzero(THETA);
if Ts==0
   nRHP = sum(real(z)>0);
else
   nRHP = sum(abs(z)>1);
end
%WNO = [nRHP nRHP-order(G1)]

% The nu-gap is the peak gain of PSI when WNO=0
if nRHP==order(G1)
   [p1,m1] = size(sys1);
   LNCF1 = lncf(sys1);
   PSI = LNCF1 * [zeros(p1,m1) eye(p1);-eye(m1) zeros(m1,p1)] * G2;
   gap = min(1,getPeakGain(PSI,ttol));
end

% NOTE: In CT, the zeros if G1'*G2 are the eigenvalues of
% b1w = b1/(eye(m1)+d2'*d1);
% yc2 = (eye(p1)+d2*d1')\c2;
% A = [-(a1-b1w*d2'*c1)', c1'*yc2; b2*b1w', a2-b2*d1'*yc2];

%---------- LOCALGAP -----------------%
function delta = LOCALgap(G1,G2,ttol)
% Note
%    delta(P1,P2) = max(delta->(P1,P2),delta->(P2,P1))
% and
%    delta->(P1,P2) = delta->(P2,P1)
% whenever both quantities are < 1.

% Note: Just need to compute delta(P1,P2) and check that delta(P2,P1)<1
m = size(G1,2);
[~,~,delta] = hinffi([G2 G1],m,1-ttol);
if delta<Inf
   opt = hinfsynOptions('AbsTol',ttol/2,'RelTol',ttol/2);
   [~,~,delta] = hinffi([G1 G2],m,[0 1-ttol],opt);
end
delta = min(1,delta);

