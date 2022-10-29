function [FACT,Mr,Nr] = rncf(sys)
%RNCF  Compute right normalized coprime factorization.
%
%   RNCF computes the right normalized coprime factorization of the
%   SISO or MIMO linear system SYS:
%
%      SYS = Nr/Mr,  Mr~ Mr + Nr~ Nr = I,  [Mr;Nr] stable.
%
%   FACT = RNCF(SYS) returns a state-space realization FACT of [Mr;Nr].
%
%   [FACT,Mr,Nr] = RNCF(SYS) also returns the coprime factors Mr and Nr
%   separately. Note that FACT, Mr, and Nr all share the same (A,B)
%   matrices and have the same number of states as SYS.
%
%   See also LNCF, NCFMR, NCFSYN.

%   Copyright 2018 MathWorks and MUSYN Inc.
if nmodels(sys)~=1
   error(message('Robust:analysis:rncf3'))
elseif hasdelay(sys)
   error(message('Robust:analysis:rncf4'))
end
try
   [A,B,C,D,E,Ts] = dssdata(sys);
catch
   error(message('Robust:analysis:rncf2'))
end
[p,m] = size(D);
n = size(A,1);
if isequal(E,eye(n))
   E = [];
end
TU = sys.TimeUnit;

% Solve Riccati equation
if n>0
   Q = zeros(n);
   R = [eye(m) D';D -eye(p)];
   S = [zeros(n,m) C'];
   if Ts==0
      [~,K] = icare(A,[B zeros(n,p)],Q,R,S,E);
   else
      [X,K] = idare(A,[B zeros(n,p)],Q,R,S,E);
   end
   if isempty(K) || ~all(isfinite(K(:)))
      % (A,B) is not stabilizable, (C,A) has unobservable mode on stability
      % boundary, or E is singular. Note that R is always invertible.
      error(message('Robust:analysis:rncf1'))
   end
else
   K = zeros(m+p,n);
end

% Compute Z = S^(-1/2) with
%   CT:  S = I+D'*D
%   DT:  S = I+D'*D+B'*X*B
if n==0 || Ts==0
   [~,s,v] = svd(D);
   nsv = min(p,m);
else
   [UX,TX] = schur(X); % Note: X>=0
   [~,s,v] = svd([D ; sqrt(abs(diag(TX))).*(UX'*B)]); % [D;X^(1/2)*B]
   nsv = min(p+n,m);
end
s = diag(s(1:nsv,1:nsv));
Z = v * diag([1./sqrt(1+s.^2) ; ones(m-nsv,1)]) * v';

% Construct outputs
% Note: F in Zhou is -K(1:m,:)
F = -K(1:m,:);
FACT = dss(A+B*F,B*Z,[F;C+D*F],[Z;D*Z],E,Ts,'TimeUnit',TU);
if nargout>1
   Mr = subparen(FACT,{1:m,':'});
   Nr = subparen(FACT,{m+1:m+p,':'});
end