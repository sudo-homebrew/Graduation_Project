function Z = minPos33(A,B,C,D,E)
% MINPOS33  Min-inertia Hermitian completion in 3-by-3 block matrix.
%
%   Computes Z that minimizes the number of positive eigenvalues in 
%
%             [  A   B   Z  ]
%      P(Z) = [  B'  C   D  ]
%             [  Z'  D'  E  ]
%
%   that is, 
%
%       i+(P(Z)) =  max( i+([A B;B' C]) , i+([C D;D' E]) )
%
%   MINPOS33 shifts small eigenvalues of C to improve cond(C) and keep 
%   the norm of Z in check. The shift is selected to preserve the positive 
%   inertias of [A B;B' C] and [C D;D' E].

%   Author: P. Gahinet
%   Copyright 2018 The MathWorks, Inc.

% C0 = C;

% Improve cond(C) and reduce ||Z|| by shifting the small eigenvalues of C
[U,T] = schur(C);
T = diag(T);
e1 = eig([A B;B' C]);
e2 = eig([C D;D' E]);
emag = abs([e1;e2]);
zeroTol = 1e3*eps*max(emag);
delta = min(emag)/4;
iS = find(abs(T)<max(delta,zeroTol));
if ~isempty(iS)
   % Shift small eigenvalues of C
   if delta<zeroTol
      % Any positive shift may change positive inertia of [A B;B' C] or [C D;D' E]
      T(iS) = -zeroTol;
   else
      % Use largest positive shift preserving positive inertias of 
      % [A B;B' C] and [C D;D' E]
      T(iS) = +delta;
   end
   C = U*(T.*U');
   C = (C+C')/2;
end

% Take Schur complement of C
AA = A-(B/C)*B';
CD = C\D;
EE = E-D'*CD;

% Minimize the positive inertia of [AA Y;Y' EE]
% Note: Tiny eigenvalues have unreliable sign but OK here since their 
%       contribution to Y is small.
[U1,D1] = schur((AA+AA')/2);
D1 = diag(D1);
ip1 = find(D1>0);  np1 = numel(ip1);
[~,is] = sort(D1(ip1),'descend');  ip1 = ip1(is);  % dominant ones first!
[U2,D2] = schur((EE+EE')/2);
D2 = diag(D2);
ip2 = find(D2>0);  np2 = numel(ip2);
[~,is] = sort(D2(ip2),'descend'); ip2 = ip2(is);
np = min(np1,np2);
DY = diag(sqrt(D1(ip1(1:np)).*D2(ip2(1:np))));  % use dominant NP
Y = 1.1 * U1(:,ip1) * blkdiag(DY,zeros(np1-np,np2-np)) * U2(:,ip2)';

% Compute Z
Z = Y + B*CD;

% Verification
% e = eig([A B Z;B' C0 D;Z' D' E]);
% zerotol = 1e3*eps*max(abs(e));
% np = [sum(e1>zerotol) sum(e2>zerotol) sum(e>zerotol)]