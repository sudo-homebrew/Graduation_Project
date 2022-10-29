function [G1,G2] = modreal(G,cut,ordtype)
% MODREAL State-space modal truncation/realization.
%
% [G1,G2] = MODREAL(G,CUT,ORDTYPE) with G,G1,G2 being LTI objects, produces a 
% structural modal truncation/realization based on the number "CUT".
%
% If "CUT" is less than the size(A), 
%   For continuous case: D1 = D + C2*inv(-A2)*B2; D2 = 0; 
%   For discrete case:   D1 = D + C2*inv(eye(size(A2))-A2)*B2; D2 = 0; 
% If "CUT" is not provided, a complete modal realization is returned.
%
% ORDTYPE = 'mag'  ( abs(eig(G1))  < abs(eig(G2)), default)
% ORDTYPE = 'real' ( real(eig(G1)) > real(eig(G2)) )
%
% In either case, the resulting state-space mdoels are ordered in block
% diagonal form with real eigenvalues in 1x1 or Jordan block (for repeated 
% eigenvalues) and complex eigenvalues "a+bj" in real 2x2 blocks [a b;-b a].
%
% This routines overcomes the common "Defective Eigenvalue" problem embeded in
% most physical systems with rigid body dynamics. SLOWFAST work with system
% without jw-axis pole(s).
%
% Overloaded methods:
%  help lti/modreal.m
% 
% See also SLOWFAST, REIG, BLKRSCH.

% R. Y. Chiang & M. G. Safonov 1/2003
% Copyright 1988-2005 The MathWorks, Inc. 


% [emsg,nag1,xsflag,Ts,a,b,c,d,cut]=mkargs5x('ss',varargin); error(emsg);
G = ss(G);
[a,b,c,d,Ts]= ssdata(G);
n = length(a);

nag1 = nargin;
if nag1 < 2
   cut = n;
end 
if nag1<3
   ordtype = 'mag';
else
   ordtype = ltipack.matchKey(ordtype,{'mag','real'});
end
switch ordtype
 case 'mag'
    ordtype = 6; % ascending order
 case 'real'
    ordtype = 3; % descending order
end

[Ab,X] = ordblk(a,cut,ordtype);
[rd,cd] = size(d);
A = Ab;   B = pinv(X)*b; C = c*X; D = d;
%ModalRes = norm(pinv(X)*a*X - Ab);

if cut ~= n
   a1 = A(1:cut,1:cut); b1 = B(1:cut,:); c1 = C(:,1:cut);
   a2 = A(cut+1:n,cut+1:n); b2 = B(cut+1:n,:); c2 = C(:,cut+1:n);
   if Ts
      d1 = d + c2*((eye(size(a2))-a2)\b2);
   else
      d1 = d + c2*pinv(-a2)*b2; 
   end
   d2 = zeros(rd,cd);
else
   a1 = A; b1 = B; c1 = C; d1 = D; 
   a2 = [];b2 = zeros(0,cd);c2 = zeros(rd,0);d2 = zeros(rd,cd);
end

%

G1 = ss(a1,b1,c1,d1,Ts);
G2 = ss(a2,b2,c2,d2,Ts);

function [Ak,T] = ordblk(A,cut,ordtype)

% Real ordered block diagonal form 
%
% [Ak,T] = ordblk(A,cut,ordtype) produces a real block-ordered decomposition
% of matrix A such that                             
%
%             Ak = blkdiag([Jordan Block of repeated zero eigenvalues],
%                          [complex eigenvalue]   , .....]);
%                                              2x2
%             where Jordan Block of the repeated zero eigenvalues is in the form of
%               
%                          [0 1 0 0 ...
%                           0 0 1 0 ...
%                           0 0 0 1 ...
%                            ..........]
%
%             and each complex eigenvalue a+bj is in 2x2 real matrix
%
%                          [a b;
%                          -b a].
%
% The resulting matrix Ak is ordered in block ascending form based on their 
% eigenvalue magnitudes.
%
% See also SLOWFAST, STABPROJ, and BLKRSCH.

% R. Y. Chiang & M. G. Safonov 1/2003
% Copyright 1988-2004 The MathWorks, Inc. 
% All Rights Reserved.

n = length(A);

if cut == n
   if ordtype == 6
        [T,Ak] = reig(A,2);      % magnitude ascending order
   end
   if ordtype == 3
        [T,Ak] = reig(A,1);      
        T = T(:,n:-1:1);
        Ak = Ak(n:-1:1,n:-1:1);  % real part desecend order
   end
   return
end

%
% General case:
%

[Tk,Aks] = blkrsch(A,ordtype,cut);   

Ak1 = Aks(1:cut,1:cut); Ak2 = Aks(cut+1:n,cut+1:n); A12 = Aks(1:cut,cut+1:n);

X = lyap(Ak1,-Ak2,A12);
[Tk2,Ak2] = reig(Ak2,2); 
Tdiag = [eye(cut,cut) X*Tk2; zeros(n-cut,cut) Tk2];

Ak = blkdiag(Ak1,Ak2);
T = Tk*Tdiag;
%
% ---------- End of ORDBLK.M % RYC/MGS
