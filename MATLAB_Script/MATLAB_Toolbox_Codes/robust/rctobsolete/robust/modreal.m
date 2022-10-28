function [a1,b1,c1,d1,a2,b2,c2,d2] = modreal(varargin);

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
% In either case, the resulting state-space models are ordered in block
% diagonal form with real eigenvalues in 1x1 or Jordan block (for repeated 
% eigenvalues) and complex eigenvalues "a+bj" in real 2x2 blocks [a b;-b a].
%
% This routines overcomes the common "Defective Eigenvalue" problem embedded in
% most physical systems with rigid body dynamics. SLOWFAST work with system
% without jw-axis pole(s).
%
% Overloaded methods:
%  help lti/modreal.m
% 
% See also SLOWFAST, REIG, BLKRSCH.

%old help
%MODREAL State-space modal truncation/realization.
%
% [A1,B1,C1,D1,A2,B2,C2,D2] = MODREAL(A,B,C,D,CUT) or
% [SYS2,SYS3] = MODREAL(SYS1,CUT)  produces a slow-fast
% modal realization based on the number "cut" one specifies.
%
% If "CUT" is less than the size(A), D1 = D + C2*inv(-A2)*B2; D2 = 0;
%
% If "CUT" is not provided, a complete modal realization is returned.

% R. Y. Chiang & M. G. Safonov 1/93
% Copyright 1988-2005 The MathWorks, Inc.
% All Rights Reserved.

disp('  ');
disp('        - - Working on modal form decomposition - -');

[emsg,nag1,xsflag,Ts,a,b,c,d,cut]=mkargs5x('ss',varargin); error(emsg);
if Ts, error('LTI inputs must be continuous time (Ts=0)'), end

n = size(a)*[1;0];
if nag1 < 5
   cut = n;
end

[X,Lam] = reig(a,2);
[rd,cd] = size(d);
A = Lam;   B = inv(X)*b; C = c*X; D = d;
ModalRes = norm(inv(X)*a*X - Lam)

if cut ~= n
   a1 = A(1:cut,1:cut); b1 = B(1:cut,:); c1 = C(:,1:cut);
   a2 = A(cut+1:n,cut+1:n); b2 = B(cut+1:n,:); c2 = C(:,cut+1:n);
   d1 = d + c2*inv(-a2)*b2; d2 = zeros(rd,cd);
else
   a1 = A; b1 = B; c1 = C; d1 = D; a2 = [];b2 = [];c2 = [];d2 = [];
end

%
if xsflag
   a1 = mksys(a1,b1,c1,d1);
   b1 = mksys(a2,b2,c2,d2);
end
%
% ------ End of MODREAL.M % RYC/MGS %