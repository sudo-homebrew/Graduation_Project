function KOPT = h2K(A,B1,B2,C1,C2,D11,D12,D21,Ts,Sx)
% Computes H2 optimal controller. Returns [] when controller fails to 
% exist or stabilize the closed-loop system.
%
% Sx is a structure containing the diagonal state scaling for the X and Y 
% Riccati equations.

%   Author(s): P. Gahinet
%   Copyright 2018 Musyn Inc. and The MathWorks, Inc.
KOPT = [];
[nX,nU] = size(B2);
nY = size(C2,1);
[nE,nD] = size(D11);
NULL = zeros(nX);
SX = Sx.X;  SY = Sx.Y;  % scalings

% Solve X Riccati equation
AX = SX .* A ./ SX';
BX = [SX .* B2 , zeros(nX,nE)];
S = [zeros(nX,nU) , SX .\ (C1')];
RX = [zeros(nU) D12';D12 -eye(nE)];
if Ts==0
   [X1,X2,X3] = ricpack.CARE('s',AX,BX,NULL,NULL,RX,S,[]);
else
   [X1,X2,X3] = ricpack.DARE('s',AX,BX,NULL,RX,S,[]);
end
if nX>0 && isempty(X1)  % may happen when Regularize=off
   return
end
XT = X3(1:nU,:);
XX = X2/X1;  % scaled Riccati solution
X = SX .* XX .* SX';   X = (X+X')/2;

% Solve Y Riccati equation
AY = SY .\ A .* SY';
BY = [SY .* C2' , zeros(nX,nD)];
S = [zeros(nX,nY) , SY .\ B1];
RY = [zeros(nY) D21;D21' -eye(nD)];
if Ts==0
   [Y1,Y2,Y3] = ricpack.CARE('s',AY',BY,NULL,NULL,RY,S,[]);
else
   [Y1,Y2,Y3] = ricpack.DARE('s',AY',BY,NULL,RY,S,[]);
end
if nX>0 && isempty(Y1)
   return
end
YT = Y3(1:nY,:);
YY = Y2/Y1;  % scaled Riccati solution
Y = SY .* YY .* SY';   Y = (Y+Y')/2;

% Compute DK
if Ts==0
   % DK should zero Dcl = D11+D12*DK*D21 for the H2 norm to be finite
   DK = -pinv(D12) * D11 * pinv(D21);
   Z2X = [];   Z2Y = [];
else
   % Compute Z2X and Z2Y such that DK = -Z2X * [A B1;C1 D11] * Z2Y
   aux = [-X1 BX;BX'*X2 RX]\...
      [diag(SX) zeros(nX,nE); zeros(nU,nX+nE) ; zeros(nE,nX) eye(nE)];
   Z2X = aux(nX+1:nX+nU,:);
   aux = [-Y1 BY;BY'*Y2 RY]\...
      [diag(SY) zeros(nX,nD); zeros(nY,nX+nD) ; zeros(nD,nX) eye(nD)];
   Z2Y = aux(nX+1:nX+nY,:);
   DK = - Z2X * [A B1;C1 D11] * Z2Y';
%    DKe = -(D12'*D12+B2'*X*B2)\ ([B2'*X,D12']*[A B1;C1 D11]*[Y*C2';D21']) / (D21*D21'+C2*Y*C2');
%    norm(DK-DKe)/norm(DK)
end

% H2 controller formula
XT1 = XT/X1;
YT1 = (YT/Y1)';
Ku = XT1 .* SX';  % positive state-feedback gain (opposite of LQR gain)
Lx = -SY .* YT1;  % observer gain
BK = Lx + B2*DK;
CK = Ku - DK*C2;
if hasInfNaN(BK) || hasInfNaN(CK)
   % X1 or Y1 singular
   return
end
AK = A + B2*Ku - BK*C2;

% Check closed-loop stability
% Note: Test stability in [xP;xK-xP] coordinates where Acl = [Acl1 *;0 Acl2]
Acl1 = AX + (SX .* B2)*XT1;
Acl2 = AY + YT1*(C2 .* SY');
CLP = [eig(Acl1);eig(Acl2)];
if (Ts==0 && all(real(CLP)<0)) || (Ts~=0 && all(abs(CLP)<1))
   KOPT = struct('A',AK,'B',BK,'C',CK,'D',DK,...
      'X',X,'Y',Y,'Ku',Ku,'Lx',Lx,'Z2X',Z2X,'Z2Y',Z2Y);
end

% INSTRUMENTATION (FOR DEVELOPMENT PURPOSE ONLY)
%    % Eigenvalue sensitivity
%    Acl0 = [A+B2*DK*C2 B2*CK;BK*C2 AK];
%    CLP0 = eig(Acl0);
%    %[esort(CLP0) esort(CLP)]
%    if (Ts==0 && xor(all(real(CLP)<0),all(real(CLP0)<0))) || ...
%          (Ts~=0 && xor(all(abs(CLP)<1),all(abs(CLP0)<1)))
%       fprintf('**** EIG(ACL) INCONSISTENCY\n')
%    end
%    % Closed-loop performance
%    P = ss(A,[B1 B2],[C1;C2],[D11 D12;D21 zeros(nY,nU)],Ts);
%    K = ss(AK,BK,CK,DK,Ts);
%    G2 = norm(lft(P,K));

