function INFO = h2MakeINFO(P,nY,nU,KOPT,dC1,dD12,dB1,dD21,Su,Sy)
% Builds INFO structure for H2SYN.

%   Copyright 2018 The MathWorks, Inc.

% Create object
X = KOPT.X;  Y = KOPT.Y;
K = Su .* KOPT.Ku;
L = KOPT.Lx .* Sy';
Lu = Su .* KOPT.D .* Sy';
INFO = struct('X',X,'Y',Y','Ku',K,'Lx',L,'Lu',Lu,'Preg',[],...
   'NORMS',[],'KFI',[],'GFI',[],'HAMX',[],'HAMY',[]);

% Regularized plant
[A,B1,B2,C1,C2,D11,D12,D21,D22,Ts] = titodata(P,nY,nU);
[D11r,C1r,D12r,B1r,D21r] = ...
   rctutil.regSynData(D11,C1,D12,B1,D21,dC1,dD12./Su',dB1,Sy.\dD21);
Preg = ss(A,[B1r B2],[C1r ; C2],[D11r D12r;D21r D22],Ts);
INFO.Preg = Preg;
nX = size(A,1);

% Hamiltonians
[nE,nD] = size(D11r);
NULL = zeros(nX);
EYE = eye(nX);
% HAMX
B = [B2 zeros(nX,nE)];
S = [zeros(nX,nU) C1r'];
R = [zeros(nU) D12r';D12r -eye(nE)];
if Ts==0
   HAMX = [A NULL;NULL -A']-[B;-S] * (R\[S' B']);
else
   AA = A-B*(R\S');
   HAMX = [EYE B*(R\B');NULL AA']\[AA NULL;S*(R\S') EYE];
end
INFO.HAMX = HAMX;
% HAMY
B = [C2' zeros(nX,nD)];
S = [zeros(nX,nY) B1r];
R = [zeros(nY) D21r;D21r' -eye(nD)];
if Ts==0
   HAMY = [A' NULL;NULL -A]-[B;-S] * (R\[S' B']);
else
   AA = A'-B*(R\S');
   HAMY = [EYE B*(R\B');NULL AA']\[AA NULL;S*(R\S') EYE];
end
INFO.HAMY = HAMY;

% Norms of subproblems
% Note: Computing them for original data can introduce small discrepancies
% but is easier to interpret (e.g., KFI multiplies [x;w] rather than [x;wr]
[nE,nD] = size(D11);
if Ts==0
   FI  = trace(B1'*X*B1);
   FC  = trace(C1*Y*C1');
   OE = trace(-Y*K'*(B2'*X+D12'*C1)); % note: K is opposite of LQR gain
   DF = trace(L'*X*(Y*C2'+B1*D21'));
   KFI = [K zeros(nU,nD)];
else
   % See Robust and Optimal Control, Zhou et al, p. 562
   % Note: Z2X = (D12'*D12+B2'*X*B2)\[B2'*X,D12']
   %       Z2Y = (D21*D21'+C2*Y*C2')\[C2*Y,D21]
   Z2X = Su .* KOPT.Z2X(:,1:nX+nE);
   Z2Y = Sy .* KOPT.Z2Y(:,1:nX+nD);
   % FI
   KFI = -Z2X * [A B1;C1 D11];  % u = KFI*[x;w]
   F0 = KFI(:,nX+1:nX+nD);
   B = B1+B2*F0;  D = D11+D12*F0;
   FI  = trace(B'*X*B+D'*D);
   % OE = Trace((D12'*D12+B2'*X*B2) * 
   %            ((Lu*D21-F0)*(Lu*D21-F0)'+(Lu*C2-F2)*Y*(Lu*C2-F2)')
   % Use equivalent formula with fewer cancellations
   ABCD = [A B1;C1 D11];
   aux1 = [X*B2;D12] * Z2X * ABCD;
   aux2 = ABCD * (blkdiag(Y,eye(nD)) - Z2Y' * [C2*Y,D21]);
   OE = trace(aux1'*aux2);
   % FC
   F0 = -[C1 D11] * Z2Y';
   C = C1+F0*C2;  D = D11+F0*D21;
   FC  = trace(C*Y*C'+D*D');
   % DF
   aux1 = ABCD * Z2Y' * [C2*Y,D21];
   aux2 = (blkdiag(X,eye(nE)) - Z2X' * [B2'*X,D12']) * ABCD;
   DF = trace(aux1*aux2');
end
INFO.NORMS = sqrt(max(0,[FI OE DF FC]));
INFO.KFI = KFI;
M = [A B1;C1 D11]+[B2;D12]*KFI;
iE = nX+1:nX+nE;  jD = nX+1:nX+nD;
INFO.GFI = ss(M(1:nX,1:nX),M(1:nX,jD),...
   M(iE,1:nX),M(iE,jD),Ts,'TimeUnit',P.TimeUnit);


