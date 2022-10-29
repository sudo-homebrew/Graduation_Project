function [KSTRUCT,INFO,GAMOUT] = hinfKC(GAM,A,B1,B2,C1,C2,D11,D12,D21,Ts,Sx,OPTS)
% Computes central controller for performance level GAM > MAX(GAMX,GAMY).
% Returns [] when GAM is not feasible.
%
% Sx is a structure containing the diagonal state scaling for the X and Y 
% Riccati equations.

%   Author(s): P. Gahinet
%   Copyright 2018 Musyn Inc. and The MathWorks, Inc.
KSTRUCT = [];  GAMOUT = Inf;
INFO = struct('GAM',GAM,'PASS',false,'X',NaN,'Y',NaN','ndigits',OPTS.ndigit); % for display
[nX,nU] = size(B2);
nY = size(C2,1);
[nE,nD] = size(D11);
NULL = zeros(nX);
SX = Sx.X;  SY = Sx.Y;  % scalings

% Solve X Riccati equation
AX = SX .* A ./ SX';
B = [SX .* [B2 B1/GAM] , zeros(nX,nE)];
S = [zeros(nX,nU+nD) , SX .\ (C1')];
[tauX,B,S] = ltipack.scaleBC(B,S,1+norm(AX,1));  % implicit error scaling
SX = tauX*SX;
aux = [D12 , D11/GAM];  R = [diag([zeros(nU,1);-ones(nD,1)]) aux';aux -eye(nE)];
if Ts==0
   [X1,X2,X3] = ricpack.CARE('s',AX,B,NULL,NULL,R,S,[]);
else
   [X1,X2,X3] = ricpack.DARE('s',AX,B,NULL,R,S,[]);
end
if nX>0 && isempty(X1)   % may happen when Regularize=off
   if OPTS.Instrument, disp('######### X1=[] ############'), end
   return
end
XT = X3(1:nU,:);
XU = X3(nU+1:nU+nD,:);
XX = X2/X1;  % scaled Riccati solution
X = SX .* XX .* SX';   X = (X+X')/2;
INFO.X = X;

% Solve Y Riccati equation
AY = SY .\ A .* SY';
B = [SY .* [C2' C1'/GAM] , zeros(nX,nD)];
S = [zeros(nX,nY+nE) , SY .\ B1];
[tauY,B,S] = ltipack.scaleBC(B,S,1+norm(AY,1)); % implicit disturbance scaling
SY = SY*tauY;
aux = [D21 ; D11/GAM];  R = [diag([zeros(nY,1);-ones(nE,1)]) aux;aux' -eye(nD)];
if Ts==0
   [Y1,Y2,Y3] = ricpack.CARE('s',AY',B,NULL,NULL,R,S,[]);
else
   [Y1,Y2,Y3] = ricpack.DARE('s',AY',B,NULL,R,S,[]);
end
if nX>0 && isempty(Y1)
   if OPTS.Instrument, disp('######### Y1=[] ############'), end
   return
end
YT = Y3(1:nY,:);
YU = Y3(nY+1:nY+nE,:);
YY = Y2/Y1;  % scaled Riccati solution
Y = SY .* YY .* SY';   Y = (Y+Y')/2;
INFO.Y = Y;

% Scaled plant matrices
B1X = SX .* B1;
B2X = SX .* B2;
C1X = C1 ./ SX';
B1Y = SY .\ B1;
B2Y = SY .\ B2;
C1Y = C1 .* SY';
C2Y = C2 .* SY';

% Compute DK
if Ts==0
   DK = localComputeDK_CT(D11,D12,D21,GAM);
else
   DK = localComputeDK_DT(lrscale(A,SX,SY),B1X,B2X,C1Y,C2Y,D11,D12,D21,X1,X2,Y1,Y2,GAM);
end

% Central controller matrices
SXY = SX.*SY;
XTU1 = (XT-DK*(D21/GAM)*XU)/X1;
YTU = YT'-YU'*(D12/GAM)*DK;
YTU1 = Y1'\YTU;
if isinf(GAM)
   BK = B2*DK - SY .* YTU1;
else
   BK = SY .* (( Y2' * (SXY.*(X2/X1).*SXY') / GAM^2 - Y1') \ (YTU - Y1'*(B2Y*DK)));
end
CK = XTU1 .* SX' - DK*C2;
if hasInfNaN(BK) || hasInfNaN(CK)
   % X1 or Y1 singular
   return
end
AK = A - BK*C2 + ((B2*XT + (B1-BK*D21)*XU/GAM)/X1) .* SX';

% Check closed-loop stability
% 1) Test stability in [xP;xK-xP] coordinates
% 2) Use scaled descriptor form to reduce eigenvalue sensitivity and avoid
%    issues with balancing and
% 3) Don't use inversion-free formula where X1 and Y1 are moved to Ecl.
%    This makes the pencil (Acl,Ecl) singular when (A,B2,C2) is not
%    stabilizable + detectable
GXY = YY * (SXY .* XX .* SXY') / GAM^2;
aux1 = B1Y - GXY * (B1Y+B2Y*DK*D21) + YTU1*D21;
aux2 = (aux1/GAM)*(XU/X1);
Acl = [AX+B2X*XTU1  (B2X*CK).*SY'; aux2  AY-GXY*(AY+B2Y*DK*C2Y)+YTU1*C2Y+aux2.*SXY'];
Ecl = [eye(nX) NULL ; NULL eye(nX)-GXY];
CLP = eig(Acl,Ecl);
if (Ts==0 && all(real(CLP)<0)) || (Ts~=0 && all(abs(CLP)<1))
   if isinf(GAM)
      % Compute finite performance (NOTE: Ecl=I for GAM=Inf)
      % Note: Also confirms H2 controller is stabilizing in the presence of
      %       non-minimal modes on jw-axis
      Bcl = [B1X+B2X*DK*D21 ; -aux1];
      Ccl = [C1X+D12*XTU1 , (D12*CK).*SY'];
      Dcl = D11+D12*DK*D21;
      GAMOUT = norminf(ltipack.ssdata(Acl,Bcl,Ccl,Dcl,[],Ts),...
         min(OPTS.RelTol,1e-6),[],true);
   else
      GAMOUT = GAM;
   end
   INFO.PASS = (GAMOUT<Inf);
   if INFO.PASS
      Ku = (XT/X1) .* SX';
      Kw = (XU/X1) .* (SX'/GAM);
      Lx = BK-B2*DK;
      Lu = DK;
      KSTRUCT = struct('A',AK,'B',BK,'C',CK,'D',DK,'GAM',GAM,...
         'X',X,'Y',Y,'Ku',Ku,'Kw',Kw,'Lx',Lx,'Lu',Lu);
   end
end

% INSTRUMENTATION (FOR DEVELOPMENT PURPOSE ONLY)
if OPTS.Instrument
   % Eigenvalue sensitivity
   Acl0 = [A+B2*DK*C2 B2*CK;BK*C2 AK];
   CLP0 = eig(Acl0);
   %[esort(CLP0) esort(CLP)]
   if (Ts==0 && xor(all(real(CLP)<0),all(real(CLP0)<0))) || ...
         (Ts~=0 && xor(all(abs(CLP)<1),all(abs(CLP0)<1)))
      fprintf('**** EIG(ACL) INCONSISTENCY\n')
   end
   % Closed-loop performance
   P = ss(A,[B1 B2],[C1;C2],[D11 D12;D21 zeros(nY,nU)],Ts);
   K = ss(AK,BK,CK,DK,Ts);
   GACT = getPeakGain(lft(P,K),1e-6);
   if Ts==0
      SpecAbs = max(real(CLP));
   else
      SpecAbs = max(abs(CLP));
   end
   if GACT>GAM*(1+2e-6)
      FLAG = '';
   else
      FLAG = '******';
   end
   if isempty(KSTRUCT)
      fprintf('Fail: GAM = %.7g/%.7g, B2CK = %.3e, BKC2 = %.3e, rho = %.6g, SpecAbs = %.6g %s\n', ...
         GAM,GACT,norm(lrscale(B2*CK,SX,1./SX),1),norm(lrscale(BK*C2,1./SY,SY),1),...
         max(real(eig(X*Y)))/GAM^2,SpecAbs,FLAG)
   else
      fprintf('Pass: GAM = %.7g/%.7g, B2CK = %.3e, BKC2 = %.3e, rho = %.6g, SpecAbs = %.6g %s\n',...
         GAM,GACT,norm(lrscale(B2*CK,SX,1./SX),1),norm(lrscale(BK*C2,1./SY,SY),1),...
         max(real(eig(X*Y)))/GAM^2,SpecAbs,FLAG)
   end
end


%-------------------------------------------
function DK = localComputeDK_CT(D11,D12,D21,GAM)
% Find DK such that ||D11+D12*DK*D21||<=GAM

% Reduce to finding X s.t.
%    || X   B ||
%    ||       ||  <  GAM
%    || C   D ||
[nE,nU] = size(D12);
[nY,nD] = size(D21);
if norm(D11)<GAM
   DK = zeros(nU,nY);
else
   [q12,r12] = qr(D12);
   [q21,r21] = qr(D21');
   D11t = q12'*D11*q21;
   X = ltipack.parrott(D11t(1:nU,nY+1:nD),D11t(nU+1:nE,1:nY),...
      D11t(nU+1:nE,nY+1:nD),GAM);
   DK = r12(1:nU,:)\(X-D11t(1:nU,1:nY))/r21(1:nY,:)';
end

function DK = localComputeDK_DT(A,B1,B2,C1,C2,D11,D12,D21,X1,X2,Y1,Y2,GAM)
% Find DK for performance level GAM. Suitable DK's minimize the positive
% inertia of some expression of the form M + P*X*Q'+ Q*X*P'.
zeroTol = 1e3*eps;
nX = size(A,1);
[nE,nU] = size(D12);
[nY,nD] = size(D21);
% REVISIT: WHEN CAN WE SET DK=0?
DK = zeros(nU,nY);
if isfinite(GAM)
   [q12,r12,perm12,rk12] = rrqrf([X2'*B2;D12],zeroTol,norm([B2;D12],1));
   [q21,r21,perm21,rk21] = rrqrf([C2*Y2,D21]',zeroTol,norm([C2 D21],1));
   M11 = -q12'*blkdiag(X2'*X1,eye(nE))*q12;
   M11 = (M11+M11')/2;
   M22 = -q21'*blkdiag(Y2'*Y1,eye(nD))*q21;
   M22 = (M22+M22')/2;
   M12 = (q12'*[X2'*A*Y2 X2'*B1 ; C1*Y2 D11]*q21)/GAM;
   C12 = M12(rk12+1:nX+nE,rk21+1:nX+nD);
   iz = 1:rk12;   jz = 1:rk21;
   Z = rctutil.minPos33(M11(iz,iz),[M11(iz,rk12+1:nX+nE) M12(iz,rk21+1:nX+nD)],...
      [M11(rk12+1:nX+nE,rk12+1:nX+nE) C12;C12' M22(rk21+1:nX+nD,rk21+1:nX+nD)],...
      [M12(rk12+1:nX+nE,jz) ; M22(rk21+1:nX+nD,jz)],M22(jz,jz));
   DK(perm12(iz),perm21(jz)) = GAM * (r12(iz,iz)\(Z-M12(iz,jz))/r21(jz,jz)');
   
   %% VALIDATION
   %    DK0 = maxNeg([-GAM*X2'*X1 zeros(nX,nE) X2'*A*Y2 X2'*B1; zeros(nE,nX) -GAM*eye(nE) C1*Y2 D11 ;...
   %       Y2'*A'*X2 Y2'*C1' -GAM*Y2'*Y1 zeros(nX,nD) ; B1'*X2 D11'  zeros(nD,nX) -GAM*eye(nD)],...
   %       [X2'*B2;D12],[Y2'*C2';D21']);
   %
   %    [q12,r12] = qr([X2'*B2;D12]);
   %    [q21,r21] = qr([C2*Y2,D21]');
   %    M11 = -q12'*blkdiag(X2'*X1,eye(nE))*q12;
   %    M11 = (M11+M11')/2;
   %    M22 = -q21'*blkdiag(Y2'*Y1,eye(nD))*q21;
   %    M22 = (M22+M22')/2;
   %    M12 = (q12'*[X2'*A*Y2 X2'*B1 ; C1*Y2 D11]*q21)/GAM;
   %    C12 = M12(nU+1:nX+nE,nY+1:nX+nD);
   %    Z = rctutil.minPos33(M11(1:nU,1:nU),[M11(1:nU,nU+1:nX+nE) M12(1:nU,nY+1:nX+nD)],...
   %       [M11(nU+1:nX+nE,nU+1:nX+nE) C12;C12' M22(nY+1:nX+nD,nY+1:nX+nD)],...
   %       [M12(nU+1:nX+nE,1:nY) ; M22(nY+1:nX+nD,1:nY)],M22(1:nY,1:nY),1e4*eps);
   %    DK1 = GAM * (r12(1:nU,:)\(Z-M12(1:nU,1:nY))/r21(1:nY,:)');
   %
   %    fprintf('   DK = %.3g, DK0 = %.3g, DK1 = %.3g\n',norm(DK),norm(DK0),norm(DK1))
   %    ABCD = [X2'*(A+B2*DK*C2)*Y2 X2'*(B1+B2*DK*D21) ; (C1+D12*DK*C2)*Y2 D11+D12*DK*D21];
   %    ABCD0 = [X2'*(A+B2*DK0*C2)*Y2 X2'*(B1+B2*DK0*D21) ; (C1+D12*DK0*C2)*Y2 D11+D12*DK0*D21];
   %    ABCD1 = [X2'*(A+B2*DK1*C2)*Y2 X2'*(B1+B2*DK1*D21) ; (C1+D12*DK1*C2)*Y2 D11+D12*DK1*D21];
   %    TESTM = [blkdiag(-X2'*X1,-eye(nE)) ABCD/GAM;ABCD'/GAM blkdiag(-Y2'*Y1,-eye(nD))];
   %    TESTM0 = [blkdiag(-X2'*X1,-eye(nE)) ABCD0/GAM;ABCD0'/GAM blkdiag(-Y2'*Y1,-eye(nD))];
   %    TESTM1 = [blkdiag(-X2'*X1,-eye(nE)) ABCD1/GAM;ABCD1'/GAM blkdiag(-Y2'*Y1,-eye(nD))];
   %    e = eig(TESTM+TESTM');
   %    e0 = eig(TESTM0+TESTM0');
   %    e1 = eig(TESTM1+TESTM1');
   %    % [sort(e) sort(e0) sort(e1)]
   %    if any(diff([sum(e>0),sum(e0>0),sum(e1>0)]))
   %       fprintf('   i+(DK)=%d, i+(DK0)=%d, i+(DK1)=%d\n',sum(e>0),sum(e0>0),sum(e1>0))
   %       % keyboard
   %    end
end
