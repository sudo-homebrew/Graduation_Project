function [KSTRUCT,INFO,GAMOUT] = hinfKFI(GAM,A,B1,B2,C1,D11,D12,Ts,Sx,OPTS)
% Computes full-information gain u = K*[x;w] for performance level GAM.
% Returns [] when GAM is not feasible. Sx is a vector containing the 
% diagonal state scaling for the X Riccati equation.

%   Author(s): P. Gahinet
%   Copyright 2018 Musyn Inc. and The MathWorks, Inc.
KSTRUCT = [];  GAMOUT = Inf;
INFO = struct('GAM',GAM,'PASS',false,'X',NaN,'ndigits',OPTS.ndigit); % for display
[nX,nU] = size(B2);
[nE,nD] = size(D11);
NULL = zeros(nX);

% Solve Riccati equation for Xoo
AX = Sx .* A ./ Sx';
B = [Sx .* [B2 B1/GAM] , zeros(nX,nE)];
S = [zeros(nX,nU+nD) , Sx .\ (C1')];
[tauX,B,S] = ltipack.scaleBC(B,S,1+norm(AX,1));  % implicit error scaling
Sx = tauX*Sx;
aux = [D12 , D11/GAM];  R = [diag([zeros(nU,1);-ones(nD,1)]) aux';aux -eye(nE)];
if Ts==0
   [X1,X2] = ricpack.CARE('s',AX,B,NULL,NULL,R,S,[]);
else
   [X1,X2] = ricpack.DARE('s',AX,B,NULL,R,S,[]);
end
if nX>0 && isempty(X1)  % should never happen
   if OPTS.Instrument, disp('######### X1=[] ############'), end
   return
end
X = Sx .* (X2/X1) .* Sx';  X = (X+X')/2;   % Riccati solution
INFO.X = X;

% Compute gain matrix
B1X = Sx .* B1;
B2X = Sx .* B2;
C1X = C1 ./ Sx';
if Ts==0
   aux = -[zeros(nU) D12';D12 -eye(nE)]\[(B2X'*X2)/X1 zeros(nU,nD);C1X D11];
   K = aux(1:nU,:);
else
   % Central solution minimizes positive inertia only when B2'*X*B2+D12'*D12>0
   M = ((B2X'*X2)/X1)*B2X + D12'*D12;
   if any(eig(M+M')<0)
      if OPTS.Instrument
         fprintf('Fail: GAM = %.7g: B2''*X*B2+D12''*D12 is indefinite\n',GAM)
      end
      return
   end
   aux = -[-X1 B2X zeros(nX,nE);B2X'*X2 zeros(nU) D12';zeros(nE,nX) D12 -eye(nE)]\...
      [AX B1X;zeros(nU,nX+nD);C1X D11];
   K = aux(nX+1:nX+nU,:);
%    % Note: Code below computes gain K that enforces GAM peak gain even 
%    % when X is indefinite
%    K = localComputeKDT(AX,B1X,B2X,C1X,D11,D12,X1,X2,GAM);
end
if hasInfNaN(K)
   % X1 singular
   return
end

% Check stability
Kx = K(:,1:nX);
Acl = AX + B2X * Kx;
CLP = eig(Acl);
if (Ts==0 && all(real(CLP)<0)) || (Ts~=0 && all(abs(CLP)<1))
%    if Ts~=0
%       % Return central solution
%       aux = -[-X1 B2X zeros(nX,nE);B2X'*X2 zeros(nU) D12';zeros(nE,nX) D12 -eye(nE)]\...
%          [AX B1X;zeros(nU,nX+nD);C1X D11];
%       K = aux(nX+1:nX+nU,:);
%       Kx = K(:,1:nX);
%    end
   Kw = K(:,nX+1:nX+nD);
   KSTRUCT = struct('KFI',[Kx .* Sx' , Kw],'GAM',GAM,'X',X);
   if nargout>2 && isinf(GAM)
      % Compute finite performance (NOTE: Ecl=I for GAM=Inf)
      Dcl = ltipack.ssdata(Acl,B1X+B2X*Kw,C1X+D12*Kx,D11+D12*Kw,[],Ts);
      GAMOUT = norminf(Dcl,min(OPTS.RelTol,1e-6),[],true);
   else
      GAMOUT = GAM;
   end
   INFO.PASS = true;
end

% INSTRUMENTATION
if OPTS.Instrument
   % Eigenvalue sensitivity
   Kx = K(:,1:nX) .* Sx';
   Kw = K(:,nX+1:nX+nD);
   Acl0 = A+B2*Kx;
   CLP0 = eig(Acl0);
   %[esort(CLP0) esort(CLP)]
   if (Ts==0 && xor(all(real(CLP)<0),all(real(CLP0)<0))) || ...
         (Ts~=0 && xor(all(abs(CLP)<1),all(abs(CLP0)<1)))
      fprintf('**** EIG(ACL) INCONSISTENCY\n')
   end
   % Closed-loop performance
   CL = ss(Acl0,B1+B2*Kw,C1+D12*Kx,D11+D12*Kw,Ts);
   GACT = getPeakGain(CL,1e-6);
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
      fprintf('Fail: GAM = %.7g/%.7g, B2*Kx = %.3e, SpecAbs = %.6g %s', ...
         GAM,GACT,norm(lrscale(B2*Kx,Sx,1./Sx),1),SpecAbs,FLAG)
   else
      fprintf('Pass: GAM = %.7g/%.7g, B2*Kx = %.3e, SpecAbs = %.6g %s', ...
         GAM,GACT,norm(lrscale(B2*Kx,Sx,1./Sx),1),SpecAbs,FLAG)
   end
end

%-------------------------------
% function K = localComputeKDT(A,B1,B2,C1,D11,D12,X1,X2,GAM)
% % Find K for performance level GAM. Suitable K's minimize the positive
% % inertia of some expression of the form M + P*X*Q'+ Q*X*P'.
% zeroTol = 1e3*eps;
% [nX,nU] = size(B2);
% [nE,nD] = size(D11);
% if GAM==Inf
%    % Use central solution (X>=0 guaranteed for GAM=Inf)
%    aux = -[-X1 B2 zeros(nX,nE);B2'*X2 zeros(nU) D12';zeros(nE,nX) D12 -eye(nE)]\...
%       [A B1;zeros(nU,nX+nD);C1 D11];
%    K = aux(nX+1:nX+nU,:);
% else
%    [q12,r12,perm12,rk12] = rrqrf([X2'*B2;D12],zeroTol,norm([B2;D12],1));
%    [q21,r21,perm21,rk21] = rrqrf(blkdiag(X1,eye(nD))',zeroTol,1);
%    X12 = X2'*X1;
%    M11 = -q12'*blkdiag(X12,eye(nE))*q12;
%    M11 = (M11+M11')/2;
%    M22 = -q21'*blkdiag(X12/GAM^2,eye(nD))*q21;
%    M22 = (M22+M22')/2;
%    M12 = (q12'*[X2'*A*X1 X2'*B1 ; C1*X1 D11]*q21)/GAM;
%    C12 = M12(rk12+1:nX+nE,rk21+1:nX+nD);
%    iz = 1:rk12;   jz = 1:rk21;
%    Z = rctutil.minPos33(M11(iz,iz),[M11(iz,rk12+1:nX+nE) M12(iz,rk21+1:nX+nD)],...
%       [M11(rk12+1:nX+nE,rk12+1:nX+nE) C12;C12' M22(rk21+1:nX+nD,rk21+1:nX+nD)],...
%       [M12(rk12+1:nX+nE,jz) ; M22(rk21+1:nX+nD,jz)],M22(jz,jz));
%    K(perm12(iz),perm21(jz)) = GAM * (r12(iz,iz)\(Z-M12(iz,jz))/r21(jz,jz)');
%    
% %    ABCD = blkdiag(X2',eye(nE)) * ([A B1;C1 D11]+[B2;D12]*K) * blkdiag(X1,eye(nD));
% %    TESTM = [blkdiag(-X2'*X1,-eye(nE)) ABCD/GAM;ABCD'/GAM blkdiag(-X2'*X1/GAM^2,-eye(nD))];
% %    X = X2/X1;
% %    fprintf('   max(TESTM) = %.3g,  min(X) = %.3g\n',...
% %       max(eig((TESTM+TESTM')/2)),min(eig((X+X')/2)))
% end
