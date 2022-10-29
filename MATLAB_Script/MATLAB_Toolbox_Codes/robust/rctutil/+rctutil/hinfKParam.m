function J = hinfKParam(P,nY,nU,GAM,X,Y)
% For a regular plant P, compute the parameterization
%    K = LFT(J,Q)  with ||Q||oo < GAM
% of all GAM-suboptimal Hoo controllers.
%
% Reference: "State-Space formulae for all stabilizing controllers that
% satisfy an Hoo-norm bound and relations to risk sensitivity," by Keith
% Glover and John Doyle, Systems & Control Letters 11 (1988), pp. 167-172.

%   Copyright 2018 The MathWorks, Inc.

if isct(P)
   [a,b1,b2,c1,c2,d11,d12,d21,d22] = titodata(P,nY,nU);
   [nE,nD] = size(d11);
   nX = size(a,1);
   
   % Transform plant data to enforce D12=[I;0] and D21=[0,I]
   [q12,r12] = qr(d12);  % D12 "tall" and full column rank, by assumption
   q12 = q12(:,[(nU+1):end 1:nU])';
   r12 = r12(1:nU,:);
   
   [q21,r21] = qr(d21'); % D21 "wide" and full row rank, by assumption
   q21 = q21(:,[(nY+1):end 1:nY]);
   r21 = r21(1:nY,:)';
   
   c1 = q12*c1;  c2 = r21\c2;
   b1 = b1*q21;  b2 = b2/r12;
   d11 = q12*d11*q21;
   d12 = [zeros(nE-nU,nU); eye(nU)];  % d12 = q12*d12/r12 = [0;I]
   d21 = [zeros(nY,nD-nY), eye(nY)];  % d21 = r21\d21*q21 = [0,I]
   ddot1 = [d11/GAM;d21];  % (ne+ny)-by-nd
   d1dot = [d11/GAM,d12];  % ne-by-(nd+nu)
   
   % Riccati solutions
   Rx = d1dot'*d1dot;  Rx(1:nD,1:nD) = Rx(1:nD,1:nD)-eye(nD);
   Ry = ddot1*ddot1';  Ry(1:nE,1:nE) = Ry(1:nE,1:nE)-eye(nE);
%    xinf = X/GAM;
%    yinf = Y/GAM;
%    Sx = c1'*d1dot;
%    [xinf0,LX,REPORTX,axeig,xFail] = rctutil.care4Hsyn(a,b,c1'*c1,Rx,Sx,1e-8,b2);
%    norm(xinf-xinf0)/(1+norm(xinf))
%    Sy = b1*ddot1';
%    [yinf0,LY,REPORTY,ayeig,yFail] = rctutil.care4Hsyn(a',c',b1*b1',Ry,Sy,1e-8,c2');
%    norm(yinf-yinf0)/(1+norm(yinf))

   f = -Rx\(d1dot'*c1 + [b1/GAM b2]'*X);   f(1:nD,:) = f(1:nD,:)/GAM;
   h = -(b1*ddot1' + Y*[c1/GAM;c2]')/Ry;   h(:,1:nE) = h(:,1:nE)/GAM;
   f12 = f((nD-nY+1):nD,:);
   f2 = f((nD+1):(nD+nU),:);
   h12 = h(:,(nE-nU+1):nE);
   h2 = h(:,(nE+1):(nE+nY));

   d1111 = d11(1:(nE-nU),1:(nD-nY))/GAM;
   d1112 = d11(1:(nE-nU),(nD-nY+1):nD)/GAM;
   d1121 = d11((nE-nU+1):nE,1:(nD-nY));
   d1122 = d11((nE-nU+1):nE,(nD-nY+1):nD);
   d11hat = -d1122 - ((d1121*d1111')/(eye(nE-nU)-d1111*d1111'))*d1112;
   dum = eye(nY)-d1112'/(eye(nE-nU)-d1111*d1111')*d1112;
   d21hat = chol((dum+dum')/2);
   dum = eye(nU)-d1121/(eye(nD-nY)-d1111'*d1111)*(d1121'/GAM^2);
   d12hat = chol((dum+dum')/2)';

   z = eye(nX)-Y*X/GAM^2;
   b2hat = (b2+h12)*d12hat;
   c2hat = -d21hat*(c2+f12)/z;
   b1hat = -h2+(b2hat/d12hat)*d11hat;
   c1hat = f2/z+(d11hat/d21hat)*c2hat;
   ahat = a+h*[c1;c2]+(b2hat/d12hat)*c1hat;
   % Form the controller (recall D22=0 is ALWAYS true in this function, as the
   % nonzero D22 case is handled elsewhere.  Back-substitute scaling factors
   % r12 and r21
   b1hat = b1hat/r21;
   c1hat = r12\c1hat;
   bhat = [b1hat,b2hat];
   chat = [c1hat;c2hat];
   dhat = [r12\d11hat/r21,r12\d12hat; d21hat/r21, zeros(size(d11hat'))];
   
   J = ss(ahat,bhat,chat,dhat);
   if norm(d22,1)>0
      % Account for nonzero D22
      J = lft([zeros(nU,nY) eye(nU);eye(nY) -d22],J,nU,nY);
   end
   
else
   % Transform to continuous and back
   % Note: Prone to failure for singular plant as there is no guarantee Pc
   % is properly regularized.
   paaz = sqrt(eps)/8;
   p = pole(P);
   if (max(abs(p+1))>paaz) % if no poles near z=-1
      paaz=0;
   end
   % The transformation below doesn't seem correct for all sample times.
   % shouldn't [2, 1-paaz] be [Ts, 1-paaz]?
   TustAug = [2,1-paaz];  % [Ts,1-paaz];
   Pc = bilin(P,-1,'S_Tust',TustAug);
   [A,B1,B2,C1,C2,D11,D12,D21] = titodata(Pc,nY,nU);
   % Compute Riccati solutions for continuous plant
   Sx = struct('X',1,'Y',1);
   OPTS = struct('RelTol',1e-6,'ShowProgress',false,'Instrument',false,'ndigit',2);
   KGAM = rctutil.hinfKC(GAM,A,B1,B2,C1,C2,D11,D12,D21,0,Sx,OPTS);
   if isempty(KGAM)
      J = [];  % failure
   else
      Jc = rctutil.hinfKParam(Pc,nY,nU,GAM,KGAM.X,KGAM.Y);
      J = bilin(Jc,1,'S_Tust',TustAug);
      J.Ts = P.Ts;
   end
   
end



