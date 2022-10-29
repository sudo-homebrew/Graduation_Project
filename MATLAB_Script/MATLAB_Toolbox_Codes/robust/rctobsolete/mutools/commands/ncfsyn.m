function    [sysk,emax,sysobs]=ncfsyn(sysgw,factor,opt)
%NCFSYN  Glover-McFarlane loop shaping.
%
%   [K,CL,GAM,INFO] = NCFSYN(G,W1,W2) computes the Glover-McFarlane
%   controller K for the plant G and loop-shaping weights W1,W2. Pick
%   W1,W2 so that the shaped plant Gs=W2*G*W1 has the desired loop shape.
%   NCFSYN computes the controller Ks that maximizes the robust stability
%   margin b(Gs,Ks) (see NCFMARGIN) and returns K=W1*Ks*W2 as loop-shaping
%   controller for the original plant G.
%
%             z1                          z2
%     d1 -->O--->[ W2 ]-->[ Ks ]-->[ W1 ]---->O--->[ G ]---+
%         + |    <---------------------->   + |            |
%           |                K                d2           |
%           +----------------------------------------------+
%
%   NCFSYN also returns the closed-loop CL = [I;K]*inv(I-G*K)*[I,G] from
%   [d1;d2] to [z1;z2] in the diagram above, the H-infinity performance
%   GAM, and a struct INFO with the following data:
%      INFO.gopt   optimal H-infinity performance GOPT
%      INFO.emax   nu-gap stability margin bopt=1/GOPT
%      INFO.Gs     shaped plant Gs=W2*G*W1
%      INFO.Ks     optimal controller for Gs, same as NCFSYN(Gs,1,1).
%
%   [...] = NCFSYN(G,W1,W2,TOL) specifies how tightly GAM should approximate
%   the optimal performance GOPT. The default is TOL=1e-3 (0.1% gap). Try
%   increasing TOL when the controller K has undesirable fast dynamics.
%   Very small values of TOL are likely to cause numerical difficulties.
%
%   Note:
%      * NCFSYN assumes positive feedback.
%      * Ks minimizes the H-infinity performance
%           GAM = || [I;Ks]*inv(I-Gs*Ks)*[I,Gs] ||oo
%               = || [I;Gs]*inv(I-Ks*Gs)*[I,Ks] ||oo
%               = 1 / b(Gs,Ks)
%        In general this differs from || [I;K]*inv(I-G*K)*[I,G] ||oo.
%
%   Example:
%      G = tf([1 5],[1 2 10]);
%      W1 = tf(1,[1 0]);
%      [K,CL,gam,INFO] = NCFSYN(G,W1);
%      % Compare actual and target loop shapes
%      sigma(G*K,G*W1)
%      % Verify b(Gs,Ks)=1/gam
%      b = ncfmargin(INFO.Gs,INFO.Ks,+1);
%      [b 1/gam]
%
%   See also ncfmargin, gapmetric, makeweight, hinfsyn.

%   Copyright 1991-2004 MUSYN Inc. and The MathWorks, Inc.

% OLD HELP
% function  [sysk,emax,sysobs]=ncfsyn(sysgw,factor,opt)
%
%   Synthesizes a controller to robustly stabilize
%   a family of systems given by a ball of uncertainty
%   in the normalized coprime factors of the system description.
%
%   Inputs:
%   -------
%    SYSGW  - the weighted system to be controlled.
%    FACTOR - =1 implies that an optimal controller is required.
%             >1 implies that a suboptimal controller is required
%                achieving a performance FACTOR less than optimal.
%    OPT    - 'ref'  includes a reference input (optional).
%
%
%   Outputs:
%   -------
%    SYSK   -  H-infinity loopshaping controller
%    EMAX   -  stability margin as an indication robustness to
%              unstructured perturbations. EMAX is always less
%              than 1 and values of EMAX below 0.3 generally indicate
%              good robustness margins.
%    SYSOBS -  H-infinity loopshaping observer controller. This variable
%              is created only if FACTOR>1 and OPT = 'ref'
%
%   See also: HINFNORM, HINFSYN, and NUGAP
if nargin<2,
    disp('usage: [k,cl]=ncfsyn(sysgw,factor,opt)');
    return;
 end;

[type,p,m,n]=minfo(sysgw);

if type~='syst',
    disp('sysgw needs to be of type SYSTEM');
    return;
 end;

if nargin<3, opt='noref'; end


if strcmp(opt,'noref') & (factor==1),
     [sysnlcf,sig] = sncfbal(sysgw);
   %  [tmp1,tmp2,tmp3,n]=minfo(sysnlcf);
     emax = sqrt(1-sig(1)^2);
     sysUV=hankmr(sysnlcf,sig,0,'a');
     sysUV=cjt(sysUV);
     sysk=mscl(cf2sys(sysUV),-1);

 end; %if strcmp(opt,'noref') & (factor==1),

if strcmp(opt,'noref') & (factor>1),
     [a,b,c,d]=unpck(sysgw);
     R=eye(p)+d*d';
     S=eye(m)+d'*d;
     Hamx=[a-b*(S\d')*c, -(b/S)*b'; -c'*(R\c), -(a-b*(S\d')*c)'];
     Hamz=[(a-b*(d'/R)*c)', -c'*(R\c) ; -b*(S\b')  -(a-b*(d'/R)*c)];
     [x1,x2,fail]=ric_schr(Hamx);
     if fail == 0
	X=x2/x1;
     else
	sysk = [];
        disp('Error in solving the X Hamiltonian');
	return
     end
     [z1,z2,fail]=ric_schr(Hamz);
     if fail == 0
	Z=z2/z1;
     else
	sysk = [];
        disp('Error in solving the Z Hamiltonian');
	return
     end
     F=-S\(d'*c+b'*X);
     H=-(b*d'+Z*c')/R;
     sig=eig(Z*X);
     emax=1/sqrt(1+max(real(sig)));
     gamma=factor/emax;
     W2=eye(n)-gamma^(-2)*(eye(n)+Z*X);
     sysk=pck(a+b*F-W2\(Z*c'*(c+d*F)),W2\(Z*c'),-b'*X,-d');
 end; %if strcmp(opt,'noref') & (factor>1)

if (nargin>2) & strcmp(opt,'ref') & factor>1,
     [a,b,c,d]=unpck(sysgw);
     R=eye(p)+d*d';
     S=eye(m)+d'*d;
     Hamx=[a-b*(S\d')*c, -(b/S)*b'; -c'*(R\c), -(a-b*(S\d')*c)'];
     Hamz=[(a-b*(d'/R)*c)', -c'*(R\c) ; -b*(S\b')  -(a-b*(d'/R)*c)];
     [x1,x2,fail]=ric_schr(Hamx);
     if fail == 0
	X=x2/x1;
     else
	sysk = [];
        disp('Error in solving the X Hamiltonian');
	return
     end
     [z1,z2,fail]=ric_schr(Hamz);
     if fail == 0
	Z=z2/z1;
     else
	sysk = [];
        disp('Error in solving the Z Hamiltonian');
	return
     end
     F=-S\(d'*c+b'*X);
     H=-(b*d'+Z*c')/R;
     sig=eig(Z*X);
     emax=1/sqrt(1+max(real(sig)));
     gamma=factor/emax;
     W2=eye(n)-gamma^(-2)*(eye(n)+Z*X);
     Hi=-(b*d'+W2\(Z*c'))/R;
     sqrtS=sqrtm(S); bm=(b-W2\(Z*c'*d))/S;
     sysobs=pck(a+Hi*c,[-Hi, bm, zeros(n,m)],...
              -b'*X,[-d', zeros(m,m), -sqrtS]);
     sysk=pck(a+Hi*c-bm*b'*X,[-Hi-bm*d',-bm*sqrtS],-b'*X,[-d', -sqrtS]);
end; %  if (nargin>2) & (opt=='ref') & factor>1

if  strcmp(opt,'ref') & (factor==1),
     [sysnrcf,sig] = sncfbal(transp(sysgw));
     sysnrcf=transp(sysnrcf);
     emax = sqrt(1-sig(1)^2);
     disp(['emax = ',num2str(emax)]);
     sysUV=hankmr(sysnrcf,sig,0,'a');
     sysUV=cjt(sysUV);
     sysUV=mmult(sysUV,[-eye(p) zeros(p,m);zeros(m,p) eye(m)]);
     sysL=mmult(sysUV,[-eye(p) zeros(p,m); zeros(m,p) eye(m)],sysnrcf);
     sysUVn=mmult(minv(sysL),sysUV);
     sysUVn=sysbal(sysUVn);
     [typeUV,pUV,mUV,nUV]=minfo(sysUV);
     [typeUVn,pUVn,mUVn,nUVn]=minfo(sysUVn);
     sysUVn=strunc(sysUVn,min(nUV,nUVn)); % remove nonminimal states
     sysUVn=madd(mmult(sysUVn,[eye(p) zeros(p,2*m); zeros(m,m+p) eye(m)]),...
                  [zeros(m,p) eye(m) zeros(m,m)]);
     sysk=cf2sys(sysUVn);
  end; %if (nargin>2) & (opt=='ref') & (factor==1)
%
%