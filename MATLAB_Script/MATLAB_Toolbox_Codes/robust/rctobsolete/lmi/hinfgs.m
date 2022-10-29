function [gopt,pdk,Ropt,Sopt]=hinfgs(pds,r,gmin,tol,tolred)
% HINFGS   Design gain-scheduled H-infinity controllers
%
%   [GOPT,PDK,R,S]=HINFGS(PDS,R,GMIN,TOL,TOLRED) performs gain-scheduled
%   H-infinity control for the parameter-dependent systems with an affine
%   dependence on the time-varying parameters. These parameters are assumed
%   to be measured in real-time. This function implements the quadratic
%   H-infinity performance approach.
%
%   Input:
%    PDS       parameter-dependent plant (see PSYS).
%    R         1x2 vector specifying the dimensions of D22:
%                     R(1) = nbr of measurements
%                     R(2) = nbr of controls
%    GMIN      target value for GOPT.  Set GMIN = 0 to compute the
%              optimum GOPT, and set GMIN = GAMMA to test whether
%              the performance GAMMA is achievable   (Default = 0)
%    TOL       desired relative accuracy on the optimal performance
%              GOPT  (Default = 1e-2)
%    TOLRED    optional (default value = 1e-4).  Reduced-order
%              synthesis is performed whenever
%                   rho(X*Y) >=  (1 - TOLRED) * GAMA^2
%
%   Output
%    PDK       polytopic representation  PDK = [K1 , ... , Kn]  of
%              the gain-scheduled controller. The vertex controller
%              Kj  is associated with the  j-th  corner of the
%              parameter box as given by POLYDEC
%    GOPT      optimal performance if GMIN=0, and some achievable
%              performance < GMIN  otherwise
%    R,S       solutions of the characteristic LMI system
%
%   See also  PSYS, POLYDEC, PDSIMUL, HINFLMI.

% Author: P. Gahinet  6/94
%   Copyright 1995-2011 The MathWorks, Inc.
if nargin < 2,
   error('usage: [gopt,pdk,R,S]=hinfgs(pds,r,gmin,tol,tolred)');
elseif length(r)~=2,
   error('R must be a two-entry vector');
elseif min(r)<=0,
   error('The entries of R must be positive integers');
elseif ~ispsys(pds),
   error('PDS must be a parameter-dependent system');
elseif pds(7,1)~=10,
   error('Not available when the E matrix varies');
elseif nargin == 2,
   gmin=0;  tol=1e-2; tolred=1e-4;
elseif nargin == 3,
   tol=1e-2; tolred=1e-4;
elseif nargin == 4,
   tolred=1e-4;
end
Ropt=[]; Sopt=[]; pdk=[];


% convert to polytopic if necessary
if strcmp(psinfo(pds),'aff'), pds=aff2pol(pds); end


% tolerances
macheps=mach_eps;
tolsing=sqrt(macheps);
penalty=max(gmin,1)*1e-8;


% dimensions
p2=r(1);  m2=r(2);
[~,nv,ns,ni,no] = psinfo(pds);       % nv = number of vertices
p1=no-p2;   m1=ni-m2;
if p1<0 || m1<0,
   error('D11 is empty according to the input R');
end

% check that b2,c2,d12,d21,d22 cte
[~,~,b2,~,c2,~,d12,d21,d22]=localHinfPar(psinfo(pds,'sys',1),r);
for ii=2:nv,
   [~,~,b2x,~,c2x,~,d12x,d21x,d22x]=localHinfPar(psinfo(pds,'sys',ii),r);
   if norm(b2-b2x,1) > 0 || norm(d12-d12x,1) > 0,
      error('B2 and D12 should be constant: filter the input u');
   elseif norm(c2-c2x,1) > 0 || norm(d21-d21x,1) > 0,
      error('C2 and D21 should be constant: filter the output y');
   elseif norm(d22-d22x,1) > 0,
      error('D22 should be constant: filter u or y');
   end
end


%%%%% LMI synthesis set-up

[~,~,~,~,NR]=svdparts([b2;d12],0,tolsing);
%cnr=size(NR,2);
NR=mdiag(NR,eye(m1));

[~,~,~,~,~,NS]=svdparts([c2,d21],0,tolsing);
%cns=size(NS,2);
NS=mdiag(NS,eye(p1));


% vars
setlmis([]);
lmivar(1,[ns 1]);        % R
lmivar(1,[ns 1]);        % S
lmivar(1,[1 0]);         % gamma

% terms
for ii=1:nv,
   
   [A,B1,~,C1,~,D11]=localHinfPar(psinfo(pds,'sys',ii),r);
   
   jj=newlmi;
   aux1=[A;C1]; aux2=[eye(ns) zeros(ns,p1)];
   lmiterm([jj 0 0 0],NR);
   lmiterm([jj 1 1 1],aux1,aux2,'s');
   lmiterm([jj 1 1 3],[zeros(ns,p1);eye(p1)],[zeros(p1,ns),-eye(p1)]);
   lmiterm([jj 2 1 0],[B1' D11']);
   lmiterm([jj 2 2 3],-1,1);
   
   jj=newlmi;
   aux1=[A,B1]; aux2=[eye(ns);zeros(m1,ns)];
   lmiterm([jj 0 0 0],NS);
   lmiterm([jj 1 1 2],aux2,aux1,'s');
   lmiterm([jj 1 1 3],[zeros(ns,m1);eye(m1)],[zeros(m1,ns),-eye(m1)]);
   lmiterm([jj 2 1 0],[C1 D11]);
   lmiterm([jj 2 2 3],-1,1);
end

jj=newlmi;
lmiterm([-jj 1 1 1],1,1);
lmiterm([-jj 2 1 0],1);
lmiterm([-jj 2 2 2],1,1);

lmisys=getlmis;


% objective = gamma + eps * Trace(R+S)

nvars=ns*(ns+1)+1;
Rdiag=diag(decinfo(lmisys,1));
Sdiag=diag(decinfo(lmisys,2));
c_obj = zeros(nvars,1);
c_obj(nvars)=1;                     % gamma
c_obj(Rdiag)=penalty*ones(ns,1);    % eps*Trace(R)
c_obj(Sdiag)=penalty*ones(ns,1);    % eps*Trace(S)



options=[tol 200 1e8 0 0];

[gopt,xopt]=mincx(lmisys,c_obj,options,[],gmin);


if ~isempty(gopt),
   Ropt=dec2mat(lmisys,xopt,1);
   Sopt=dec2mat(lmisys,xopt,2);
   gopt=gopt-penalty*(sum(diag(Ropt))+sum(diag(Sopt)));
   fprintf('\n Optimal quadratic RMS performance:  %9.4e\n',gopt);
else
   gopt=Inf; return
end



% controller computation

orderK=-1;

while isempty(pdk),
   
   for ii=1:nv,
      kv=klmi(psinfo(pds,'sys',ii),r,gopt,...
         Ropt,gopt*eye(ns),Sopt,gopt*eye(ns),tolred);
      
      nk=sinfo(kv);   % order of K
      
      if ~isempty(pdk) && nk~=orderK,
         pdk=[];  gopt=1.01*gopt;  break
      else
         orderK=nk;  pdk=[pdk kv]; %#ok<*AGROW>
      end
   end
   
end;

pdk=psys(pdk);


if norm(d22,1)>0,
   warning('When D22 is nonzero, feed y-D22*u instead of y to the controller (u being the controller output).') %#ok<WNTAG>
end



%---------- local functions ---------------------------------

function [a,b1,b2,c1,c2,d11,d12,d21,d22]=localHinfPar(sys,r)

[rp,cp]=size(sys);
na=sys(1,cp);
p2=r(1); m2=r(2);
p1=rp-(na+p2)-1; m1=cp-(na+m2)-1;

[a,b1,c1,d11]=ltiss(sys);
b2=b1(:,m1+1:m1+m2);  b1=b1(:,1:m1);
c2=c1(p1+1:p1+p2,:); c1=c1(1:p1,:);
d12=d11(1:p1,m1+1:m1+m2);   d21=d11(p1+1:p1+p2,1:m1);
d22=d11(p1+1:p1+p2,m1+1:m1+m2);  d11=d11(1:p1,1:m1);
