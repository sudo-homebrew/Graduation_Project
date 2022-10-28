function [sysout,sig,errc,Tkeep] = sbmudl(sys,tol)
%SBMUDL  Balancing LTI system to improve DKSYN numerics
%
%[SYSOUT,HSV] = SBMUDL(SYS,TOL) balanced realization of the system
% state-space model, SYS. SYSOUT is the balanced LTI system and HSV
% is a vector containing the Hankel-singular values greater than TOL. 
% If TOL is omitted then it is set to max(HSV(1)*1.0E-12,1.0E-16).
%
% See also: BALANCMR

%   Copyright 2003-2010 The MathWorks, Inc.

% Get handle of local subroutine
nin = nargin;
if nin==2 && isa(tol,'char')
    sysout = @LOCALsbmudl;
    return
end

if nin == 1
   tol = [];
end
if isa(sys,'ss') && ndims(sys)==2
   Ts = get(sys,'Ts');
   if Ts == 0
      [A,B,C,D] = ssdata(sys);
      [Ab,Bb,Cb,Db,sig,errc,Tkeep] = LOCALsbmudl(A,B,C,D,tol);
      sysout = ss(Ab,Bb,Cb,Db);
   else
      error('Does not handle discrete systems')
   end
end

function [Ab,Bb,Cb,Db,sig,errc,Tkeep] = LOCALsbmudl(A,B,C,D,tol)

errc = 0;
[~,m]=size(B); [p,n]=size(C);
[T,A]=schur(A);
B = T'*B;
C = C*T;
% check that A is stable.
SLOPE = 0.0001;
etol = n*n*100*eps;
eAmat = eig(A);
if ~all(real(eAmat)<-SLOPE*abs(imag(eAmat))-etol) 
   errc = 1;
   Ab = [];
   Bb = [];
   Cb = [];
   Db = [];
   sig = [];
   Tkeep = [];
   return
end

S = sjh6(A,C);
perm = n:-1:1;
R = sjh6(A(perm,perm)',B(perm,:)');
R = R(perm,perm)';
[U,T,V] = svd(S*R);
sig = diag(T);
T = U'*S; Tkeep = T;
B = T*B; A = T*A;
T = R*V;
C = C*T; A = A*T;
% calculate the truncated dimension nn
if isempty(tol)
 % thsi needs to be looked at carefully for AUTOREDU
 % recall how the elasticity problems with E LARGE did
 % not autoredu properly, by sclaing E to 1 made everything
 % work.  there are some things which are not invariant to scaling.
   tol = max([sig(1)*1.0E-12,1.0E-16]);
end
nn = n;
for i=n:-1:1, 
   if sig(i)<=tol,
      nn=i-1; 
   end
end
if nn==0
   Db = D;
   Ab = []; Bb = []; Cb = []; sig = [];
else
   sig = sig(1:nn);
   % diagonal scaling  by sig(i)^(-0.5)
   irtsig = sig.^(-0.5);
   onn=1:nn;
   A(onn,onn)=A(onn,onn).*(irtsig*irtsig');
   B(onn,:)=(irtsig*ones(1,m)).*B(onn,:);
   C(:,onn)=C(:,onn).*(ones(p,1)*irtsig');
   Ab = A(onn,onn);
   Bb = B(onn,:);
   Cb = C(:,onn);
   Db = D;
end
