function sys = rssnb(beta,n,p,m,varargin)
%RSSNB   Generate random stable continuous-time state-space models
%with H_inf norm = 1.
%
%   SYS = RSSNB(BETA,N) generates an Nth-order SISO state-space model SYS
%   whose poles are bounded in magniude by BETA.
%
%   SYS = RSSNB(BETA,N,P) generates a single-input Nth-order model with 
%   P outputs whose poles are bounded in magniude by BETA.
%
%   SYS = RSSNB(BETA,N,P,M) generates an Nth-order model with P outputs
%   and M inputs whose poles are bounded in magniude by BETA.
%
%   SYS = RSSNB(BETA,N,P,M,S1,...,Sk) generates a S1-by-...-by-Sk array of
%   state-space models with N states, P outputs, and M inputs whose poles 
%   are bounded in magniude by BETA.
%
%   See also RSS, DRSS, DRSSNB

%   Copyright 2003-2011 The MathWorks, Inc.

switch nargin
   case 0
      beta = 500*rand;
      n=max([1,round(abs(10*randn(1,1)))]);
      p=max([1,round(4*randn(1,1))]);
      m=max([1,round(4*randn(1,1))]);
   case 1
      n=max([1,round(abs(10*randn(1,1)))]);
      p=max([1,round(4*randn(1,1))]);
      m=max([1,round(4*randn(1,1))]);
   case 2
      m=1;
      p=1;
   case 3
      m=1;
end
arraydims= [varargin{:}];

% Check all inputs are positive integers
sizes = [m n p arraydims];
if ~isequal(sizes,round(sizes)) || ~all(isfinite(sizes)) || any(sizes<0) 
   error('Input arguments must be non negative integers.')
end

if isempty(beta) || isinf(beta)
   % Generate random state matrices
   a = randn([n,n,arraydims]);
   b = randn([n,m,arraydims]);
   c = randn([p,n,arraydims]);
   d = randn([p,m,arraydims]);

   % Scale poles to lie in unit disk and then bilinear transform
   I = eye(n);
   sqrt2 = sqrt(2);
   for k = 1:prod(arraydims)
     a_d = a(:,:,k);
     b_d = b(:,:,k);
     c_d = c(:,:,k);
     d_d = d(:,:,k);

     e = eig(a_d);
     emax = max([abs(e); 1e-8]);
     a_d = (1-rand^2)*a_d/emax;  

     iad = inv(I+a_d);

     a(:,:,k) = iad*(a_d-I); %#ok<*MINV>
     b(:,:,k) = sqrt2*iad*b_d;
     c(:,:,k) = sqrt2*c_d*iad;
     d(:,:,k) = d_d-c_d*iad*b_d;
   end
else
   % Generate random state matrices
   a = zeros([n,n,arraydims]);
   for j=1:prod(arraydims)
      i = 1;
      while i<=n
         if i<=n-2 && rand>0.5 %2 states left to assign
            mag = exp((log(beta)-log(beta/2000))*rand+log(beta/1000));
            theta = (1.5*pi/2 - 1.01*pi/2)*rand + 1.01*pi/2;
            sig = mag*cos(theta);
            om  = mag*sin(theta);
            a(i,i,j) = sig;
            a(i,i+1,j) = -om;
            a(i+1,i,j) = om;
            a(i+1,i+1,j) = sig;
            i = i+2;
         else
            a(i,i,j) = -exp((log(beta)-log(beta/1000))*rand+log(beta/1000));
            i = i+1;
         end
      end
   end
   b = randn([n,m,arraydims]);
   c = randn([p,n,arraydims]);
   d = zeros([p,m,arraydims]);
end

% Normalize each system to have H_inf gain = 1
sys = ss(a,b,c,d);
% Set tolerance for Hinf bound
tol = 0.001;
sysg = (1+tol)*norm(sys,inf,tol); %Upper bound on Hinf norm
sqsysg = sqrt(sysg);
for k = 1:prod(arraydims)
  % Normalize the individual matrices, then construct,
  % rather than normalizing the SS, since that incurs
  % a lot of LTI overhead.
  %sys(:,:,k) = sys(:,:,k)/sysg(k);
  b(:,:,k) = b(:,:,k)/sqsysg(k);
  c(:,:,k) = c(:,:,k)/sqsysg(k);
  d(:,:,k) = d(:,:,k)/sysg(k);
end
sys = ss(a,b,c,d);
