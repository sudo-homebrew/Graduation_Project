function mu = getssv(M,Dr,Dc,G)
% Computes the smallest mu such that
%    M'*Dr*M + j*(G*M-M'*G) < mu^2 Dc
% when Dc>0, and returns mu=Inf when Dc is not positive definite.
% 
% NOTE: Inputs Dr and Dc should be symmetric to ensure MU is real. 

% Copyright 2019 The MathWorks, Inc.

% assert(norm(Dr-Dr',1)==0)
% assert(norm(Dc-Dc',1)==0)
[Rc,p] = chol(Dc);
if p>0
   % Dc not positive definite
   mu = Inf;
elseif isempty(G)
   % D scaling only
   [Rr,p] = chol(Dr);
   if p>0
      % Dc not positive definite
      mu = Inf;
   else
      mu = ltipack.util.gsvmax(Rr*M,Rc,0);
   end
else
   X = Rc'\(M'*Dr*M+2i*G*M)/Rc;
   mu = sqrt(max(0,max(eig((X+X')/2))));
end