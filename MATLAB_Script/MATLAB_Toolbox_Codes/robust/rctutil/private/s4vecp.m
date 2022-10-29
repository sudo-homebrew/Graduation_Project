function  [gamma,alpha,n1vec] = s4vecp(a,b,c,d)
% Subroutine for SKEWED-MU power method.
% Find alpha such that norm([a+sqrt(alpha)*b;sqrt(alpha)*c + alpha*d]) = 1
% Gamma=1 if successful.

%  Copyright 2003-2016 The MathWorks, Inc.
if isempty(a) && isempty(b) && isempty(c)
   gamma = 1;
   alpha = 1/norm(d);
   n1vec = alpha*d;
else
   na = norm(a);
   nb = norm(b);
   nc = norm(c);
   nd = norm(d);
   % Condition norm(Expression)-1 = 0 is a quartic poly in
   % beta=sqrt(alpha). Solution guaranteed to exist when na<1 and
   % [nb nc nd] is not identically zero
   if na<1 && (nb>0 || nc>0 || nd>0)
      betavec = [nd^2 , 2*real(c'*d) , nc^2+nb^2, 2*real(a'*b), na^2-1];
      r = LOCALroots(betavec);
      beta = abs(r(imag(r)>=0));
      [minres,imin] = min(abs(polyval(betavec,beta))); % min residual
      beta = beta(imin);
      alpha = beta^2;
      n1vec = [a+beta*b;beta*(c+beta*d)];
      gamma = (minres<1e-3);
   else
      alpha = 0;
      n1vec = [a;zeros(size(d))];
      gamma = 0;
   end
end
      
function r = LOCALroots(c)
% ROOTS without error checking
if c(1)==0
   c = c(find(c~=0,1):end);
end
n = numel(c);
if n>1
   a = diag(ones(1,n-2),-1);
   a(1,:) = -c(2:n) ./ c(1);
   r = eig(a);
else
   r = zeros(0,1);
end

