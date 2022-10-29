function [num,den,tmin] = fitmagfrdeng(mag,theta,dd,nd,wt,lb,ub,ttol,tmax)
% MAG: NPTS-by-1
% THETA: NPTS-by-1
% DD: scalar
% ND: scalar
% WT: NPTS-by-1
% LB: NPTS-by-1
% UB: NPTS-by-1
% TTOL: scalar
% TMAX: scalar

%   Copyright 2003-2008 The MathWorks, Inc.

% Initialize outputs
num = [];
den = [];
tmin= [];

% Form matrices used in linear program.  
% Constraints are:
%  1) Original cutting planes (A1, c1)
%  2) Upper Bound Constraint  (A2,c2)
%  3) Lower Bound Constraint (A3,c3)
%  4) Upper Bound for Objective (A4,c4) 
%  5) Lower Bound for Objective (A5,c5)
%  6) New cutting planes (A6, c6)
npts = length(theta);
z = exp(1i*theta);
wtinv = 1./wt;
cosmat = cos(theta*(dd:-1:0));
magsq = mag.^2;

%  1) Original cutting planes (A1, c1)
% PJS 5/11/06--Add cutting planes to enforce pos. def.
cptol = 1e-10;
Ncp = 100;
cptheta = linspace(0,pi,Ncp); 
cptheta = cptheta(:);
cpcosmat = cos(cptheta*(dd:-1:0));
A1 = [-cpcosmat(:,(end-nd):end) zeros(Ncp,dd+1)];
c1 = repmat(-cptol,Ncp,1);
A1 = [A1; zeros(Ncp,nd+1) -cpcosmat];
c1 = [c1; repmat(-cptol,Ncp,1)];               

%  2) Upper Bound Constraint  (A2,c2)
idx2 = find( ~isinf(ub) );
ubsq = ub(idx2).^2;
A2 = [lrscale(cosmat(idx2,(end-nd):end),1./ubsq,[]) -cosmat(idx2,:)]; 
c2 = zeros(length(idx2),1);

%  3) Lower Bound Constraint (A3,c3)
idx3 = find( ~isinf(lb) );
lbsq = lb(idx3).^2;
A3 = [lrscale(cosmat(idx3,(end-nd):end),-1./lbsq,[]) cosmat(idx3,:)]; 
c3 = zeros(length(idx3),1);

%  4) Upper Bound for Objective (A4,c4) 
% Not needed if upper bound constraint already enforces |n/d|<=magdata
idx4 = find( mag<ub );
Anum4 = lrscale(cosmat(idx4,(end-nd):end),1./magsq(idx4),[]);
Aden4 = cosmat(idx4,:); 
c4 = zeros(length(idx4),1);

%  5) Lower Bound for Objective (A5,c5)
% Not needed if lower bound constraint already enforces |n/d|>=magdata
idx5 = find( mag>lb );
Anum5 = lrscale(cosmat(idx5,(end-nd):end),1./magsq(idx5),[]);
Aden5 = cosmat(idx5,:); 
c5 = zeros(length(idx5),1);

%  6) New cutting planes (A6, c6)
A6 = [];
c6 = [];

% Initialize Bisection
tub = min(tmax,10); 
tlb = 0;
tub = max(tub,tlb+2*ttol); % 4/4/09 PJS: Ensure at least one iteration 
tmax = max(tmax,tlb+2*ttol); % 4/4/09 PJS: Ensure at least one iteration
%t = (tub+tlb)/2;
t = tub;

% GJB 17Sept08: pete: linprog - goofy and SLOW results, use LMI always
% PJS 15Nov09: Use interior point LP solver
%  solverflg = 1 --> linprog
%  solverflg = 2 --> LMILab
%  solverflg = 3 --> Our interior point solver
solverflg = 3;

% Only constraints 4-6 are modified during the loop. When using 
% LMILab, create constraints 1-3 before entering the loop to
% prevent repeated calls to lmiterm
if solverflg==2
    A = [A1;A2;A3];
    c = [c1;c2;c3];
    N123 = size(A,1);
    
    setlmis([]);
    xvar = lmivar(2,[size(A,2) 1]);        
    for i1 = 1:N123
        lmiterm([i1 1 1 xvar],A(i1,:),1/2,'s')
        lmiterm([-i1 1 1 0],c(i1))
    end
    lmisys123 = getlmis;          
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%     Solve via bisection and linear programming
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
goodpolya = []; goodpolyb = [];
if solverflg==1
    opts = optimset('Algorithm','interior-point-legacy','Display','off');
elseif solverflg==2
    opts = zeros(1,5);
    opts(end) = 1;
end

infeascnt = 0;
feasible = false;
cpcnt = 0;

% XXX 27Mar09-- tub<=2*tmax allows tub to fail on tmax-epsilon and
% then get one more try after doubling.
while tub-tlb > (ttol*t +ttol) && (tub <= 2*tmax)
    % Set up Linear Program:  Find x such that A*x <= c
    % where x= [a_nd ... a0  b_{dd} b_{dd-1} ... b0]
    % [PJS 5/12/06: Since the optimal b_dd may be <0, can't assume b_dd=1]
    R = -1-t*wtinv(idx4);
    A4 = [Anum4 lrscale(Aden4,R,[])];
    L = 1./(1+t*wtinv(idx5));    
    A5 = [-Anum5 lrscale(Aden5,L,[])];

    if  solverflg==1 
        % Check if linprog is in path
        lpflg = exist('linprog','file');

        if lpflg==0
            error('linprog does not exist in the path');
        end
        A = [A1;A2;A3;A4;A5;A6];
        c = [c1;c2;c3;c4;c5;c6];    
        [x,fval,flg]=linprog([],A,c,[],[],[],[],[],opts);
    elseif solverflg==2
        A = [A4;A5;A6];
        c = [c4;c5;c6];
        setlmis(lmisys123);
        for i1 = 1:size(A,1);    
            lmiterm([N123+i1 1 1 xvar],A(i1,:),1/2,'s')
            lmiterm([-(N123+i1) 1 1 0],c(i1))
        end

        lmisys = getlmis;          
        opts = zeros(1,5);
        opts(end) = 1;
        [fval,x] = feasp(lmisys,opts);
        if isempty(fval) || fval > 0
            flg = -1;
        else
            flg = 1;
        end        
    else
        % Solve with Mehrotra's predictor-corrector method
        A = [A1;A2;A3;A4;A5;A6];
        c = [c1;c2;c3;c4;c5;c6];    
 
        % We can't assume b_dd=1 but we can assume b0=1. Normalizing b0=1
        % ensures the feas set is bounded. This allows us to set cptol=0
        % since normalizing + the interior point solver ensures any feas
        % solution will strictly satisfy the cutting plane constraints.
        % (Setting cptol=0 improves perf on problems with small magdata).               
        An = A(:,1:end-1); 
        cn = -A(:,end); % cptol=0 --> c=0 --> cn = c-A(:,end) = -A(:,end) 
        [xn,flg] = LOCALlppc(An,cn); 
        x = [xn;1];              
	end

	% Check if LP was feasible: GJB changes 7Oct05
	if flg<=0  %LP infeasible   
        if ~feasible
            infeascnt = infeascnt+1;
            tlb = t;
            if tub>=tmax
                tub=2*tub;
            else
                tub = min(2*tub,tmax);
            end
        else
            tlb=t;
        end
        t = (tub+tlb)/2;     
	else  % LP feasible
        % Check that polynomials related to a and b have no roots 
        % on the unit circle
        a = x(1:(nd+1));
        polya = [ (a(1:end-1)/2)'   a(end)  fliplr((a(1:end-1)/2)')];
        rootsa = roots(polya);

        % GJB 2Nov04: pete says increase tolerance
        % aidx= find( abs(abs(rootsa)-1) < 1e3*nd*eps
        aidx= find( abs(abs(rootsa)-1) < 1e6*nd*eps );
        newtheta = [];
        if ~isempty(aidx)
            % Found root of a on unit disk. Store cutting-plane frequencies.
            temp  = angle(rootsa(aidx))';
            idx = find( (temp>=0) & (temp <=pi) );
            newtheta = [.99; 1; 1.01]*temp(idx);
            newtheta = reshape(newtheta, 3*length(idx) ,1);
        end

        b = x(nd+2:end);
        polyb = [ (b(1:end-1)/2)'   b(end)  fliplr((b(1:end-1)/2)')];
        rootsb = roots(polyb);
        % GJB 2Nov04: pete says increase tolerance
        % bidx= find( abs(abs(rootsb)-1) < 1e3*dd*eps );
        bidx= find( abs(abs(rootsb)-1) < 1e6*dd*eps );
        if ~isempty(bidx)
            % Found root of b on unit disk. Store cutting-plane frequencies.
            temp  = angle(rootsb(bidx))';
            idx = find( (temp>=0) & (temp <=pi) );
            newtheta2 = [.99; 1; 1.01]*temp(idx);
            newtheta2 = reshape(newtheta2, 3*length(idx) ,1);
        end

        if ( ~isempty(aidx) || ~isempty(bidx) )
            cpcnt = cpcnt+1;
            % Pete changed 7Oct05 to address overbounding issues
            if cpcnt < 50 %20
                    % Form cutting planes:
                    %    a(w0)>= tol for w0 where a(w0)=0
                    %    and b(w1)>= tol for w1 where b(w1)=0       
                    if ~isempty(aidx)
                    nnt = length(newtheta);
                    newcosmat = cos(newtheta*(nd:-1:0)); 
                    A6 = [A6; -newcosmat zeros(nnt,dd+1)];
                    c6 = [c6; repmat(-cptol,nnt,1)];
                end
                if ~isempty(bidx)
                    nnt2 = length(newtheta2);
                    newcosmat2 = cos(newtheta2*(dd:-1:0)); 
                    A6 = [A6; zeros(nnt2,nd+1) -newcosmat2];
                    c6 = [c6; repmat(-cptol,nnt2,1)];               
                end                 
            else           
                flg = -1;
                cpcnt = 0;
                if ~feasible
                    infeascnt = infeascnt+1;
                    tlb = t;
                    if tub>=tmax
                        tub=2*tub;
                    else
                        tub = min(2*tub,tmax);
                    end           
                    %tub = 2*tub;
                else
                    tlb=t;
                end
                t = (tub+tlb)/2;     
            end
        else
            feasible = true;
            cpcnt = 0;
            % Found a solution that can be factorized
            %xgood = x; Agood=A; cgood = c;Rgood = R; tgood = t;
            goodpolya = polya; 
            goodpolyb = polyb;
            g=abs(polyval(goodpolya,z)./polyval(goodpolyb,z));
            fit=max([(g./magsq(1:npts)-1)./wtinv(1:npts);...
            (magsq(1:npts)./g-1)./wtinv(1:npts)]);
            % We should have: fit <= t 
            tub = fit;        
            t = (tub+tlb)/2;
        end
    end % if flg<=0 
%   fprintf('tub = %4.2f \t t = %4.2f \t tlb = %4.2f',tub,t,tlb);
%   fprintf('\t flg = %i \t feas = %i \n',flg,feasible);
end % while tub-tlb > ttol
%fprintf('\n');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%     Use spectral factorization to recover sys
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
if isempty(goodpolya)
  %disp('No feasible solution. Increase tmax, nd, or dd.');
  num = [];
  den = [];
  tmin = tmax;
  return;
else
  % Store stable roots and then factor goodpolya  
  rootsa = roots(goodpolya); 
  stbrootsa = rootsa(abs(rootsa)<1-10*eps);

  num = poly(stbrootsa);
  num = real(num*sqrt( goodpolya(end)*prod(-1./stbrootsa) ));

  % Store stable roots and then factor goodpolya
  rootsb = roots(goodpolyb);
  stbrootsb = rootsb(abs(rootsb)<1-10*eps);
  den = poly(stbrootsb);
  den = real(den*sqrt( goodpolyb(end)*prod(-1./stbrootsb) ));

  % Find tmin
  %g=abs( polyval(goodpolya,z)./polyval(goodpolyb,z) );
  %tmin=max([(g./magsq(1:npts)-1)./wtinv(1:npts);...
  %            (magsq(1:npts)./g-1)./wtinv(1:npts)]);
end

function [xopt,flg,yopt] = LOCALlppc(A,b)
% function [xopt,flg,yopt] = lppc(A,b)
%
% This function uses Mehrotra's predictor-corrector method to solve the LP
% feasibility problem:  
%       Find x satsifying A*x<=b.
% The problem is recast into a primal/dual linear programming problem.
% The optimization terminates if a feasible point is found or if the
% dual problem proves infeasibility.
% 
% Inputs:
%   A: m-by-n matrix. 
%   b: m-by-1 vector. 
%
% Outputs:
%   xopt: Primal point (n-by-1). 
%   flg:  flg=1 if the LP is feasible, i.e. all(A*xopt<=b), else flg=0
%   yopt: Dual point (m-by-1). 
%

% 11/22/2009   PJS Initial coding
%
% Refs:
%  **S. Mehrotra, "On the Implementation of a Primal-Dual Interior Point
%  Method," SIAM Journal on Optimization, Vol. 2, #4, pp. 575–601, 1992.
%  **J. Nocedal and S.J. Wright, "Numerical Optimization," Springer, 2006.
%  Chapter 14: "Linear Programming: Interior-Point Methods"
%  **S.J. Wright, "Primal-Dual Interior Point Methods," SIAM, 1997
%  Chapter 10: "Practical Aspects of Primal-Dual Algorithms"


%-----------------------------------------------------------------
% Numerical Conditioning
%-----------------------------------------------------------------
A0 = A; 
[m0,n0] = size(A0); 

% Remove trivial constraints (block rows of zeros in A)
rowscl = sqrt( max(abs(A),[],2) );
nzidx = find(rowscl~=0);
zidx = find(rowscl==0);
if ~isempty(zidx)
    if any(b(zidx)<0)
        xopt = zeros(n0,1);
        flg = -1;
        yopt = [];
        return
    end
    A(zidx,:)=[];
    b(zidx,:)=[];
    rowscl(zidx)=[];  
end

% Scale LP constraints  for numerical conditioning
A = lrscale(A,1./rowscl,[]);
b = b./rowscl;

%-----------------------------------------------------------------
% Solve LP feasibility problem
%-----------------------------------------------------------------
if all(b>=0)
    xopt = zeros(n0,1);
    flg = 1;
    yopt = [];
    return
else    
    % Call Local predictor-corrector algorithm
    [y0,x0] = LOCALpc(A',zeros(n0,1),b);                
    xopt = x0;
    flg = all( A*x0<=b );
    yopt = zeros( m0 , 1);
    yopt(nzidx) = y0./rowscl;
end

return;

%-----------------------------------------------------------------
% [xopt,yopt] = LOCALpc(A,b,c)
%
% This function uses Mehrotra's predictor-corrector algo to solve:
%    [Primal]  p*=min c'*x  subject to A*x=b, x>=0
%    [Dual]    d*=max b'*y  subject to A'*y+s=c, s>=0
% The primal/dual notation is flipped from that given above so that
% it matches the references.
function [xopt,yopt,sopt] = LOCALpc(A,b,c)

% Algorithm Parameters
[m,n] = size(A);
eta = 0.9;              % Step length factor
tol = 1e-9;             % Stop Tolerance
maxstep = 100;          % Max number of steps
x0fac = 10*max(abs( [A(:); b(:); c(:)] ));  % Initial condition parameter
cstop = 2*x0fac*abs(sum(c));  % Stop condition on c'*x

% Initialization 
step = 0;       
x = x0fac*ones(n,1);
s = x; 
y = zeros(m,1);

% Begin Primal-Dual Iteration
while 1
    % Compute primal/dual/complementarity residuals & duality measure
    Rc = A'*y+s-c;
    Rb = A*x-b;
    Rxs1 = x.*s;
    mu = (x'*s)/n;
        
    % Check stopping conditions
    % (These conditions are specific for the feasibility problem)
    nRc = norm(Rc);
    nRb = norm(Rb);
    nR=max( nRc ,nRb );
    if all(A'*y<=c) || (c'*x < -cstop) || (step > maxstep) ...
            || ( (nR <= tol) && (mu*n <= tol) )
        break;
    end
        
    % Solve for affine scaling direction (predictor step)
    sinv = 1./s;
    Ax = lrscale(A,[],x);
    Asinv = lrscale(A,[],sinv);
    H = (Ax*Asinv');
    rcondH = rcond(H);

    % Note: H becomes ill-condioned as the algo converges but the 1999
    % SIAM J. Optim Paper by Wright (vol9, #4) shows that the errors
    % introduced into the primal-dual step are less significant than
    % expected due to the structure of the equations. 
    g1 = -Rb-Ax*(Rc./s)+Asinv*Rxs1;
    if rcondH >= 10*eps        
        % Solve normal equations
        dyAff = H\g1;        
    else
        % Switch from mldivide to ldl to avoid ill-conditioning warnings.    
        [LL,DD]=ldl(H);
        LLt = LL';           
        DDinv = diag(DD);
        DDinv( DDinv~=0 ) = 1./DDinv( DDinv~=0 );
        dyAff = LLt\lrscale(LL\g1, DDinv ,[] );
        %dyAff = LLt\lrscale(LL\g1, 1./diag(DD) ,[] );
    end
    dsAff = -Rc-A'*dyAff;
    dxAff = (-Rxs1-x.*dsAff)./s;
        
    % Primal/Dual Affine-Scaling Steplengths
    axAff = min( 1,  -1/min( min(dxAff./x), -1) );
    asAff = min( 1,  -1/min( min(dsAff./s), -1) );
    muAff = (x+axAff*dxAff)'*(s+asAff*dsAff)/n;
    
    % Centering Parameter
    sig = (muAff/mu)^3;
    
    % Solve for corrector step
    Rxs2 = x.*s+dxAff.*dsAff-sig*mu;
    g2 = -Rb-Ax*(Rc./s)+Asinv*Rxs2;
    if rcondH >= 10*eps
        % Solve normal equations
        dy = H\g2;
    elseif 1
        % LDL factorization from above H=LL*DD*LL' can be re-used here
        dy = LLt\lrscale(LL\g2, DDinv ,[] );
        %dy = LLt\lrscale(LL\g2, 1./diag(DD) ,[] );
    end
    ds = -Rc-A'*dy;
    dx = (-Rxs2-x.*ds)./s;

    % Calculate step length
    ax = -1/min( min(dx./x), -1);
    ax = min( 1, eta*ax );
    as = -1/min( min(ds./s), -1);
    as = min( 1, eta*as );
    
    % Take step
    x = x + ax*dx;
    y = y + as*dy;
    s = s + as*ds;
    
    % Update step count
    step = step+1;

%    fprintf(['step = %d \t cx = %s \t by = %s \t mu*n = %s \t'...
%            'norm(r) = %s \t rcond(H) =%s \n'], ...
%            step,num2str(c'*x,3),num2str(b'*y,3), num2str(mu*n,3),...
%            num2str(nR,3) , num2str(rcond(H),3) );        
end
%fprintf('step = %d \n',step);

% Create output variables
xopt = x;
yopt = y;
sopt = s;



