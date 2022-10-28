function [maxval,maxloc] = wcslft(m,L,R,n,ny,nu)
%   This is an improved version of WCRGGEN.  It does not involve
%   conversion to polynomials, and then roots.  It works directly with
%   the complex matrices, using exactly the ideas of state-space h-infinity
%   norm computation.  The basic idea is that by solving an eigenvalue
%   problem (complex, [2n x 2n]) you can get good information about the
%   level sets of the function || F_U ( M , delta I_n ) ||. 

% Copyright 2003-2006 The MathWorks, Inc.

%   Also, it goes directly at sigma_max, rather than just
%   scalar approximations.  This can easily be used both for extremely large
%   repeated scalar uncertainty blocks, as well as with frequency variable
%   blocks.  It is generally not as fast as WCRG1 and WCRGGEN for large
%   problems (order > 5 or 6, say).  But, on these larger problems,
%   it is more accurate. 

%   this is code for explicit representation of M.  Another possibility
%   is that M is only available via v -> Mv, w -> M'*w.  Or, in a simulation
%   application, it may be that M is a very structured combination of much
%   smaller matrices.

if  n==1 && ny==1 && nu==1
  [maxval,maxloc,tryr] = wcrggen1cop(m,L,R);
else
  tol = 0.001;  % XXXMINOR ? what do we want here
  itol = 10000*(n+max([ny nu]))*eps;
  
  m11 = m(1:n,1:n);
  m12 = m(1:n,n+1:n+nu);
  m21 = m(n+1:n+ny,1:n); 
  m22 = m(n+1:n+ny,n+1:n+nu);
  In = eye(n);
  Iny = eye(ny); 

  A = [m11 m12*m12';zeros(n,n) m11'];
  B = [m12*m22';m21'];
  D = m22*m22';
  C = [m21 m22*m12'];
  
  if all(diff([0 L R])>0)
    whichcase = 2; % [Ri Li]
    Ri = 1/R;
    Li = 1/L;
  elseif all(diff([L 0 R])>0)
    whichcase = 3; % (-inf Li]  [Ri inf)
    Li = 1/L;
    Ri = 1/R;
  elseif all(diff([L R 0])>0)
    whichcase = 1; % [Ri Li]
    Ri = 1/R;
    Li = 1/L;
  elseif all(diff([0 L R])>=0)
    whichcase = 5; % [Ri inf]
    if L==R
      whichcase = 6;            
    else
      Ri = 1/R;
      Li = Inf;
    end
  elseif all(diff([L R 0])>=0)
    whichcase = 4; % [-inf Li]
    if L==R
      whichcase = 6;
    else
      Ri = -Inf;
      Li = 1/L;
    end
  end
  
  if whichcase==1 || whichcase==2 || whichcase==4 || whichcase==5
    
    cleft = norm(m22 + L*((m21/(In-L*m11))*m12));
    cright = norm(m22 + R*((m21/(In-R*m11))*m12));
    
    % PJS 12/22/08 -- If L causes (In-L*m11) to be singular then
    % m21/(In-L*m11) can be a matrix of NaN. Same for R.
    if ~isfinite(cleft)
        maxval = inf;
        maxloc = L;
        return;
    elseif ~isfinite(cright)
        maxval = inf;
        maxloc = R;
        return;
    end
    
    maxloc = L;
    maxval = cleft;
    if cright>cleft
      maxloc = R;
      maxval = cright;
    end
    go = 1;            
    while go==1
       sigtry = maxval*(1+tol);
       gtry = 1/sigtry/sigtry;
       evl = eig(A+((gtry*B)/(Iny-gtry*D))*C);
       [rat,ratidx] = sort(abs(imag(evl)./real(evl)));
       lkeep=length(find(rat<itol));
       if ceil(lkeep/2) ~= floor(lkeep/2)
          lkeep = lkeep+1;
       end
       realidx = ratidx(1:lkeep);
       evlr = real(evl(realidx));
       % AKP, 2/25/2004. The 2 commented lines below were wrong.
       % The 2 lines without comment are new, and fix it.
       % Example: L=1. R =2, pole at 0, peak at 1.5.
       % EVLR will have two values near 0, which should
       % have been removed.  TMP will have the two(+) values at 1.5.
       % ultimately, CHKVAL will check some invalid DELTA values, since
       % EVLR has stuff out-of range.
       %WRONG
       %realidx = realidx(tmp);
       %if length(realidx)>0
       %CORRECT
       evlr = evlr(evlr>Ri & evlr<Li);
       if ~isempty(evlr)
          %END FIX
          go = 0;
          chkval1 = (sort(1./evlr))';
          
          % PJS 4/9/2004. It is possible for chkval1 to
          % have only one eigenvalue.
          % EXAMPLE: L=0, R=2, pole at -0.004 and peak
          % near 1.2.  The pole at -0.004 gets removed.
          %WRONG
          %chkval = chkval1(1:end-1) + diff(chkval1)/2;
          %CORRECT: Next 5 lines
          if length(chkval1)>1
             chkval = chkval1(1:end-1) + diff(chkval1)/2;
          else
             chkval = chkval1;
          end
          for i=1:length(chkval) % this loop can easily be done in parallel
             tmp = m22 + chkval(i)*((m21/(In-m11*chkval(i)))*m12);
             ntry = norm(tmp);
             if ntry>maxval
                maxval = ntry;
                maxloc = chkval(i);
                go = 1;
             end
          end
       else
          go =0;
       end
    end
  elseif whichcase==3
    cleft = norm(m22 + L*((m21/(In-L*m11))*m12));
    cright = norm(m22 + R*((m21/(In-R*m11))*m12));

    % PJS 12/22/08 -- If L causes (In-L*m11) to be singular then
    % m21/(In-L*m11) can be a matrix of NaN. Same for R.
    if ~isfinite(cleft)
        maxval = inf;
        maxloc = L;
        return;
    elseif ~isfinite(cright)
        maxval = inf;
        maxloc = R;
        return;
    end
    
    czero = norm(m22);
    maxloc = L;
    maxval = cleft;
    if cright>=cleft && cright>=czero
      maxloc = R;
      maxval = cright;
    elseif czero>=cleft && czero>=cright
      maxloc = 0;
      maxval = czero;
    end
    go = 1;   
    
    while go==1       
      sigtry = maxval*(1+tol) + 4*sqrt(eps);
      gtry = 1/sigtry/sigtry;
      candH = A+((gtry*B)/(Iny-gtry*D))*C;
      evl = eig(candH);

      % PJS 4/3/06--Function errored out below due to lkeep = 2 but
      % ratidx only of length 1.  This occurred b/c the example had
      % two real eigenvalues which were at Li+/-eps so only one
      % appeared in evlL.  The code below removes evals which straddle
      % the boundary from evlL and evlR. 
      [~,idx] = sort( real(evl) );
      evl = evl(idx);  
      evlR = evl(real(evl)>Ri);
      levlR = length(evlR);
      if ceil(levlR/2) ~= floor(levlR/2)
          evlR = evlR(2:end);
      end          
      evlL = evl(real(evl)<Li);
      levlL = length(evlL);
      if ceil(levlL/2) ~= floor(levlL/2)
          evlL = evlL(1:end-1);
      end
      
      %evl = eig(candH);
      %evlR = evl(find(real(evl)>Ri));
      %evlL = evl(find(real(evl)<Li));

      
      [rat,ratidx] = sort(abs(imag(evlR)./real(evlR)));
      lkeep=length(find(rat<itol));          
      if ceil(lkeep/2) ~= floor(lkeep/2)
         lkeep = lkeep+1;
      end   
      realidx = ratidx(1:lkeep);
      evlR = real(evlR(realidx));
      
      [rat,ratidx] = sort(abs(imag(evlL)./real(evlL)));
      lkeep=length(find(rat<itol));          
      if ceil(lkeep/2) ~= floor(lkeep/2)
         lkeep = lkeep+1;
      end   
      %try, realidx = ratidx(1:lkeep); catch, keyboard, end
      realidx = ratidx(1:lkeep); 
      evlL = real(evlL(realidx));

      chkvalR = [];
      chkvalL = [];
      if ~isempty(evlR)
         chkvalR = (sort(1./evlR))';
      end
      if ~isempty(evlL)
         chkvalL = (sort(1./evlL))';				
      end
      chkval = [chkvalL chkvalR];     
            
      if ~isempty(chkval)
         chkval = chkval(1:end-1) + diff(chkval)/2;
      end   
      go = 0;
      for i=1:length(chkval) % this loop can easily be done in parallel
         tmp = m22 + chkval(i)*((m21/(In-m11*chkval(i)))*m12);
         if any(isinf(tmp(:)))
            ntry = inf;
         else
            ntry = norm(tmp);
         end
         if ntry>maxval
           maxval = ntry;
           maxloc = chkval(i);
           go = 1;
         end
      end 
    end
    
  elseif whichcase==6
    maxval = norm(m22 + L*((m21/(In-L*m11))*m12));    
    % PJS 12/22/08 -- If L causes (In-L*m11) to be singular then
    % m21/(In-L*m11) can be a matrix of NaN. 
    if isnan(maxval)
        maxval = inf;
    end    
    maxloc = L;
    
  else
    error('Invalid Interval');
  end
end

%   % TESTING ROUTINE    
%   NPTS = 200;
%   n = 22;
%   ny = 4;
%   nu = 5;
%   m = crandn(n+ny,n+nu);
%   LR = sort(randn(1,2));
%   L = LR(1);
%   R = LR(2);
%   [u,s,v] = svd(starp((L+R)/2*eye(n),m));
%   Mscalar = daug(eye(n),u(:,1)')*m*daug(eye(n),v(:,1));
%   fin = flops;
%   t1 = clock;
%   [maxval,maxloc] = wcslft(m,L,R,n,ny,nu);
%   t2 = clock;
%   fout = flops;
%   vals = sort([linspace(L,R,NPTS) maxloc]);
%   nrm = zeros(1,NPTS+1);
%   for i=1:NPTS+1
%       tmp = starp(vals(i)*eye(n),m);
%       nrm(i) = norm(tmp);
%   end
%   t3 = clock;
%   [mxgain,delta = wcrggen(mscalar,L,R,m);
%   t4 = clock;
%   plot(vals,nrm,maxloc,maxval,'o')
%   title(['T1 = ' num2str(etime(t2,t1)) ', T2 = ' num2str(etime(t4,t3)) ...
%    ', TPLOT = ' num2str(etime(t3,t2))]);
%   disp(['RATE = ' (fout-fin)/etime(t2,t1) ' FLOP/SEC']);

