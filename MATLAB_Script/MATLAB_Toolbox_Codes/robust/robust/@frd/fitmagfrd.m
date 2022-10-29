function sys = fitmagfrd(data,ord,reldeg,weight,constraint,opts)
%FITMAGFRD   Fit frequency response magnitude data with stable,
% minimum-phase state-space model using a version of log-Chebyshev
% magnitude design.
%
%   B = FITMAGFRD(A,N) is a stable, minimum-phase SS object, with
%   state-dimension N, whose frequency response magnitude closely
%   matches the magnitude data in A.  A is a 1-by-1 FRD object,
%   and N is a nonnegative integer.
%
%   B = FITMAGFRD(A,N,RD) forces the relative degree of B to be RD.  RD
%   should be a nonnegative integer.  The default value for RD is 0.
%   The default for RD can also be specified by setting RD = [].
%
%   B = FITMAGFRD(A,N,RD,WT) uses the magnitude of WT to weight the
%   optimization fit criteria.  WT may be a DOUBLE, SS or FRD.  If
%   WT is a scalar, then it is used to weight all entries of the
%   error criteria (A-B).  If WT is a vector, it should be the same
%   size as A, and each individual entry of WT acts as a weighting
%   function on the corresponding entry of (A-B).  The default value
%   for WT is 1, and can also be specified by setting WT = [].
%
%   B = FITMAGFRD(A,N,RD,WT,C) enforces additional magnitude constraints
%   on B, specified by the values of C.LowerBound and C.UpperBound.  These 
%   should be empty, DOUBLE, or FRD (with Frequency equal to A.Frequency).  
%   If C.LowerBound is non-empty then the magnitude of B will be constrained 
%   to lie above C.LowerBound. Similarly, the UpperBound field can be used 
%   to specify an upper bound on the magnitude of B.
%
%   Alternatively, if C is a DOUBLE, or FRD (with C.Frequency equal to
%   A.Frequency), then the upper and/or lower bound constraints on B are
%   taken directly from A, as: 
%      if C(w) == -1, then enforce abs(B(w)) <= abs(A(w))
%      if C(w) ==  1, then enforce abs(B(w)) >= abs(A(w))
%      if C(w) ==  0, then no additional constraint
%   
%   FITMAGFRD uses a version of log-Chebyshev magnitude design, solving 
%      min f     subject to (at every frequency point in A):  
%              |d|^2 /(1+ f/WT) < |n|^2/A^2 < |d|^2*(1 + f/WT)
%                   additional constraints imposed with C
%   where B = n/d.  n and d have orders (N-RD) and N, respectively.
%   The problem is solved using bisection on f and linear programming 
%   for fixed f.
%
%   Finally, an alternate approximate method for stable, minimum-phase fits
%   of magnitude data is B = FITFRD(GENPHASE(A),N,RD,WT).  This is often
%   faster than FITMAGFRD, but cannot be used with the lower and upper
%   bound constraints defined by C.
%
%   See also: FITFRD, GENPHASE.

%   Copyright 2003-2011 The MathWorks, Inc.


%    OPTS       - (optional, default=[*** 0.01]) 
%         1 x 2 vector of options = [tmax ttol]. As a default,
%         tmax is a crude upper bound on problem.
%XXXMAJOR error messages are not clear regarding that A and WT and
% CONSRAINT should all be 1-by-1.
%XXXMINOR lots of the code has NTF sprinkled throughout, since
% it comes from FITFRD.  Here, Ntf must be 1, so some clarity
% could be achieved through simplification.
% XXXMINOR we should inherit all of the dynamic system properties from FRDATA
% Coding:  8/29/2001   PJS      Based on magfit.m 
%         10/11/2001   PJS      Map jw-axis to unit disk for better
%                               numerical conditioning
%         10/17/2001   PJS      Improved cutting plane constraints
%         10/24/2001   PJS      Added code for strictly proper fitting
%         08/02/2003   PJS      Final improvements
%         03/29/2009   PJS      Add more general upper/lower bounds
narginchk(2,5)
ni = nargin;
if ni<3
   reldeg = [];
end
if ni<4
   weight = [];
end
if ni<5
   constraint = [];
end
if ni<6
   opts = [];
end

szf = size(data);
if ~(isa(data,'frd') && isequal(szf,[1 1]))
   ctrlMsgUtils.error('Robust:fitting:FITMAGFRD1');
end
Ntf = 1;

% Get response data and frequency vector in rad/TimeUnit
% Note: * Ignore delays since fitting the magnitude
%       * ROMEGA expressed in rad/TimeUnit
[Resp,Freq,Ts] = frdata(data);
respdata = abs(Resp(:));
MaxVal = max(respdata);
MinVal = min(respdata);
SclFac = 1/sqrt(MinVal*MaxVal);
respdata = respdata*SclFac;
romega = Freq * funitconv(data.FrequencyUnit,'rad/TimeUnit',data.TimeUnit);
nfreq = length(romega);
if Ts==0
   if romega(1)==0
      w1 = romega(2);
   else
      w1 = romega(1);
   end
   w2 = romega(end);
   % [alpha,wm,domeorig,L,P,ejtheta] = hp2dmax(w1,w2,romega);
   [alpha,~,domeorig,L] = hp2dmax(w1,w2,romega);
else
   thetavec = romega*abs(Ts);
   keep = find(thetavec>=0 & thetavec<=pi);
   if length(keep)~=nfreq
      ctrlMsgUtils.error('Robust:fitting:InvalidDiscreteFrequencies')
   end
   if thetavec(1)==0
      t1 = thetavec(2);
   else
      t1 = thetavec(1);
   end
   if thetavec(end)==pi
      t2 = thetavec(end-1);
   else
      t2 = thetavec(end);
   end
   % [b,tm,domeorig,L,P,ejpsi] = d2dmax(t1,t2,thetavec);
   [b,~,domeorig,L] = d2dmax(t1,t2,thetavec);
end

% Get WEIGHT as NFREQ x 1 double array
if isnumeric(weight)
   if isempty(weight) || isscalar(weight)
      weightdata = ones(nfreq,1);      
   elseif isvector(weight) || length(weight)==nfreq
      weightdata = reshape(abs(weight),[nfreq 1]);       
   else
      error(message('Robust:fitting:FITMAGFRD2'))
   end
elseif isa(weight,'DynamicSystem')
   % Enforce matching units
   if ~isequal(size(weight),[1 1])
      error(message('Robust:fitting:FITMAGFRD3'))
   elseif ~strcmp(weight.TimeUnit,data.TimeUnit)
      error(message('Robust:fitting:IncompatibleTimeUnits','fitmagfrd'))
   elseif isa(weight,'FRDModel') && ~isequal(weight.Frequency,Freq)
      error(message('Robust:fitting:FITFRD3'))
   end
   % Compute mag data for weight
   weightdata = abs(freqresp(weight,romega,'rad/TimeUnit'));
   weightdata = reshape(weightdata,[nfreq 1]);
else
   error(message('Robust:fitting:FITFRD5','fitmagfrd'))
end   
% at this point, weightdata is NFREQ x 1 double   

% XXX PJS 29Mar09: Update Constraint Handling
constrflg = true;
lowerbound = -inf([nfreq,Ntf]);
upperbound = inf([nfreq,Ntf]);
if isempty(constraint)
   constrflg = false;
elseif isstruct(constraint) % XXX
   try
      if ~isempty(constraint.LowerBound)
         lowerbound = LOCALCheckConstraint(constraint.LowerBound,romega,Ts,nfreq);
         lowerbound = lowerbound*SclFac;
      end
      if ~isempty(constraint.UpperBound)
         upperbound = LOCALCheckConstraint(constraint.UpperBound,romega,Ts,nfreq);
         upperbound = upperbound*SclFac;
      end
   catch ME
      throw(ME)
   end
else
   try
      Cvec = LOCALCheckConstraint(constraint,romega,Ts,nfreq);
   catch ME
      throw(ME)
   end
   Cvec = sign(real(Cvec));
   idx = find( Cvec==1);
   % No need to scale bounds here, as RESPDATA was already scaled
   lowerbound(idx) = respdata(idx);
   idx = find( Cvec==-1);
   upperbound(idx) = respdata(idx);
end

% Error checking on opts
if ~isempty(opts) && (length(opts)>=2)
	if opts(1)>0
     tmax = opts(1);
	end
	if opts(2)>0
     ttol = opts(2);
	end
else
	ttol = 0.01;   
   % GJB: Pete change based on errors 2Nov04
	% tmax= (1+ttol)*max(weightdata)*( ( max(respdata)/min(respdata) )^2-1 );
   if constrflg
      tmax= (1+ttol)*max(weightdata)*( ( max(respdata)/min(respdata) )^2-1 );
   else
      tmax= (1+ttol)*max(weightdata)*( max(respdata)/min(respdata) - 1);
   end
end

% XXX Check reldeg>=0 && reldeg <= ord?
if isempty(reldeg)
   reldeg = zeros(1,Ntf);
elseif length(reldeg)==1
   reldeg = reldeg*ones(1,Ntf);
end

if length(reldeg)==Ntf && all(reldeg>=0) && all(reldeg<=ord)
   reldeg = reldeg(:)';
else
   ctrlMsgUtils.error('Robust:fitting:FITMAGFRDConstraint1');
end
   
zz = exp(1i*domeorig);
if Ts==0
	tmp = repmat(zz+1,[1 Ntf]).^repmat(reldeg,[nfreq,1]);
else
	tmp = repmat(zz-1/b,[1 Ntf]).^repmat(reldeg,[nfreq,1]);
end
% XXXX PJS 4/29/2009 -- tmp will be complex if reldeg~=0. 
%  Can we just take abs of it?
tmp = abs(tmp);
weightdata = weightdata.*tmp;
respdata = respdata./tmp;
lowerbound = lowerbound./tmp;
upperbound = upperbound./tmp;

[num,den] = fitmagfrdeng(respdata,domeorig,ord,ord-reldeg(:),...
   weightdata,lowerbound,upperbound,ttol,tmax);
num = [zeros(1,numel(den)-numel(num)) num];  % equalize lengths of NUM,DEN
[Amat,Bmat,Cmat,Dmat] = compreal(num,den);
pssmat = [Amat Bmat;Cmat Dmat];
sys = lft(kron(L,eye(ord)),pssmat,ord,ord);
Amat = sys(1:ord,1:ord);
Bmat = sys(1:ord,ord+1:end);
Cmat = sys(ord+1:end,1:ord);
Dmat = sys(ord+1:end,ord+1:end);
if Ts==0
   for i = 1:Ntf
      Cmat(i,:) = Cmat(i,:)*(-2*alpha)^reldeg(i);
      Dmat(i,:) = Dmat(i,:)*(-2*alpha)^reldeg(i);
      [Cmat(i,:),Dmat(i,:)] = ...
         sisormzero(Amat,Bmat,Cmat(i,:),Dmat(i,:),alpha,reldeg(i));
   end
else
   if b~=0
      gam = b^2/(b^2-1);
      for i = 1:Ntf
         Cmat(i,:) = Cmat(i,:)/(gam)^reldeg(i);
         Dmat(i,:) = Dmat(i,:)/(gam)^reldeg(i);
         [Cmat(i,:),Dmat(i,:)] = sisormzero(Amat,Bmat,Cmat(i,:),Dmat(i,:),-1/b,reldeg(i));
      end
   end
end
% Undo the scaling
sys = ss(Amat,sqrt(SclFac)\Bmat,sqrt(SclFac)\Cmat,SclFac\Dmat,...
   Ts,'TimeUnit',data.TimeUnit);

% -----------------------------------------------------------
% LOCALCheckConstraint
% Check constraint and then output as a double
function Cout = LOCALCheckConstraint(constraint,omega,Ts,nfreq)
Ntf = 1;

szc = size(constraint);
if isa(constraint,'frd')
   % Data
   [cResponseData,cFrequency,cTs] = frdata(constraint);
   cromega = cFrequency * funitconv(constraint.FrequencyUnit,'rad/TimeUnit',constraint.TimeUnit);
   if ~FRDModel.isSameFrequencyGrid(cromega,omega)
      ctrlMsgUtils.error('Robust:fitting:FITMAGFRDConstraint2');
   elseif cTs~=-1 && cTs~= Ts
      ctrlMsgUtils.error('Robust:fitting:FITMAGFRDConstraint3');
   elseif szc(1)~=1 && szc(2)~=1
      ctrlMsgUtils.error('Robust:fitting:FITMAGFRDConstraint4');
   elseif szc(1)==1 && szc(2)==Ntf
      Cout = permute(cResponseData,[3 2 1]);
   elseif szc(1)==Ntf && szc(2)==1
      Cout = permute(cResponseData,[3 1 2]);
   elseif (szc(1)==1 && szc(2)==1)
      Cout = repmat(cResponseData(:),[1 Ntf]);
   else
      ctrlMsgUtils.error('Robust:fitting:FITMAGFRDIncompDim')
   end
elseif isa(constraint,'double')
   if szc(1)==1 && szc(2)==1
      Cout = repmat(constraint,[nfreq,Ntf]);
   elseif szc(1)~=1 && szc(2)~=1
      ctrlMsgUtils.error('Robust:fitting:FITMAGFRDConstraint4')
   elseif max(szc) == Ntf
      % Cout = constraint(:)';
      Cout = repmat(abs(constraint),nfreq,1);
   elseif max(szc) ~=nfreq
      ctrlMsgUtils.error('Robust:fitting:FITMAGFRDIncompDim')
   end
else
   ctrlMsgUtils.error('Robust:fitting:FITMAGFRDConstraint5');
end

Cout = Cout(:);
