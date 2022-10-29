function [sys,fail] = fitfrd(frdsys,ord,reldeg,weight,code,idnum)
%FITFRD   Fit frequency response data with state-space model.
%   B = FITFRD(A,N) is a state-space object, with state-dimension N,
%   whose frequency response closely matches the frequency response
%   data in A.  A is an FRD object, and N is a nonnegative integer.
%
%   A must have either 1 row or 1 column, though it need not
%   be 1-by-1.  B will be the same size as A.  In all cases, N should
%   be a nonnegative scalar.
%
%   B = FITFRD(A,N,RD) forces the relative degree of B to be RD.  RD
%   should be a nonnegative integer.  The default value for RD is 0.
%   If A is a row (or column) then RD can be a vector of the same size
%   as well, specifying the relative degree of each entry of B.  If
%   RD is a scalar, then it specifies the relative degree for all entries
%   of B.  The default for RD can also be specified by setting RD = [].
%
%   B = FITFRD(A,N,RD,WT) uses the magnitude of WT to weight the
%   optimization fit criteria.  WT may be a DOUBLE, SS or FRD.  If
%   WT is a scalar, then it is used to weight all entries of the
%   error criteria (A-B).  If WT is a vector, it should be the same
%   size as A, and each individual entry of WT acts as a weighting
%   function on the corresponding entry of (A-B).
%
%   See also INVFREQS, INVFREQZ.

%   Copyright 2003-2015 The MathWorks, Inc.

% old help
%   CODE   : 0 - no restriction on pole location of SYS (default)
%            1 - restrict SYS to be stable, minimum-phase, requires
%                   FRDATA to be 1x1 VARYING matrix
%            2 - constrain rational fit so that SYS is stable
%   IDNUM  : number of LS iterations (default = 6)
ni = nargin;
if ni < 2
   ctrlMsgUtils.error('Robust:fitting:OneInputArgument');
end
if ni<3
   reldeg = [];
end
if ni<4
   weight = [];
end
if ni<5 || ~(isequal(code,1) || isequal(code,2))
   % CODE should be 0,1,2
   code = 0;
end
if ni<6 || ~(isa(idnum,'double') && isscalar(idnum) && rem(idnum,1)==0 && idnum>=0)
   % IDNUM should be a nonnegative integer
   idnum = 6;
end
fail = 0;

szf = size(frdsys);
if ~isa(frdsys,'frd') || length(szf)>2 || min(szf)>1
   ctrlMsgUtils.error('Robust:fitting:FITFRD1');
end
flag = 0;  % input is a ROW
Ntf = szf(2);
if (szf(1)~=1)
   frdsys = frdsys.';
   flag = 1; % input is a COLUMN
   Ntf = szf(1);
end

% Extract data
Ts = frdsys.Ts;
romega = frdsys.Frequency * funitconv(frdsys.FrequencyUnit,'rad/TimeUnit',frdsys.TimeUnit);
respdata = freqresp(frdsys,romega);
nfreq = numel(romega);
if Ts==0
   w = romega(romega>0 & romega<Inf);
   if numel(w)<2,
      error(message('Robust:fitting:InvalidContinuousFrequencies'))
   end
   [alpha,~,domeorig,L,~,~] = hp2dmax(w(1),w(end),romega);
else
   thetavec = romega*abs(Ts);
   th = thetavec(thetavec>0 & thetavec<pi);
   if any(thetavec>(1+100*eps)*pi) || numel(th)<2
      error(message('Robust:fitting:InvalidDiscreteFrequencies'))
   end
   [b,~,domeorig,L,~,~] = d2dmax(th(1),th(end),thetavec);
end

% Get WEIGHT as NFREQ x NTF double array
if isnumeric(weight)
   if isempty(weight) || isscalar(weight)
      weightdata = ones(nfreq,Ntf);
   elseif isvector(weight) && numel(weight)==Ntf
      weightdata = repmat(abs(reshape(double(weight),[1 Ntf])),[nfreq,1]);
   else
      ctrlMsgUtils.error('Robust:fitting:FITFRD2');
   end
elseif isa(weight,'DynamicSystem')
   % Enforce matching units
   if ~strcmp(weight.TimeUnit,frdsys.TimeUnit)
      ctrlMsgUtils.error('Robust:fitting:IncompatibleTimeUnits','fitfrd')
   end
   % Convert to FRD with grid ROMEGA
   try
      weight = frd(weight,romega);
   catch ME %#ok<NASGU>
      ctrlMsgUtils.error('Robust:fitting:FITFRD3');
   end
   weightdata = abs(weight.ResponseData);
   szw = size(weight);
   if isequal(szw,[1 1])
      weightdata = repmat(weightdata(:),[1 Ntf]);
   elseif isequal(szw,[1 Ntf]) || isequal(szw,[Ntf 1])
      weightdata = reshape(permute(weightdata,[3 1 2]),[nfreq Ntf]);
   else
      ctrlMsgUtils.error('Robust:fitting:FITFRD4');
   end
else
   ctrlMsgUtils.error('Robust:fitting:FITFRD5','fitfrd');
end   
% at this point, weightdata is NFREQ x NTF double

if isempty(reldeg)
   reldeg = zeros(1,Ntf);
elseif length(reldeg)==1
   reldeg = reldeg*ones(1,Ntf);
end
if length(reldeg)==Ntf && all(reldeg>=0) && all(reldeg<=ord)
   reldeg = reldeg(:)';
else
   ctrlMsgUtils.error('Robust:fitting:InvalidRelativeDegree');
end

respdata = permute(respdata,[3 2 1]);
if ord>0
   zz = exp(1i*domeorig);
   if Ts==0
      tmp = repmat(zz+1,[1 Ntf]).^repmat(reldeg,[nfreq,1]);
   else
      tmp = repmat(zz-1/b,[1 Ntf]).^repmat(reldeg,[nfreq,1]);
   end
   weightdata = weightdata.*tmp;
   respdata = respdata./tmp;
   
   [Ncell,den] = fitsyseng(respdata,domeorig,ord,ord-reldeg(:),weightdata,idnum);
   
   %  This is the hack-CODE for enforcing stable -- reflect the poles, and
   %  then holding the denominator fixed, refit the numerator with 1
   %  iteration.
   if code==2
      rs = roots(den);
      uloc = find(abs(rs)>1);
      if ~isempty(uloc)
         rs(uloc) = rs(uloc)./(abs(rs(uloc)).^2);
         sden = real(poly(rs).');
         [Ncell,~] = fitsyseng(respdata,domeorig,ord,ord-reldeg(:),weightdata,1,sden);
         den = sden;
      end
   end
   for i=1:Ntf
      Ncell{i} = Ncell{i}';  % make the individual N's row vectors
   end
   den = den(:).'; % make D a row vector
   
   % GJB 15Oct04: Adding MINREAL to remove P/Z cancellations
   sysC = minreal(ss(tf(Ncell,den,-1)));
   [Amat,Bmat,Cmat,Dmat] = ssdata(sysC);
   ord = size(Amat,1);
   pssmat = [Amat Bmat;Cmat Dmat];
   try
      sys = lft(kron(L,eye(ord)),pssmat,ord,ord);
   catch %#ok<*CTCH>
      fail = 1;
      [Ncell,den] = fitsyseng(respdata,domeorig,ord-1,ord-1-reldeg(:),weightdata,idnum);
      for i=1:Ntf
         Ncell{i} = Ncell{i}';  % make the individual N's row vectors
      end
      den = den(:).'; % make D a row vector
      sysC = ss(tf(Ncell,den,-1)); % SYSS will DIM-by-1, regardless
      sysC = minreal(sysC);
      [Amat,Bmat,Cmat,Dmat] = ssdata(sysC);
      ord = size(Amat,1);
      pssmat = [Amat Bmat;Cmat Dmat];
      try
         sys = lft(kron(L,eye(ord)),pssmat,ord,ord);
      catch
         ctrlMsgUtils.error('Robust:fitting:NumericalErrorDisk2Disk');
      end
   end
   Amat = sys(1:ord,1:ord);
   Bmat = sys(1:ord,ord+1:end);
   Cmat = sys(ord+1:end,1:ord);
   Dmat = sys(ord+1:end,ord+1:end);
   if Ts==0
      for i = 1:Ntf
         Cmat(i,:) = Cmat(i,:)*(-2*alpha)^reldeg(i);
         Dmat(i,:) = Dmat(i,:)*(-2*alpha)^reldeg(i);
         [Cmat(i,:),Dmat(i,:)] = sisormzero(Amat,Bmat,Cmat(i,:),Dmat(i,:),alpha,reldeg(i));
      end
      sys = ss(Amat,Bmat,Cmat,Dmat);
   else
      if b~=0
         gam = b^2/(b^2-1);
         for i = 1:Ntf
            Cmat(i,:) = Cmat(i,:)/(gam)^reldeg(i);
            Dmat(i,:) = Dmat(i,:)/(gam)^reldeg(i);
            [Cmat(i,:),Dmat(i,:)] = sisormzero(Amat,Bmat,Cmat(i,:),Dmat(i,:),-1/b,reldeg(i));
         end
      end
      sys = ss(Amat,Bmat,Cmat,Dmat,Ts);
   end
else
   sys = zeros(1,Ntf);
   for kk=1:Ntf
      amat = [weightdata(:,kk);zeros(nfreq,1)];
      bmat = [weightdata(:,kk).*real(respdata(1:nfreq,kk)); ...
         weightdata(:,kk).*imag(respdata(1:nfreq,kk))];
      sys(kk) = amat\bmat;
   end
   sys = ss([],[],[],sys,Ts);
   if flag==1
      sys = sys.';
   end
end

if code == 1 && Ntf == 1
   sys = spectralfact(sys,[]);
end

if flag==0
   sys = sys.';
end
sys.TimeUnit = frdsys.TimeUnit;
% sys = strans(sys);
% if code == 1 &  Ntf == 1
%    if Ts==0
%       sys = extsmp(sys);
%    else
%       sys = co2di(extsmp(di2co(sys,1)),1);
%    end
% end
