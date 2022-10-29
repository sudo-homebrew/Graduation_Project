function resp = genphase(d)
%GENPHASE  Generate phase of stable, minimum-phase system from the magnitude
%          of its frequency response.
%
%   COMPLEXRESP = GENPHASE(MAGRESP) uses the complex-cepstrum
%   (Oppenheim & Schafer, Digital Signal Processing, pg 501) to
%   generate a complex frequency response, COMPLEXRESP, whose
%   magnitude is equal to the real, positive response MAGRESP,
%   but whose phase corresponds to a stable, minimum phase
%   function.  Both MAGRESP and COMPLEXRESP are FRD.
%
%   See also FITFRD, FITMAGFRD.

%   Copyright 2003-2015 The MathWorks, Inc.
if nargin == 0
   %disp(['usage: complexresp = genphase(magresp)']);
   ctrlMsgUtils.error('Robust:fitting:GENPHASEin');
elseif ~isequal(size(d),[1 1])
   ctrlMsgUtils.error('Robust:fitting:GENPHASEfrd');
end

% Get data
[ResponseData,Frequency,Ts] = frdata(d);
romega = Frequency * funitconv(d.FrequencyUnit,'rad/TimeUnit',d.TimeUnit);
magorig = ResponseData(:).';  % row vector
if Ts==0
   w = romega(romega>0 & romega<Inf);
   if numel(w)<2,
      error(message('Robust:fitting:InvalidContinuousFrequencies'))
   end
   [~,~,domeorig] = hp2dmax(w(1),w(end),romega);
else
   thetavec = romega*abs(Ts);
   th = thetavec(thetavec>0 & thetavec<pi);
   if any(thetavec>(1+100*eps)*pi) || numel(th)<2
      error(message('Robust:fitting:InvalidDiscreteFrequencies'))
   end
   [~,~,domeorig] = d2dmax(th(1),th(end),thetavec);
end
% Select number of point for phase reconstruction
hnpts = pow2(max(12,nextpow2(numel(domeorig))));  % 2^12=4096 points minimum
compd = enggenphase(domeorig,magorig,hnpts,2*hnpts);
resp = frd(compd,Frequency,d);

%---------------------------------------------------------------
function compd = enggenphase(domeorig,magorig,hnpts,npts)
% interpolate the frequency and magnitude data to
% HNPTS points linearly spaced around top half of disk
[lindome,linmag] = LOCALterpol(domeorig,magorig,hnpts);
% duplicate data around disk
dome = [lindome (2*pi)-fliplr(lindome)];          %all disk
mag = [linmag  fliplr(linmag)];                   %all disk
% complex cepstrum to get min-phase
ymag = log( mag .^2 );
ycc = ifft(ymag);                              % 2^N
nptso2 = npts/2;                               % 2^(N-1)
xcc = ycc(1:nptso2);                           % 2^(N-1)
xcc(1) = xcc(1)/2;                             % halve it at 0
xhat = exp(fft(xcc));                          % 2^(N-1)
domeg = dome(1:2:nptso2-1);                    % 2^(N-2)
xhat = xhat(1:nptso2/2);                       % 2^(N-2)
% interpolate back to original frequency data
[compd] = LOCALterpolb(domeg,xhat,domeorig);


function [freq,mag] = LOCALterpol(freqin,magin,npts)
%   This function interpolates the irregularly spaced
%   frequency/magnitude data FREQIN and MAGIN,
%   in the following way.  First, it assumes that
%   FREQIN is a row vector of nonnegative frequencies,
%   between 0 and pi:
%   foreach 1 <= i <= NPTS
%      if      pi*(i-1)/NPTS  <  FREQIN(1),
%        then    FREQ(i)  :=  MAG(1)
%      if      FREQIN(length(FREQIN))  <=  pi*(i-1)/NPTS
%        then    MAGOUT((i) =  MAGIN(length(MAGIN))
%      for other values of FREQIN(i), the program does
%      linear interpolation of LOG(MAG) between data points
lmagin = log(magin);
freq = (pi/npts)*(0:npts-1);
lmag = zeros(1,npts);
topval = length(freqin);

plow = freq<freqin(1);
lmag(plow) = lmagin(1);

phigh = freqin(topval)<=freq;
lmag(phigh) = lmagin(topval);

% The loop is much faster than the INTERP1, for linear interp
% lmag(mstart:mstop) = interp1(freqin,lmagin,freq(mstart:mstop));
for i=2:topval
   p = ceil(npts*freqin(i-1)/pi+1):(ceil(npts*freqin(i)/pi+1)-1);
   rat = freq(p) - freqin(i-1)*ones(1,length(p));
   rat = (1/(freqin(i)-freqin(i-1)))*rat;
   lmag(p)=(ones(1,length(p))-rat)*lmagin(i-1)+rat*lmagin(i);
end
mag = exp(lmag);


function [mag] = LOCALterpolb(fastfre,fastmag,slowfre)
%   Interpolates back after complex cepstrum calculation,
%   from the interpolated, regularly spaced data, to the original
%   frequency response data.
nptsslo = length(slowfre);
mag = zeros(1,nptsslo);
nptsfst = length(fastfre);
strt = 1;
stop = nptsslo;
lowstuff = find(slowfre<=fastfre(1));
if ~isempty(lowstuff)
   mag(lowstuff) = fastmag(1);
   strt = length(lowstuff) + 1;
end
highstuff = find(slowfre>=fastfre(nptsfst));
if ~isempty(highstuff)
   mag(highstuff) = fastmag(nptsfst);
   stop = nptsslo - length(highstuff);
end
mag(strt:stop) = interp1(fastfre,fastmag,slowfre(strt:stop));
