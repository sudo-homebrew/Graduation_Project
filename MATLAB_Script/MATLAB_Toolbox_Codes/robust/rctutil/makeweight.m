function W = makeweight(DC,MID,HF,Ts,N)
%MAKEWEIGHT  Construct weighting function with monotonic gain profile.
%
%   MAKEWEIGHT helps create frequency-weighting functions for robust
%   controller synthesis (see H2SYN, HINFSYN, MUSYN). Such functions
%   are useful to capture frequency-dependent requirements, for example,
%   low sensitivity at low frequency, or minimum roll-off at high
%   frequency.
%
%   W = MAKEWEIGHT(DC,[FREQ MAG],HF) creates a first-order, continuous-time
%   weight W(s) satisfying the constraints:
%      * W(0) = DC
%      * W(inf) = HF
%      * |W(j*FREQ)| = MAG (W has gain MAG at the frequency 0 < FREQ < Inf).
%   The values DC, MAG, HF must satisfy |DC| > MAG > |HF| (low-pass weight)
%   or |DC| < MAG < |HF| (high-pass weight).
%
%   W = MAKEWEIGHT(DC,[FREQ MAG],HF,Ts) creates a first-order, discrete-time
%   weight W(z) satisfying the constraints:
%      * W(1) = DC
%      * W(-1) = HF
%      * |W(exp(j*FREQ*Ts))| = MAG (W has gain MAG at 0 < FREQ < pi/Ts).
%
%   W = MAKEWEIGHT(DC,[FREQ MAG],HF,Ts,N) uses an N-th order transfer
%   function with poles and zeros in a Butterworth pattern to meet the
%   constraints above. The higher the order N, the steeper the transition
%   from low to high gain.
%
%   W = MAKEWEIGHT(DC,WCROSS,HF,...) specifies the gain crossover frequency
%   WCROSS and is equivalent to setting FREQ=WCROSS and MAG=1.
%
%   See also MKFILTER, AUGW, H2SYN, HINFSYN, MUSYN.

%   Copyright 1986-2020 The MathWorks, Inc.
narginchk(3,5)
nin = nargin;
if nin<4
   Ts = 0;
elseif ~(isnumeric(Ts) && isscalar(Ts) && isreal(Ts) && Ts>=0 && Ts<Inf)
   error(message('Robust:design:makeweight1'))
end
if nin<5
   N = 1;
elseif ~(isnumeric(N) && isscalar(N) && isreal(N) && N>0 && N<Inf && rem(N,1)==0)
   error(message('Robust:design:makeweight2'))
end

% Validate first 3 inputs
if ~(isnumeric(DC) && isscalar(DC) && isreal(DC) && isfinite(DC))
   error(message('Robust:design:makeweight3'))
end
if ~(isnumeric(HF) && isscalar(HF) && isreal(HF) && isfinite(HF))
   error(message('Robust:design:makeweight4'))
end
if ~(isnumeric(MID) && any(numel(MID)==[1 2]) && isreal(MID))
   error(message('Robust:design:makeweight5'))
end
if isscalar(MID)
   freq = MID;   mag = 1;
else
   freq = MID(1);  mag = MID(2);
end
if ~(freq>0 && freq<pi/Ts)
   if Ts==0
      error(message('Robust:design:makeweight6'))
   else
      error(message('Robust:design:makeweight7'))
   end
end
if (mag-abs(DC))*(mag-abs(HF))>=0
   error(message('Robust:design:makeweight8'))
elseif rem(N,2)==0 && HF*DC<0
   error(message('Robust:design:makeweight9'))
end

if N==1
   p = freq * sqrt(((HF/mag)^2-1)/(1-(DC/mag)^2)); % pole
   if Ts==0
      W = tf([HF DC*p],[1 p]);
   else
      % Apply bilinear transform to continuous-time solution 
      bt = freq*sin(freq*Ts)/(cos(freq*Ts)-1);
      W = tf([DC*p-HF*bt DC*p+HF*bt],[p-bt p+bt],Ts);
   end
   W = ss(W);
else
   % Use zero crossing to determine suitable natural frequency wn for the poles
   if abs(DC)>mag
      % low-pass
      wLB = freq * 0.5*(mag/abs(DC))^(1/N);
      wUB = freq * min(10,2*(mag/abs(HF))^(1/N));
   else
      % high pass
      wLB = freq * max(0.1,0.5*(abs(DC)/mag)^(1/N));
      wUB = freq * 2*(abs(HF)/mag)^(1/N);
   end
   % Note: Do not clip wUB to pi/Ts (may be infeasible for small N).
   wn = fzero(@(wn) log(LOCALevalGain(freq,DC,HF,N,wn)/mag),[wLB wUB]);
   %[wLB wn wUB]
   [z,p,k] = LOCALbutterBuild(DC,HF,N,wn);
   W = ss(zpk(z,p,k));
   if Ts>0
      % Note: Tustin with prewarping delivers gains of DC,MAG,HF at the 
      %       frequencies w=0,FREQ,pi/Ts (recall that hd(+1)=h(0) and 
      %       hd(-1)=h(inf)).
      opt = c2dOptions('Method','tustin','PrewarpFrequency',freq);
      W = c2d(W,Ts,opt);
   end
end
   
%--------------------------------------------------------------------------------  

function mag = LOCALevalGain(freq,DC,HF,N,wn)
[z,p,k] = LOCALbutterBuild(DC,HF,N,wn);
s = 1i*freq;
mag = abs(k*prod(s-z)/prod(s-p));

function [z,p,k] = LOCALbutterBuild(DC,HF,N,wn)
% Build transfer function B(s) with 
%   * zeros and poles in Butterworth pattern
%   * wn = natural frequency of poles
%   * B(0) = DC and B(inf) = HF (both nonzero)
if rem(N,2)==0
   r = -exp(1i*pi*(1:2:N-1)/(2*N));
   r = [r,conj(r)];
else
   r = -exp(1i*pi*(2:2:N-1)/(2*N));
   r = [r,-1,conj(r)];
end
p = wn*r;
if HF==0
   z = [];
   k = DC*wn^N;
elseif DC==0
   z = zeros(N,1);
   k = HF;
else
   z = (wn*abs(DC/HF)^(1/N))*r;
   if HF*DC<0
      z(imag(z)==0) = -z(imag(z)==0);  % only works for N odd
   end
   k = HF;
end

% function A = LOCALbwpoly(w,n)
% % P = BWPOLY(W,N) creates N'th order polynomial P (as 1-by(n+1) row vector)
% % with roots in a Butterworth pattern, in the left-half plane, at magnitude
% % equal to W.  The polynomial is normalized with P(END) = 1.
% nd2 = n/2;
% if floor(nd2)==ceil(nd2)
%    thetavec = pi/2/n + (0:nd2-1)*pi/n;
%    C = 1;
% else
%    thetavec = pi/2/n + (0:(n-1)/2-1)*pi/n;
%    C = [1 1];
% end
% for i=1:length(thetavec)
%    xi = sin(thetavec(i));
%    C = conv(C,[1 2*xi 1]);
% end
% % Code below works well even if W depends on REALP (or UREAL).  This is a
% % strategy in all MUSYN loopshaping stages.
% A = C(1);
% for i=2:n+1
%    A = [(1/w)*A C(i)];
% end
% % [C(1)/w^n C(2)/w^(n-1)       C(n)/w C(n+1)]