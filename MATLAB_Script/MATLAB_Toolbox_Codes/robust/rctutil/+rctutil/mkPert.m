function [A,B,C,D] = mkPert(lvec,rvec,freq,Ts)
% MKMIMOPERTN
%   [A,B,C,D] = MKPERT(L,R,OMEGA,Ts) creates a state-space system SYS 
%   with the following properties
%
%     SYS(OMEGA) = L*R
%     NORM(SYSN,'inf') = NORM(L)*NORM(R)
%     SYSN(0) = SYS(HIGHFREQUENCY) = 0
%
%   OMEGA is in rad/TimeUnit and the returned (A,B,C,D) are implicitly in
%   the same TimeUnit.

%   Copyright 2004-2019 The MathWorks, Inc. and PA

% If OMEGA=0, and L*R is real, then we could...
%   CT: L*tf(1,[1 1])*R
%   DT: L*tf(1,[1 0],Ts)*R
%   DT: L*tf([1 1],[2 0],Ts)*R
lvec = lvec(:);  % column
rvec = rvec(:).';% row
nl = numel(lvec);
nr = numel(rvec);
Ts = abs(Ts);
tol = eps^(3/4);

[mxVal,idx] = max(abs([lvec;rvec']));
if mxVal==0
   A = [];
   B = zeros(0,nr);
   C = zeros(nl,0);
   D = zeros(nl,nr);
else
   if idx<=nl
      nVal = lvec(idx);
      lvec = lvec/nVal;
      lvec(idx) = 1;
      rvec = rvec*nVal;
   else
      nVal = rvec(idx-nl);
      lvec = lvec*nVal;
      rvec = rvec/nVal;
      rvec(idx-nl) = 1;
   end
   [LA,LB,LC,LD] = localMIMOrat(freq,lvec,Ts,tol);
   [RA,RB,RC,RD] = localMIMOrat(freq,rvec,Ts,tol);
   [SA,SB,SC,SD] = localFit1(freq,Ts,tol);
   [A,B,C,D] = ltipack.ssops('mult',LA,LB,LC,LD,[],SA,SB,SC,SD,[]);
   [A,B,C,D] = ltipack.ssops('mult',A,B,C,D,[],RA,RB,RC,RD,[]);
end

%---------------- Local functions -----------------------------------

function [A,B,C,D] = localFit1(w,Ts,tol)
% [A,B,C,D] = localFit1(W) creates system data whose frequency response
% is exactly 1 at the given positive frequency, W.  The frequency
% response is 0 at low and high frequency, and the peak magnitude is 1.

% CT: Use tf([2*xi*w 0],[1 2*xi*w w^2]) with xi=1/sqrt(2)
% DT: Use Tustin discretization with prewarping at w
xi = 1/sqrt(2);
if Ts==0
   % Construct continuous-time solution
   alpha = xi*w;
   if alpha<tol || alpha>1/tol
      % Return pure gain for w=0 or w=Inf (transfer function becomes unstable)
      A = []; B = zeros(0,1); C = zeros(1,0); D = 1;
   else
      tau = sqrt(2*alpha);
      A = alpha * [-1 1;-1 -1];
      B = tau * [1;1];
      C = [0 tau];
      D = 0;
   end
else
   % Hand calculation of Tustin discretization with prewarping at w
   alpha = xi*tan(w*Ts/2);
   if alpha<tol || alpha>1/tol
      % Return pure gain for w=0 and w=pi/Ts
      A = []; B = zeros(0,1); C = zeros(1,0); D = 1;
   else
      aux1 = 2*alpha;
      aux2 = alpha * aux1;
      aux3 = (1+alpha)^2+alpha^2;
      tau = 2*sqrt(alpha)/aux3;
      A = [1-aux2 aux1;-aux1 1-aux2]/aux3;
      B = tau * [1+aux1 ; 1];
      C = tau * [-alpha 1+alpha];
      D = aux1/aux3;
   end
end


function [A,B,C,D] = localMIMOrat(frequency,gainvec,ts,tol)
szg = size(gainvec); 
if szg(2)>1
   A = []; B = []; C = zeros(1,0); D = zeros(1,szg(2));
   for i=1:length(gainvec)
      [a,b,c,d] = localSISOrat(frequency,gainvec(i),ts,tol);
      A = blkdiag(A,a);
      B = blkdiag(B,b);
      C = [C c]; %#ok<*AGROW>
      D(i) = d;
   end
else
   A = []; B = zeros(0,1); C = []; D = zeros(szg(1),1);
   for i=1:length(gainvec)
      [a,b,c,d] = localSISOrat(frequency,gainvec(i),ts,tol);
      A = blkdiag(A,a);
      B = [B;b];
      C = blkdiag(C,c);
      D(i) = d;
   end
end


function [a,b,c,d] = localSISOrat(w,gain,Ts,tol)
wTs = w*Ts;
if isreal(gain) || (Ts==0 && (w<tol || w>1/tol)) || ...
      (Ts>0 && (wTs<tol || wTs>pi-tol))
   % Use pure gain to avoid marginally stable or near infinite poles
   a = [];  b = zeros(0,1);  c = zeros(1,0);  d = real(gain);
else
   % Use first- or second-order interpolant whose pole has natural
   % frequency close to w.
   rp = real(gain);
   ip = imag(gain);
   gamma = abs(gain);
   theta = angle(gain);
   sgn = 1;
   % First-order allpass interpolant have a pole with natural frequency
   % close to w when angle(gain) is close to pi/2. Reduce all cases to
   % interpolating a value with angle in [pi/4,3*pi/4]
   if abs(ip)>=abs(rp)
      % First-order h(s) or -h(s)
      pow = 1;
      if ip<0
         sgn = -1;   theta = theta+pi;
      end
   else
      % Use second-order h(s)^2 or -h(s)^2
      pow = 2;
      if rp>0
         sgn = -1;   theta = theta+pi;
      elseif ip<0
         theta = theta+2*pi;
      end
      gamma = sqrt(gamma);
      theta = theta/2;  % in [pi/4,3*pi/4]
   end
   % [pi/4 theta 3*pi/4]
   
   % Interpolate c=gamma*exp(i*theta)
   if Ts==0
      % h(s) = gamma * (s-alpha)/(s+alpha) with alpha = w*tan(theta/2)
      alpha = w*tan(theta/2);  % > 0
      tau = 2*alpha*gamma;
      a = -alpha;
      b = sqrt(tau);
      c = -b;
      d = gamma;
   else
      % h(z) = gamma * (alpha*z-1)/(z-alpha) with
      % alpha = cos((theta+w*Ts)/2)/cos((theta-w*Ts)/2)
      alpha = cos((theta+w*Ts)/2)/cos((theta-w*Ts)/2);  % |alpha|<1
      a = alpha;
      tau = gamma*(1-alpha^2);
      b = sqrt(tau);
      c = -b;
      d = gamma*a;
   end
   
   % Finalize realization
   if pow>1
      [a,b,c,d] = ltipack.ssops('mult',a,b,c,d,[],a,b,c,d,[]);
   end
   if sgn<0
      d = -d;  c = -c;
   end
end