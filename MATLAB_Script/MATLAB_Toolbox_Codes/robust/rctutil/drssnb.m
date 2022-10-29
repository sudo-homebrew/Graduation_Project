function sys = drssnb(bandwidth,Ts,n,varargin)
%DRSSNB  Generate random stable discrete-time state-space models
%with H_inf norm = 1.
%
%   SYS = DRSSNB(BW,TS,N) generates an Nth-order SISO state-space model SYS.
%
%   SYS = DRSSNB(BW,TS,N,P) generates a single-input Nth-order model with 
%   P outputs.
%
%   SYS = DRSSNB(BW,TS,N,P,M) generates an Nth-order model with P outputs
%   and M inputs.
%
%   SYS = DRSSNB(BW,TS,N,P,M,S1,...,Sk) generates a S1-by-...-by-Sk array of
%   state-space models with N states, P outputs, and M inputs.
%
%   See also RSS, DRSS, RSSNB.

%   Copyright 2003-2006 The MathWorks, Inc. 

if Ts==0 || (Ts<0 && Ts~=-1)
   error('Sample Time should be -1 or a positive number.');
end
if isempty(bandwidth)
   bandwidth = 0.98/2/abs(Ts);
else
   if bandwidth<=0
      error('Maximum bandwidth must be greater than 0.')
   end
end
%how should Ts=-1 be handled?
if Ts ~=-1
   if bandwidth>1/(2*Ts)
      bandwidth = 1/2/Ts;
   end
   sys = rssnb(bandwidth,n,varargin{:});
   sys = c2d(sys,Ts,'tustin');
else
   if bandwidth>1/2
      bandwidth = 1/2;
   end
   sys = rssnb(bandwidth,n,varargin{:});
   sys = c2d(sys,1,'tustin');
   sys.Ts = -1;
end
