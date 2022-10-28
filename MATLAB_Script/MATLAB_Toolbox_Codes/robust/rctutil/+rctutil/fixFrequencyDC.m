function [UB,LB] = fixFrequencyDC(UB,LB,Ts)
% Converts frequencies from continuous to discrete, using Tustin
% transformation and supplied sample time.  Frequencies are stored as
% 1-by-2 interval in UB.Interval, and scalar in LB.w.  Called by
% ssmussvPeak and ssmussvCurve

%   Copyright 1986-2020 The MathWorks, Inc.
Ts = abs(Ts);
for i=1:numel(UB)
   Ic = UB(i).Interval;
   Id = (2/Ts)*atan(Ic*Ts/2);  % angle((1+1j*Ic*Ts/2)./(1-1j*Ic*Ts/2));
   UB(i).Interval = Id;
   wc = UB(i).w;
   wd = (2/Ts)*atan(wc*Ts/2);  % angle((1+1j*wc*Ts/2)./(1-1j*wc*Ts/2));
   UB(i).w = wd;
end
for i=1:numel(LB)
   wc = LB(i).w;
   wd = (2/Ts)*atan(wc*Ts/2);  % angle((1+1j*wc*Ts/2)./(1-1j*wc*Ts/2));
   LB(i).w = wd;
end


