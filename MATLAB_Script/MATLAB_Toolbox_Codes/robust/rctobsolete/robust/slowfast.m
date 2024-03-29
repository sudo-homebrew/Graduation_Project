function [a11h,b1h,c1h,d1h,a22h,b2h,c2h,d2h] = slowfast(varargin)
%SLOWFAST  Slow-fast decomposition.
%
%   [GS,GF] = SLOWFAST(G,CUT) decomposes the LTI model G into the sum of a
%   slow part GS and a fast part GF:
%
%                 G(s) = GS(s) + GF(s)
%
%   The poles of GS have smaller natural frequency than the poles of GF.
%   The integer CUT specifies the order of GS.
%
%   Note: Use FREQSEP to directly specify the cutoff frequency.
%
%   See also FREQSEP, DAMP.

%old help
%SLOWFAST State-space slow-fast decomposition.
%
% [SS_1,SS_2] = SLOWFAST(SS_,CUT) or
% [A11H,B1H,C1H,D1H,A22H,B2H,C2H,D2H] = SLOWFAST(A,B,C,D,CUT) produces
%     a decomposition of G(s) into the sum of a slow part Gs(s) and a
%     fast part Gf(s)
%
%                 G(s) = Gs(s) + Gf(s)
%     where
%                 Gs(s):= ss_1 = mksys(a11h,b1h,c1h,d1h);
%                 cut = no. of modes in Gs(s)
%                 Gf(s):= ss_2 = mksys(a22h,b2h,c2h,d2h);
%
%   The regular state-space can be recovered by "branch".

% Copyright 1988-2013 The MathWorks, Inc.
disp('  ');
disp('        - - Working on slow/fast decomposition - -');

[emsg,~,xsflag,Ts,a,b,c,d,cut]=mkargs5x('ss',varargin); error(emsg);
if Ts, error('LTI inputs must be continuous time (Ts=0)'), end

%
% ----- Real Schur Decomposition :
%
[ra,~] = size(a);
[u,t,~] = blkrsch(a,6,cut);
%
a11h = t(1:cut,1:cut);
a22h = t(cut+1:ra,cut+1:ra);
a12h = t(1:cut,cut+1:ra);
v1 = u(:,1:cut); v2 = u(:,cut+1:ra);
%
x = lyap(a11h,-a22h,a12h);
%
t1 = v1;
t2 = v1 * x + v2;
s1 = v1' - x * v2';
s2 = v2';
b1h = s1 * b;
b2h = s2 * b;
c1h = c * t1;
c2h = c * t2;
d1h = 0.5*d;
d2h = 0.5*d;
%
if xsflag
   a11h = mksys(a11h,b1h,c1h,d1h);
   b1h = mksys(a22h,b2h,c2h,d2h);
end
