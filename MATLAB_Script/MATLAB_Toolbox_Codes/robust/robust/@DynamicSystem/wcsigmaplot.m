function hout = wcsigmaplot(sys,varargin)
%WCSIGMAPLOT   Plot worst-case gain of uncertain system.
%
%   WCSIGMAPLOT(USYS) plots the nominal and worst-case gains of the 
%   uncertain system USYS as a function of frequency. Here "worst-case" 
%   refers to the uncertainty value that maximizes the peak gain of the 
%   system (see WCGAIN). For MIMO systems, "gain" refers to the singular 
%   values of the frequency response matrix.
%
%   WCSIGMAPLOT(USYS,{WMIN,WMAX}) focuses on frequencies ranging between
%   WMIN and WMAX (in rad/TimeUnit).
%
%   WCSIGMAPLOT(USYS,W) uses the frequency grid W (in rad/TimeUnit) to
%   calculate the worst-case gain.
%
%   WCSIGMAPLOT(USYS,...,OPTS) specifies additional options for the 
%   worst-case gain computation or the singular value plot, see WCOPTIONS 
%   and SIGMAOPTIONS for details.
%
%   See also WCGAIN, WCOPTIONS, SIGMA, SIGMAOPTIONS, USS, UFRD.

%   Copyright 2003-2021 The MathWorks, Inc.
if nmodels(sys)>1
   error(message('Robust:analysis:WCG3'))
end

% Look for plot options
idx = cellfun(@(x) isa(x,'plotopts.SigmaOptions'),varargin);
plotopt = varargin(idx);
varargin(:,idx) = [];

% Look for wcOptions
idx = find(cellfun(@(x) isa(x,'rctoptions.wcgain') || ...
   isa(x,'rctoptions.wcAnalysis'),varargin));
if numel(idx)==1
   wcopt = varargin{idx};
   varargin(:,idx) = [];
else
   wcopt = wcOptions();
end
wcopt.Display = 'off';  
wcopt.Sensitivity = 'off';  % for speed
wcopt.VaryFrequency = 'on'; % plot worst-case gain vs. frequency

% Should be zero or one argument left
if numel(varargin)>1
   error(message('Robust:analysis:wcsigmaplot1'))
end

% Compute worst-case gain
Ts = sys.Ts;
TimeUnit = sys.TimeUnit;
try
   [WCG,WCU,Info] = wcgain(sys,varargin{:},wcopt);
catch ME
   throw(ME)
end
if isinf(WCG.LowerBound)
   error(message('Robust:analysis:WCG4'))
end
w = Info.Frequency;
if w(end)==Inf
   w(end) = 1e4*w(end-1);
end
Info.Bounds(isinf(Info.Bounds)) = NaN; % FRD does not take Inf
LB = frd(Info.Bounds(:,1),w,'Ts',Ts,'TimeUnit',TimeUnit);
UB = frd(Info.Bounds(:,2),w,'Ts',Ts,'TimeUnit',TimeUnit);

% Create plot
sysWC = usubs(sys,WCU);
sysNOM = getValue(sys);
h = sigmaplot(sys,'c:',sysNOM,'b',sysWC,'b--',...
   LB,'r--',UB,'r-',varargin{:},plotopt{:});
legend('Sampled uncertainty','Nominal','Worst perturbation','Worst-case gain (lower bound)',...
   'Worst-case gain (upper bound)','Location','SouthWest')

% Adjust Y limits to emphasize peak gain area
wcFocus = h.Responses(3).Data.Focus;
h.Responses(4).Data.Focus = wcFocus;
h.Responses(5).Data.Focus = wcFocus;
PeakGain = max(h.Responses(5).Data.SingularValues);
plotopt = getoptions(h);
ylim = plotopt.YLim{1};
if strcmp(plotopt.MagUnits,'dB')
   ylim(2) = min(ylim(2),20*ceil(mag2db(PeakGain)/20));
   ylim(1) = max(ylim(1),ylim(2)-60);
else
   ylim(2) = min(ylim(2),2*PeakGain);
   ylim(1) = max(ylim(1),PeakGain/1e3);
end
plotopt.YLim{1} = ylim;
setoptions(h,plotopt)

if nargout>0
   hout = h;
end
