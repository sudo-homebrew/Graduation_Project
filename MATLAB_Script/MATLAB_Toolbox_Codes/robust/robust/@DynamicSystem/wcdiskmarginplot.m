function hout = wcdiskmarginplot(uL,varargin)
%WCDISKMARGINPLOT  Plot worst-case disk margins of uncertain feedback loop.
%
%   WCDISKMARGINPLOT(L,SIGMA) plots the nominal and worst-case disk-based 
%   gain and phase margins of the uncertain open-loop response L. Here 
%   "worst-case" refers to the combination of uncertain element values
%   that minimizes the disk margin of the feedback loop. The scalar SIGMA
%   specifies the skew (SIGMA=0 by default, see DISKMARGIN for detail). 
%   For MIMO feedback loops, WCDISKMARGINPLOT plots the multi-loop disk 
%   margins.
%
%   WCDISKMARGINPLOT(L,SIGMA,W) draws the margins at the frequencies 
%   specified by W (expressed in radians per system time units). When 
%   W = {WMIN,WMAX}, the plot is drawn for frequencies between WMIN and 
%   WMAX. When W is a vector of frequencies, the margins are computed at 
%   these specific frequencies.
% 
%   WCDISKMARGINPLOT(...,OPTS) specifies additional options for the 
%   worst-case margin computation or the disk margin plot, see WCOPTIONS 
%   and DISKMARGINOPTIONS for details.
%
%   See also WCDISKMARGIN, WCOPTIONS, DISKMARGINOPTIONS, DISKMARGIN, 
%   DISKMARGINPLOT, USS, UFRD.

%   Copyright 2020-2021 The MathWorks, Inc.
[nL,nu,nsys] = size(uL);
if nL~=nu
   error(message('Robust:analysis:NonSquareLoopGain'))
elseif nsys>1
   error(message('Robust:analysis:WCG3'))
end

% Look for plot options
idx = cellfun(@(x) isa(x,'plotopts.DiskMarginOptions'),varargin);
plotopt = varargin(idx);
varargin(:,idx) = [];

% Look for wcOptions
idx = find(cellfun(@(x) isa(x,'rctoptions.wcAnalysis'),varargin));
if numel(idx)==1
   wcopt = varargin{idx};
   varargin(:,idx) = [];
else
   wcopt = wcOptions();
end
wcopt.Display = 'off';  
wcopt.Sensitivity = 'off';  % for speed
wcopt.VaryFrequency = 'on'; % plot worst-case gain vs. frequency

% Skew
if ~isempty(varargin) && isscalar(varargin{1})
   sigma = varargin{1};
   if ~(isnumeric(sigma) && isreal(sigma) && isfinite(sigma))
      error(message('Robust:analysis:InvalidE'))
   end
   varargin = varargin(2:end);
else
   sigma = 0; % default
end
   
% Should be zero or one argument left (frequency spec)
if numel(varargin)>1
   error(message('Robust:analysis:wcdiskmarginplot1'))
elseif isscalar(varargin)
   % Convert to UFRD when specifying explicit frequency grid
   w = DynamicSystem.checkFreqSpec(varargin{1},true);
   if isnumeric(w)
      if isa(uL,'FRDModel')
         error(message('Robust:analysis:ROBSTAB1'))
      else
         uL = ufrd(uL,w);
      end
   end
end

% Compute worst-case gain
Ts = uL.Ts;
try
   [~,WCU,Info] = wcdiskmargin(uL,'mimo',sigma,wcopt);
catch ME
   throw(ME)
end
w = Info.Frequency;
if w(1)==-Inf
   w(1) = 1e4*w(2);
end
if w(end)==Inf
   w(end) = 1e4*w(end-1);
end

% Create plot
h = diskmarginplot(uL,'c:',getValue(uL),'b',usubs(uL,WCU),'b--',...
   sigma,varargin{:},plotopt{:});

% Add curves for lower and upper bounds
LS = {'-','--'};
REAL = h.Responses(2).Data.Real;
Focus = h.Responses(3).Data.Focus;  % focus for worst perturbation
deg2rad = pi/180;
for ct=1:2
   [gm,pm] = dm2gmPlot(Info.Bounds(:,ct),sigma);
   r = addplot(h,w,gm,deg2rad*pm,Ts,Focus,REAL);
   r.setstyle('LineStyle',LS{ct},'Color','r')
   draw(r)
end

% Legend
legend('Sampled uncertainty','Nominal','Worst perturbation',...
   'Worst-case margin (lower bound)',...
   'Worst-case margin (upper bound)','Location','SouthWest')

if nargout>0
   hout = h;
end
