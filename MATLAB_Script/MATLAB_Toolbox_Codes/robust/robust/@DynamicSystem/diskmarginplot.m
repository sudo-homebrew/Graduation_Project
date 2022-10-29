function hout = diskmarginplot(varargin)
%DISKMARGINPLOT  Graphical disk margin analysis.
%
%   DISKMARGINPLOT helps visualize the results of the stability margin
%   analysis performed by DISKMARGIN. This includes gain and phase margin
%   plots, exclusion regions in the Nyquist plot, and ranges of gain and
%   phase variations corresponding to specific disk margins.
%
%   Disk-based gain and phase margins vs. frequency:
%
%   DISKMARGINPLOT(L,SIGMA) draws the disk-based margins for the negative
%   feedback loop with open-loop response L. The scalar SIGMA specifies
%   the skew (SIGMA=0 by default, see DISKMARGIN for details). For MIMO
%   responses L, DISKMARGINPLOT plots the multi-loop disk margins.
%
%   DISKMARGINPLOT(L1,L2,...,SIGMA) draws the disk-based margins for several
%   open-loop responses L1,L2,... on a single plot. You can specify a color,
%   line style, and marker for each model, as in
%      diskmarginplot(L1,'r',L2,'y--',L2,'gx').
%
%   DISKMARGINPLOT(...,W) draws the margins at the frequencies specified
%   by W (expressed in radians/system time units). When W = {WMIN,WMAX},
%   the plot is drawn for frequencies between WMIN and WMAX. When W is a
%   vector of frequencies, the margins are computed at the specified
%   frequencies.
%
%   DISKMARGINPLOT(...,PLOTOPTIONS) plots the disk-based margins with the
%   options specified in PLOTOPTIONS. See DISKMARGINOPTIONS for details.
%
%   Range of gain and phase variations:
%
%   DISKMARGINPLOT(DGM) plots the range of gain and phase variations
%   corresponding to the disk-based gain margin DGM (see GainMargin field
%   in DISKMARGIN). This shows the maximum gain-only and phase-only
%   variations (disk-based gain and phase margins) and the combined gain
%   and phase variations in between. DGM is a two-entry vector [GMIN,GMAX]
%   with GMIN<1 and GMAX>1. Use multiple rows in DGM to show several ranges
%   on the same plot.
%
%   DISKMARGINPLOT(DGM,'disk') plots the disk of values taken by the
%   multiplicative factor modeling gain and phase uncertainty (see
%   DISKMARGIN). This disk intersects the real axis in the interval
%   [GMIN,GMAX] and is the origin of the term "disk margin".
%
%   DISKMARGINPLOT(DGM,'nyquist') plots the corresponding exclusion region
%   in the Nyquist plane.
%
%   DISKMARGINPLOT(ALPHA,SIGMA,...) creates the same plots from the disk
%   margin ALPHA and skew SIGMA instead of the gain margin DGM. See DM2GM
%   or  GM2DM for details.
%
%   Plotting on specific axes:
%
%   DISKMARGINPLOT(AX,...) plots into the axes with handle AX.
%
%   Example: Compare the gain and phase margins of two open-loop responses
%   for SIGMA=-1:
%      L1 = tf(25,[1 10 10 10]);
%      L2 = tf([1 100],[1 10 20 50]);
%      diskmarginplot(L1,L2,-1)
%
%   Example: Show the "safe" (stable) range of combined gain and phase
%   variations corresponding to the gain margin [0.3,2].
%      diskmarginplot([0.3 2])
%
%   Example: The gain and phase uncertainty disk grows with ALPHA
%      diskmarginplot([1.5 1 0.5],0,'disk')
%   and shifts to the right when SIGMA increases:
%      diskmarginplot([0.36 0.75 0.66],[-5 -1 0],'disk')
%
%   Example: Compute the disk margins for several values of SIGMA and show
%   the corresponding exclusion regions in the Nyquist plot.
%      L = tf(25,[1 10 10 10]);
%      sigma = [-3 0 3];
%      for k=1:3
%         DM(k) = diskmargin(L,sigma(k));
%      end
%      nyquist(L), hold
%      diskmarginplot(cat(1,DM.GainMargin),'nyquist')
%
%   See also DISKMARGINOPTIONS, DISKMARGIN, GETDGM, DM2GM, GM2DM, UMARGIN, DYNAMICSYSTEM.

%   Copyright 1986-2021 The MathWorks, Inc.

% Get argument names
for ct = length(varargin):-1:1
   ArgNames(ct,1) = {inputname(ct)};
end

% Parse input list
% Check for axes argument
if ishghandle(varargin{1},'axes')
   ax = varargin{1};
   varargin(1) = [];
   ArgNames(1) = [];
else
   ax = [];
end

% Parsing
try
   % Look for SIGMA
   ixE = find(cellfun(@(x) isnumeric(x) && isscalar(x),varargin),1);
   if isempty(ixE)
      sigma = 0;
   else
      sigma = varargin{ixE};
      varargin(ixE) = [];
      ArgNames(ixE) = [];
   end
   [sysList,Extras,OptionsObject] = DynamicSystem.parseRespFcnInputs(varargin,ArgNames);
   if ~(isempty(OptionsObject) || isa(OptionsObject,'plotopts.DiskMarginOptions'))
      error(message('Robust:analysis:diskmarginplot11'))
   end
   [sysList,w] = DynamicSystem.checkBodeInputs(sysList,Extras);
   TimeUnits = sysList(1).System.TimeUnit; % first system determines units
   % Warn about and skip empty systems
   isEmptySys = arrayfun(@(x) isempty(x.System),sysList);
   if any(isEmptySys)
      warning(message('Control:analysis:PlotEmptyModel'))
   end
   sysList = sysList(~isEmptySys);
   % Check time unit consistency when specifying w or {wmin,wmax}
   if ~(isempty(w) || ltipack.hasMatchingTimeUnits(TimeUnits,sysList.System))
      error(message('Control:analysis:AmbiguousFreqSpec'))
   end
   
   % Create source object and check compatibility
   nsys = numel(sysList);
   src = repmat(handle(NaN),nsys,1);
   style = cell(nsys,1);
   for ct=1:length(sysList)
      sysInfo = sysList(ct);
      L = sysInfo.System;
      [ny,nu] = iosize(L);
      if ny~=nu
         error(message('Robust:analysis:NonSquareLoopGain'))
      end
      src(ct) = resppack.marginsource(L,sigma);
      src(ct).Name = sysInfo.Name;
      style{ct} = sysInfo.Style;
   end
   
   % Create plot (visibility ='off')
   if isempty(ax)
      ax = gca;
   end
   h = ltiplot(ax,'diskmargin',[],[],OptionsObject,cstprefs.tbxprefs);
catch ME
   throw(ME)
end

% Set user-defined focus (specifies preferred limits)
if isnumeric(w)
   w = unique(w);
end
h.setFreqFocus(w,['rad/' TimeUnits]);  % w is in rad/TimeUnit

% Configure responses
for ct = 1:length(src)
   % Link each response to system source
   r = h.addresponse(src(ct));
   DefinedCharacteristics = src(ct).getCharacteristics('diskmargin');
   r.setCharacteristics(DefinedCharacteristics);
   r.DataFcn = {'diskmargin' src(ct) r w};
   r.setstyle(style{ct})
end

% Draw now
if strcmp(h.AxesGrid.NextPlot,'replace')
   h.Visible = 'on';  % new plot created with Visible='off'
else
   draw(h)  % hold mode
end

% Right-click menus
ltiplotmenu(h,'diskmargin');

if nargout>0
   hout = h;
end
