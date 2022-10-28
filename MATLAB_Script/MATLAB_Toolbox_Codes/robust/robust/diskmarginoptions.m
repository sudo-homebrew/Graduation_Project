function p = diskmarginoptions(varargin)
%DISKMARGINOPTIONS Creates option set for disk margin plot.
%
%   P = DISKMARGINOPTIONS returns the default option set for disk margin 
%   plots. Use P to customize the disk margin appearance from the command
%   line. For example
%         p = diskmarginoptions;
%         p.FreqUnits = 'Hz'; 
%         p.MagScale = 'log'; 
%         p.MagUnits = 'abs';
%         h = diskmarginplot(tf(25,[1 10 10 10]),p);
%   creates a disk margin plot with frequency units in Hz and a log scale 
%   for the gain margin plot.
%
%   P = DISKMARGINOPTIONS('cstprefs') initializes the plot options with the
%   Control System Toolbox preferences.
%
%   Available options include:
%      FreqUnits                     Frequency units
%      FreqScale [linear|log]        Frequency scale
%      MagUnits [dB|abs]             Magnitude units
%      MagScale [linear|log]         Magnitude scale
%      PhaseUnits [deg|rad]          Phase units
%      Title, XLabel, YLabel         Label text and style
%      TickLabel                     Tick label style
%      Grid   [off|on]               Show or hide the grid 
%      XlimMode, YlimMode            Limit modes
%      Xlim, Ylim                    Axes limits
%
%   See also DISKMARGINPLOT, WRFC/SETOPTIONS, WRFC/GETOPTIONS.

%  Copyright 1986-2021 The MathWorks, Inc.
p = plotopts.DiskMarginOptions(varargin{:});