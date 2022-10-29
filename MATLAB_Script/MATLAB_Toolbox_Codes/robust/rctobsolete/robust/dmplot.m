function [dgo,dp] = dmplot(gm)
%DMPLOT  Interpretation of disk gain and phase margins.
%
%   DMPLOT is obsolete, use DISKMARGINPLOT instead. The counterpart of
%      dmplot(DGM)
%   is
%      diskmarginplot(DGM)
%
%   See also DISKMARGINPLOT.

% Copyright 2004-2005 The MathWorks, Inc.

% Old help
%DMPLOT  Interpretation of disk gain and phase margins.
%
%   DMPLOT by itself brings up a plot illustrating the concept
%   of disk gain margin (DGM) and disk phase margin (DPM).  Both
%   are derived from the largest disk that contains the critical 
%   point (-1,0) and does not intersect the Nyquist plot of the
%   open-loop response L.  DGM and DPM are lower bounds on the 
%   classical gain and phase margins and are used to estimate 
%   how much combined gain/phase variations can be tolerated.
%
%   DMPLOT(DGM) plots, for a given disk gain margin DGM, the 
%   trade-off curve of admissible gain/phase variations.  
%   The closed-loop system is guaranteed to remain stable for 
%   all combined gain/phase variations inside the plotted ellipse.
%   On this plot, the maximum gain and phase variations correspond
%   to the disk margins DGM and DPM.  DMPLOT expects an absolute 
%   gain value DGM.
%
%   [DG,DP] = DMPLOT(DGM) returns the data used to plot the 
%   gain/phase variation ellipse.
%
%   See also DISKMARGIN, WCMARGIN, MARGIN.
if nargin==0
   if nargout>0
      error('Too many outputs.')
   end
   sw = warning('off'); lw = lastwarn;
   load diskmargin imageMap
   imageHandle = image(imageMap);
   axisHandle = get(imageHandle, 'parent');
   figureHandle = get(axisHandle, 'parent');
   set(figureHandle, 'Units', 'normal', 'Position', [.2 .2 .4 .4]);
   set(figureHandle, 'Units', 'pixel');
   pos = get(gcf, 'Position'); 
   set(figureHandle,'Position',[pos(1) pos(2) size(imageMap,2) size(imageMap,1)]); 
   set(axisHandle,'Position',[0 0 1 1], 'XTick', [], 'YTick', []) 
   warning(sw); lastwarn(lw)
   disp(LocalDMtext)
   return
end

if ~isnumeric(gm) || ~isscalar(gm) || ~isreal(gm) || gm<=0
   error('DGM must be a positive, non-zero scalar double.')
end
cla
i = pi*[-0.5:.01:0.5]';
cc = sin(i)+1i*cos(i);
if gm<1
   gm = 1/gm;
end
if gm~=1
   m = (gm+1)/(gm-1);
   z = (1+m*cc)./(1-m*cc);
else
   z = 0.00*cc;
   disp('Disk margin is 0 for a gain margin of 1.')
end
dp = abs((180/pi)*angle(z)-180);
dg = 20*log10(abs(z));

if nargout==0
   h = plot(dg, dp);
   set(h,'LineWidth',2)
   grid
   xlabel('Gain Variation (dB)')
   ylabel('Phase Variation (deg)')
   Title = sprintf(...
      'Allowable Gain/Phase Variations for a %.3g Disk Gain Margin.\n%s',...
      gm,'(stability is guaranteed for all variations inside the ellipse)');
   title(Title)
else
   dgo = dg;
end


function text = LocalDMtext

ta = ' ';
t1 = 'This figure shows a comparison of a disk margin analysis';
t2 = 'with the classical notations of gain and phase margins.';
t3 = 'The Nyquist plot is of the loop transfer function';
t3a = ' ';
t4 = '         L = 4(s/30 + 1)/((s+1)*(s^2 + 1.6s + 16))';
t6 = ' ';
t7 = ' - The Nyquist plot of L corresponds to the blue line'; 
t8 = ' - The unit disk corresponds to the dotted red line';
t9 = ' - GM and PM indicate the location of the classical gain';
t10= '    and phase margins for the system L.';
t11= ' - DGM and DPM correspond to the disk gain and phase ';
t12= '   margins. The disk margins provide a lower bound on';
t13= '   classical gain and phase margins.';
t14= ' - The disk margin circle corresponds to the dashed black';
t15= '   line. The disk margin corresponds to the largest disk';
t16= '   centered at (GMD + 1/GMD)/2 that just touches the';
t17= '   loop transfer function L. This location is indicated';
t18= '   by the red dot.';

text = str2mat(ta,t1,t2,t3,t3a,t4,t6,t7,t8,t9,t10,t11,t12,t13,...
            t14,t15,t16,t17,t18);

