function diskmarginplot(varargin)
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

%   Copyright 1986-2020 The MathWorks, Inc.

% Check for axes argument
ni = numel(varargin);
if ni>0 && isscalar(varargin{1}) && ishghandle(varargin{1},'axes')
   ax = varargin{1};
   varargin = varargin(2:end);
   ni = ni-1;
else
   ax = [];
end

% Plot type
if ni>1 && (ischar(varargin{ni}) || isstring(varargin{ni}))
   PLOTTYPE = ltipack.matchKey(varargin{ni},{'disk','nyquist'});
   if isempty(PLOTTYPE)
      error(message('Robust:analysis:diskmarginplot5'))
   end
   ni = ni-1;
else
   PLOTTYPE = 'envelope';
end

if ni==1
   % Specifying DGM
   gm = varargin{1};
   try
      [alpha,sigma] = gm2dm(gm);
   catch ME
      throw(ME)
   end
   [gm,pm] = dm2gm(alpha,sigma);
   N = size(gm,1);
   DGMSPEC = true;
elseif ni==2
   % Specifying ALPHA,SIGMA
   alpha = varargin{1}(:);
   sigma = varargin{2}(:);
   try
      [gm,pm] = dm2gm(alpha,sigma);
   catch ME
      throw(ME)
   end
   % Equalize row lengths
   N = size(gm,1);
   if N>1 && isscalar(alpha)
      alpha = repmat(alpha,[N 1]);
   end
   if N>1 && isscalar(sigma)
      sigma = repmat(sigma,[N 1]);
   end
   % Require alpha*|1+sigma|<2 so that the uncertainty region is the
   % interior of a disk
   % NOTE: Equivalent to (1-GMIN)(1-GMAX)<0.
   if ~all(alpha>=0 & alpha.*abs(1+sigma)<2)
      error(message('Robust:analysis:diskmarginplot4'))
   end
   DGMSPEC = false;
else
   error(message('Robust:analysis:diskmarginplot1'))
end


% Target axes
if isempty(ax)
   ax = gca;
end
REPLACE = strcmp(ax.NextPlot,'replace');
if REPLACE
   cla(ax,'reset')
   set(ax,'Xgrid','on','Ygrid','on')
end

% Legend
SHOWLEGEND = (~DGMSPEC || N>1);
if SHOWLEGEND
   legstr = cell(N,1);
   if DGMSPEC
      for ct=1:N
         legstr{ct} = sprintf('DGM = [%.3g,%.3g], DPM = %.3g\n',gm(ct,1),gm(ct,2),pm(ct,2));
      end
   else
      for ct=1:N
         legstr{ct} = sprintf('alpha = %.3g, skew = %.3g\n',alpha(ct),sigma(ct));
      end
   end
end

% Create plot
Colors = {[0 0.5 0],[0 0 1],[1 0 0],[0.8 0.8 0.6]};
Ncolors = numel(Colors);
hp = [];
switch PLOTTYPE
   case 'envelope'
      th0 = (0:0.01:1)*pi;  % baseline grid
      for ct=1:N
         gmin = gm(ct,1);   gmax = gm(ct,2);
         c = (gmin+gmax)/2;  r = (gmax-gmin)/2;
         if c<0
            % PM=Inf for all gains in [GMIN,GMAX] range. This happens when 
            % sigma < -sqrt(1+4/alpha^2), e.g., diskmarginplot(1.9,-2)
            x = [gmax gmax -20];
            y = [0 180 180];
         else
            if c<2*r
               % Supplement grid for smoothness of envelope. See, e.g,
               %   diskmarginplot(0.99,1)      % c>r
               %   diskmarginplot(0.65,-2.2)   % c<r
               if c>r
                  th_peak = pi-acos(r/c);  % max phase variation
                  psi = (0:0.01:1) * acos(2*sqrt((1-r^2/c^2)/3));
                  th = pi+psi-asin((c/r)*sin(psi));
               else
                  th_peak = [];
                  psi = (1:0.01:2) * (pi/2);
                  th = psi+asin((c/r)*sin(psi));
               end
               z = c + r * exp(complex(0,sort([th0 th th_peak])));
            else
               z = c + r * exp(complex(0,th0));
            end
            x = 20*log10(abs(z));
            y = 180/pi * angle(z);
         end
         hp = [hp ; patch(x,y,zeros(size(x)),Colors{1+rem(ct-1,Ncolors)},...
            'Parent',ax,'FaceAlpha',0.5,'EdgeColor','none')];
      end
      % Add annotations
      if N==1
         dx = 0.02*diff(ax.XLim);
         dy = 0.02*diff(ax.YLim);
         xrange = x([end,1]);  % x-extent of envelope
         [~,is] = max(abs(xrange));
         text(xrange(is)/2,dy,sprintf('DGM = [%.2g,%.2g]',gmin,gmax),...
            'Parent',ax,'HorizontalAlignment','center',...
            'VerticalAlignment','bottom','FontSize',10)
         if abs(alpha*sigma)<2
            pm = pm(2);
            line([0 0],[0 pm],'Parent',ax,'Color','k','LineWidth',1)
            line(0,pm,'Parent',ax,'Color','k','Marker','.','MarkerSize',10)
            text(dx,pm/2,sprintf('DPM = %.2g deg',pm),...
               'Parent',ax,'VerticalAlignment','bottom','FontSize',10)
         end
      end
      if SHOWLEGEND
         legend(hp,legstr)
      end
      xlabel(ax,getString(message('Robust:analysis:diskmarginplot6')))
      ylabel(ax,getString(message('Robust:analysis:diskmarginplot7')))
      title(ax,getString(message('Robust:analysis:diskmarginplot8')))
      % Adjust extent of left-unbounded patches (GMIN<0)
      %    diskmarginplot(0.7,[-2.2 -2.5 -3]) 
      if any(gm(:,1)<0)
         ax.XLimMode = 'manual';
         xmin = ax.XLim(1)-1;
         ixU = find(gm(:,1)<0);
         for ct=1:numel(ixU)
            p = hp(ixU(ct));
            x = [p.XData ; xmin ; xmin];
            y = [p.YData ; 180 ; 0];
            set(p,'XData',x,'YData',y,'ZData',zeros(size(x)))
         end
      end
      
   case 'disk'
      uc = exp(complex(0,pi*(0:0.01:2)));
      for ct=1:N
         gmin = gm(ct,1);   gmax = gm(ct,2);
         c = (gmin+gmax)/2;  r = (gmax-gmin)/2;
         z = c + r * uc;
         hp = [hp ; patch(real(z),imag(z),zeros(size(z)),Colors{1+rem(ct-1,Ncolors)},...
            'Parent',ax,'FaceAlpha',0.5,'EdgeColor','none')];
      end
      axis(ax,'equal')
      if N==1
         % Add GM/PM annotations
         dx = 0.02*diff(ax.XLim);
         dy = 0.01*diff(ax.YLim);
         line([gmin gmax],[0 0],'Parent',ax,'Color','k')
         line(gmin,0,'Parent',ax,'Color','k','Marker','.','MarkerSize',10,'LineWidth',1)
         line(gmax,0,'Parent',ax,'Color','k','Marker','.','MarkerSize',10,'LineWidth',1)
         text((gmin+gmax)/2,-dy,sprintf('DGM = [%.2g,%.2g]',gmin,gmax),...
            'Parent',ax,'HorizontalAlignment','center',...
            'VerticalAlignment','top','FontSize',10)
         if abs(alpha*sigma)<2
            pm = pm(2);
            phi = (0:0.01:1)*pm;
            line(cosd(phi),sind(phi),'Parent',ax,'Color','k','LineWidth',1)
            text(cosd(pm/2)+dx,sind(pm/2),sprintf('DPM = %.2g deg',pm),...
               'Parent',ax,'VerticalAlignment','bottom','FontSize',10)
         end
      end
      line(1,0,'Parent',ax,'Color','r','Marker','+','LineWidth',2,'MarkerSize',10)
      if SHOWLEGEND
         legend(hp,legstr)
      end
      title(ax,getString(message('Robust:analysis:diskmarginplot9')))
      
   case 'nyquist'
      if ~REPLACE
         % This is to restore transparency of patches superimposed on Nyquist plot
         ax.SortMethod = 'childorder';
      end
      if N==1
         Colors = {[1 0 0]}; % show red for exclusion when showing single region
      end
      uc = exp(complex(0,(0:0.01:2)*pi));
      for ct=1:N
         gmin = gm(ct,1);   gmax = gm(ct,2);
         xmin = -1/gmin;   xmax = -1/gmax;
         c = (xmin+xmax)/2;  r = abs(xmax-xmin)/2;
         z = c + r * uc;
         % Note: Exclude patch from limit picking, too hard to manage half
         % planes, large disks, and exterior situations. Instead use 
         % "invisible" line to enforce minimum extent around critical point
         line([-1.5 1.5],[-1.5 1.5],'Color','w','LineStyle','none')
         if gmin<0
            % Exclusion region is the exterior of a disk
            XData = [real(z),1e8,1e8,-1e8,-1e8,1e8,1e8];
            YData = [imag(z),0,-1e8,-1e8,1e8,1e8,0];
            ZData = [zeros(1,numel(z)+6)];
            p = patch(XData,YData,ZData,Colors{1+rem(ct-1,Ncolors)},...
               'Parent',ax,'FaceAlpha',0.5,'EdgeColor','none',...
               'XlimInclude','off','YlimInclude','off');
         else
            % Exclusion region is the interior of a disk
            p = patch(real(z),imag(z),zeros(size(z)),Colors{1+rem(ct-1,Ncolors)},...
               'Parent',ax,'FaceAlpha',0.5,'EdgeColor','none',...
               'XlimInclude','off','YlimInclude','off');
         end
         hp = [hp ; p]; %#ok<*AGROW>
      end
      axis(ax,'equal')
      if N==1
         % Add GM/PM annotations
         line(real(uc),imag(uc),'Color',[.75 .75 .75],'LineStyle','-.');
         dx = 0.1*min(r,diff(ax.XLim)/2);
         dy = 0.01*diff(ax.YLim);
         if gmin>0
            line([-1 xmax],[0 0],'Parent',ax,'Color','k','Marker','.','MarkerSize',10,'LineWidth',1)
            text(-1+2*dx,dy,sprintf('DGM = %.2g dB',20*log10(gmax)),...
               'Parent',ax,'HorizontalAlignment','left','VerticalAlignment','bottom')
            line([xmin -1],[0 0],'Parent',ax,'Color','k',...
               'XLimInclude','off','Marker','.','MarkerSize',10,'LineWidth',1)
            text(-1-2*dx,-dy,sprintf('DGM = %.2g dB',20*log10(gmin)),...
               'Parent',ax,'HorizontalAlignment','right','VerticalAlignment','top')
         else
            line([-1 xmax],[0 0],'Parent',ax,'Color','k','Marker','.','MarkerSize',10,'LineWidth',1)
            text(-1+2*dx,dy,sprintf('DGM = %.2g dB',20*log10(gmax)),...
               'Parent',ax,'HorizontalAlignment','left','VerticalAlignment','bottom')
         end
         if abs(alpha*sigma)<2
            pm = pm(2);
            phi = 180 + (0:0.01:1)*pm;
            line(cosd(phi),sind(phi),'Parent',ax,'Color','k','LineWidth',1)
            line(-cosd(pm),-sind(pm),'Parent',ax,'Color','k','Marker','.','MarkerSize',10);
            text(cosd(180+0.6*pm)+dx,sind(180+0.6*pm),sprintf('DPM = %.2g deg',pm),...
               'Parent',ax,'HorizontalAlignment','left','FontSize',10);
         end
      end
      line(-1,0,'Parent',ax,'Color','r','Marker','+','LineWidth',2,'MarkerSize',10)
      if SHOWLEGEND
         legend(hp,legstr)
      end
      title(ax,getString(message('Robust:analysis:diskmarginplot10')))
end
