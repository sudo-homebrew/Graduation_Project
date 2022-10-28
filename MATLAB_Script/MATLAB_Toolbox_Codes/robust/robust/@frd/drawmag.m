function [sysout,pts] = drawmag(in,init_pts)
% function [sysout,pts] = drawmag(in,init_pts)
%
%  Interactive mouse-based loglog sketching and fitting tool
%
%   inputs:  in = either FRD data which will be plotted each time
%  		or  CONSTANT specifying window on data [xmin xmax ymin ymax]
%            init_pts = VARYING file with initial points (optional)
%
%   outputs: sysout = fitted SYSTEM
%            pts    = VARYING file with final points
%
%  With the mouse in plot window the following keys are recognized
%	-click mouse to add points ( may be added outside current window)
%	-type 'a' to add points ( same as clicking mouse button)
%	-type 'r' to remove nearest frequency point
%	-enter integer (0-9) order for stable, minphase fit of points
%	-type 'w' to window and click again to specify second window coordinate
%	-type 'p' to replot
%	-type 'g' to toggle grid
%
%  See Also: FITMAG, GINPUT, MAGFIT, PLOT, and VPLOT.

%   Copyright 1991-2011 The MathWorks, Inc.

%%%%%%%%%%%%%%%% initial setup and argument parsing %%%%%%%%%%%%
usage = 'usage: [sysout,pts] = drawmag(in,init_pts)';
xlab = '0-9=fit p=plot  q=quit r=remove  w=window g=grid';
ylab = 'click to add';

if nargin == 0,   disp(usage);   return
end %if

if isa(in,'double')
   szin = size(in);
   if (szin(1) == 1) && (szin(2) == 4)
      in = frd(in(3:4)',in(1:2)');
      insym = '.y';
   else
      disp([usage ' in = [xmin xmax ymin ymax] '])
      return
   end
elseif isa(in,'frd')
   in = delay2z(in);
   insym = ':';
else
   disp(['IN must be 1-by-4 vector or FRD'])
   return
end



if nargin==1
   mf = [];
   omega = [];
   clf
   hold off
   uplot('liv,lm',in,insym);
else
   if ~isa(in,'frd') || szin(1)~=1 || szin(2)~=1
      disp(['INITDATA must be 1-by-1 FRD object']	)
      return
   end% if
   % Data
   [mf,omega,Ts] = frdata(init_pts);
   clf
   hold off
   uplot('liv,lm',in,insym,init_pts,'+g');
end % if nargin ==1

title('initial data')
xlabel(xlab)
ylabel(ylab);
grid_on = 0;
% Data
[in_ResponseData,Frequency,Ts] = frdata(in);
sysout_g_ResponseData = in_ResponseData(:,:,1);
sysout_g = frd(sysout_g_ResponseData,Frequency(1),Ts);
old_ord = ' ' ;
new_ord = ' ';
hold on;
[x,y,button]=ginput(1);

%%%%%%%%%%%%%%%%%%%%%%%% main loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while button ~= abs('q')
   
   if any(button == [1 2 3 abs('a')]);		        % add mode
      loglog(x,y,'+r');
      mf = [mf ; y];
      omega = [ omega ; x];
   elseif button ==  abs('r');		                % remove mode
      [~,indx] = min(abs(omega - x));
      loglog(omega(indx), mf(indx),'+w');
      omega = [omega(1:(indx-1)); omega((indx+1):length(omega))];
      mf = [mf(1:(indx-1)); mf((indx+1):length(mf))];
   elseif (button >= 48)&&(button <= 57)&&(~isempty(omega));	% fit mode
      order = button - 48;
      new_ord = ['fit order: new(solid) = ' num2str(order) '  '];
      [omega,indx] = sort(omega);
      mf = mf(indx);
      pts = frd(mf,omega,Ts);
      % NEED MAGFIT
      %sysout = umagfit(pts,[.26 .1 order order ]);
      sysout = fitmagfrd(pts,order);
      
      log_min = floor(log10(min(omega)));
      log_max = ceil(log10(max(omega)));
      omega_ex = sort([logspace(log_min,log_max,100) omega']);
      sysout_g_old = sysout_g;
      sysout_g = frd(sysout,omega_ex);
      hold off
      clf
      axis([0 0.001 0 0.001]);
      axis;
      uplot('liv,lm',in,insym,sysout_g,'-r',sysout_g_old,'--b',pts,'+g');
      title([ new_ord old_ord]);
      if grid_on
         grid
      end
      hold on
      xlabel(xlab);
      ylabel(ylab);
      old_ord = ['old(dashed) = ' num2str(order) '  '];
   elseif (button >= 48)&&(button <= 57)&&(isempty(omega));	% fit mode
      disp('must have data to fit');
   elseif (button == abs('w'));			% window mode
      [x1,y1] = ginput(1);
      x = [x x1];
      y = [y y1];
      hold off
      clf;
      [omega,indx] = sort(omega);
      mf = mf(indx);
      pts = frd(mf,omega);
      uplot('liv,lm',in,insym,sysout_g,'-r',pts,'+g'); % not version 3
      axis( [10^(floor(log10(min(x)))),10^(ceil(log10(max(x)))),...
         10^(floor(log10(min(y)))),10^(ceil(log10(max(y)))) ] )
      if grid_on
         grid
      end
      hold on
      xlabel(xlab)
      ylabel(ylab);
      title(['window mode:  ' new_ord ]);
   elseif (button == abs('p'));			% plot mode
      hold off
      clf
      axis([0 0.0001 0 0.0001]);
      axis;
      [omega,indx] = sort(omega);
      mf = mf(indx);
      pts = frd(mf,omega);
      uplot('liv,lm',in,insym,sysout_g,'-r',pts,'+g');
      if grid_on
         grid
      end
      hold on
      xlabel(xlab)
      ylabel(ylab)
      title([ new_ord ]);
   elseif (button == abs('k'))
      keyboard;
   elseif (button == abs('g'))
      grid_on = ~grid_on;
   end %if
   
   [x,y,button]=ginput(1);
   
end %while

%%%%%%%%%%%%%%%%%%%%%%% clean up and return %%%%%%%%%%%%%%%%%%
[omega,indx] = sort(omega);
mf = mf(indx);
pts = frd(mf,omega,Ts,'FrequencyUnit',in.FrequencyUnit);

% XXX
% Time units of sysout needs to be consistent with FrequencyUnits of pts

% hold off; axis([0 0.0001 0 0.0001]); axis('auto');
hold off
axis([0 0.0001 0 0.0001]);
axis;
clf
