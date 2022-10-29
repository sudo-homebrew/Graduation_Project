function han = uplot(varargin)
% UPLOT  Plot of double and FRD data.
%
% UPLOT([plot_type],SYS1,SYS2,SYS3, ...)
% UPLOT([plot_type],[1 10],[.1 5],SYS1, ...)
% UPLOT([plot_type],SYS1,'linetype1',SYS2,'linetype2',...)
%
% Plot double and FRD objects.  The syntax is the same as the MATLAB
% plot command except that all data is contained in SYSi, and the 
% axes are specified by PLOT_TYPE.
%
% The (optional) plot_type argument must be one of:
%
%   'iv,d'       matin .vs. independent variable (default option)
%   'iv,m'       magnitude .vs. independent variable
%   'iv,lm'      log(magnitude) .vs. independent variable
%   'iv,p'       phase .vs. independent variable
%   'liv,d'      matin .vs. log(independent variable)
%   'liv,m'      magnitude .vs. log(independent variable)
%   'liv,lm'     log(magnitude) .vs. log(independent variable)
%   'liv,p'      phase .vs. log(independent variable)
%   'nyq'        real .vs. imaginary  (parametrized by indep variable)
%   'nic'        Nichols chart
%   'bode'       Bode magnitude and phase plots
%
%See also: BODE, LOGLOG, PLOT, NICHOLS, NYQUIST, SEMILOGX, SEMILOGY, SIGMA.

%   Copyright 2004-2011 The MathWorks, Inc.

nin = nargin;
if isa(varargin{1},'char')
   plottype = varargin{1};
   sidx = 2;
else
   plottype = 'iv,d';
   sidx = 1;
end

argcell = cell(0,1);
cnt = 1;
cflag = 0;
dflag = 0;
ydataloc = [];
for i=sidx:nin
   arg = varargin{i};
   switch class(arg)
   case 'frd'
      if dflag==1
         error('Double data must come in pairs');
      else
         cflag = 0;
         arg = absorbDelay(arg);
         szm = size(arg);
         if length(szm)==2
            npts = length(arg.Frequency);
            ydata = reshape(arg.ResponseData,[szm(1)*szm(2) npts]).';
            xdata = arg.Frequency;
            argcell = [argcell;{xdata};{ydata}];
            ydataloc = [ydataloc;cnt+1];
            cnt = cnt + 2;
         else
            nad = length(szm) - 2;
            npts = length(arg.Frequency);
            tmp = permute(arg.ResponseData,[1 2 4:4+nad-1 3]);
            ydata = reshape(tmp,[prod(szm) npts]).';
            xdata = arg.Frequency;
            argcell = [argcell;{xdata};{ydata}];
            ydataloc = [ydataloc;cnt+1];
            cnt = cnt + 2;
         end
      end
   case 'char'
      if dflag==1
         error('Double data must come in pairs');
      else
         if cflag==0
            argcell = [argcell;{arg}];
            cnt = cnt + 1;
            cflag = 1;
         else
            error('Never have 2 chars in a row');
         end
      end
   case 'double'
      cflag = 0;
      if dflag==0 % think xdata
         argcell = [argcell;{arg}];
         cnt = cnt + 1;
         dflag = 1;
      elseif dflag==1 % think ydata
         argcell = [argcell;{arg}];
         ydataloc = [ydataloc;cnt];
         cnt = cnt + 1;
         dflag = 0;
      end
   otherwise
      if isuncertain(arg)
         error('Cannot plot uncertain matrices or systems');
      else
         error('Cannot plot this type of data');
      end
   end
end
xmin = inf;
xmax = -inf;
for i=1:length(ydataloc)
   xmin = min([xmin min(argcell{ydataloc(i)-1})]);
   xmax = max([xmax max(argcell{ydataloc(i)-1})]);
end
for i=1:length(ydataloc)
   if length(argcell{ydataloc(i)})==1
      argcell{ydataloc(i)} = [argcell{ydataloc(i)} argcell{ydataloc(i)}];
      argcell{ydataloc(i)-1} = [xmin xmax];
   end
end
      
switch plottype
case 'iv,d'
   h = plot(argcell{:});
case 'iv,m'
   for i=1:length(ydataloc)
      argcell{ydataloc(i)} = abs(argcell{ydataloc(i)});
   end
   h = plot(argcell{:});
case 'iv,lm'
   for i=1:length(ydataloc)
      argcell{ydataloc(i)} = abs(argcell{ydataloc(i)});
   end
   h = semilogy(argcell{:});
case 'iv,p'
   for i=1:length(ydataloc)
      argcell{ydataloc(i)} = (180/pi)*angle(argcell{ydataloc(i)});
   end
   h = plot(argcell{:});
case 'liv,d'
   h = semilogx(argcell{:});
case 'liv,ld'
   h = loglog(argcell{:});
case 'liv,m'
   for i=1:length(ydataloc)
      argcell{ydataloc(i)} = abs(argcell{ydataloc(i)});
   end
   h = semilogx(argcell{:});
case 'liv,lm'
   for i=1:length(ydataloc)
      argcell{ydataloc(i)} = abs(argcell{ydataloc(i)});
   end
   h = loglog(argcell{:});
case 'liv,p'
   for i=1:length(ydataloc)
      argcell{ydataloc(i)} = (180/pi)*angle(argcell{ydataloc(i)});
   end
   h = semilogx(argcell{:});
case {'nyq'}
   for i=1:length(ydataloc)
      %x-data, real part
      argcell{ydataloc(i)-1} = real(argcell{ydataloc(i)});
      argcell{ydataloc(i)}   = imag(argcell{ydataloc(i)});
   end
   h = plot(argcell{:});
case {'ri'}
   for i=1:length(ydataloc)
      %x-data, real part
      argcell{ydataloc(i)-1} = real(argcell{ydataloc(i)});
      %y-data, imag part
      argcell{ydataloc(i)}   = imag(argcell{ydataloc(i)});
   end
   h = plot(argcell{:});
case {'nic'}
   for i=1:length(ydataloc)
      %x-data, imag part
      argcell{ydataloc(i)-1} = 360/(2*pi)*negangle(argcell{ydataloc(i)});
      %y-data, real part
      argcell{ydataloc(i)}   = 20*log10(abs(argcell{ydataloc(i)}));
   end
   h = plot(argcell{:});
case 'bode'
   subplot(2,1,1)
   magcell = argcell;
   for i=1:length(ydataloc)
      magcell{ydataloc(i)} = abs(magcell{ydataloc(i)});
   end
   hm = loglog(magcell{:});
   subplot(2,1,2)
   for i=1:length(ydataloc)
      argcell{ydataloc(i)} = (180/pi)*angle(argcell{ydataloc(i)});
   end
   hp = semilogx(argcell{:});
   h = [hm;hp];
otherwise
   error('invalid plot type');
end
      
if nargout==1
   han = h;
end
