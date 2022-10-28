%  LOGLOG Log-log scale plot for FRDs.
%     LOGLOG(...) is the same as PLOT(...), except logarithmic
%     scales are used for both the X- and Y- axes.
%  
%EXAMPLE: Generate a frequency response of a first order system and plot it.
%  
% x = tf(1,[1 2]);
% om = logspace(-2,2,80);
% xg = frd(x,om);
% loglog(abs(xg))
%
% See also: BODE, BODEMAG, PLOT, SEMILOGX, SEMILOGY

function han = loglog(varargin)

%   Copyright 2003-2011 The MathWorks, Inc.

try
  h = uplot('liv,ld',varargin{:});
catch ME
   throw(ME)
end
      
if nargout==1
   han = h;
end
