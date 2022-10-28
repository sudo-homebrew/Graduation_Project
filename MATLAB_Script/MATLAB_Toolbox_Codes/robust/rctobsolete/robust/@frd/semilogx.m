% SEMILOGX Semi-log scale plot for FRDs.
%    SEMILOGX(...) is the same as PLOT(...), except a
%    logarithmic (base 10) scale is used for the X-axis.
%  
%EXAMPLE: Generate a frequency response of a first order system and plot it.
%  
% x = tf(1,[1 2]);
% om = logspace(-2,2,80);
% xg = frd(x,om);
% semilogx(abs(xg))
%
% See also: BODE, BODEMAG, PLOT, LOGLOG, SEMILOGY
function han = semilogx(varargin)

%   Copyright 2003-2011 The MathWorks, Inc.

try
  h = uplot('liv,d',varargin{:});
catch ME
  throw(ME)
end
      
if nargout==1
   han = h;
end
