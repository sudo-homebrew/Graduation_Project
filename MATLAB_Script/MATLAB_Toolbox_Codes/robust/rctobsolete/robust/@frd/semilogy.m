% SEMILOGY Semi-log scale plot for FRDs.
%    SEMILOGY(...) is the same as PLOT(...), except a
%    logarithmic (base 10) scale is used for the Y-axis.
%  
%EXAMPLE: Generate a frequency response of a first order system and plot it.
%  
% x = tf(1,[1 2]);
% om = logspace(-2,2,80);
% xg = frd(x,om);
% semilogy(abs(xg))
%
% See also: BODE, BODEMAG, PLOT, LOGLOG, SEMILOGX

function han = semilogy(varargin)

%   Copyright 2003-2011 The MathWorks, Inc.

try
  h = uplot('iv,lm',varargin{:});
catch ME
  throw(ME)
end
      
if nargout==1
   han = h;
end
