%PLOT   Plot method for FRD objects.
%  
%EXAMPLE: Generate a frequency response of a first order system and plot it.
%  
% x = tf(1,[1 2]);
% om = logspace(-2,2,80);
% xg = frd(x,om);
% plot(abs(xg))
%
% See also: BODE, BODEMAG, LOGLOG, SEMILOGX, SEMILOGY
 
function han = plot(varargin)

%   Copyright 2003-2011 The MathWorks, Inc.

try
  h = uplot('iv,d',varargin{:});
catch ME
  throw(ME)
end
      
if nargout==1
   han = h;
end
