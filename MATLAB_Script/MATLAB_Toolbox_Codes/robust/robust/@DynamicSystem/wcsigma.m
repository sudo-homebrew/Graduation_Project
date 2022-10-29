function wcsigma(sys,varargin)
%WCSIGMA   See WCSIGMAPLOT.
%
%   See also WCSIGMAPLOT.

%   Copyright 2003-2020 The MathWorks, Inc.
try
   wcsigmaplot(sys,varargin{:})
catch ME
   throw(ME)
end