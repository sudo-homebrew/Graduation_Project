function varargout = dksynperf(sys,varargin)
%DKSYNPERF   Robust H-infinity performance optimized by DKSYN.
%
%   This function is obsolete, use MUSYNPERF instead.
%
%   See also MUSYNPERF.

%   Copyright 2003-2016 The MathWorks, Inc.
narginchk(1,3)
try
   [varargout{1:nargout}] = musynperf(sys,varargin{:});
catch ME
   throw(ME)
end