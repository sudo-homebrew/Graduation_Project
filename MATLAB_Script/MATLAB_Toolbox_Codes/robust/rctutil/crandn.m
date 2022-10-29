function out = crandn(varargin)
%

% Copyright 2004 The MathWorks, Inc.
try
   out = complex(randn(varargin{:}),randn(varargin{:}))/sqrt(2);
catch ME
   throw(ME);
end