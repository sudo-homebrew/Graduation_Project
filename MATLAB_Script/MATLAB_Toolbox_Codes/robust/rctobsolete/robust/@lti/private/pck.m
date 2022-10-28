function varargout=pck(varargin)
% ROBUST/LTI/PRIVATE/PCK.M
% 

% Copyright 2003-2004 The MathWorks, Inc.

varargout=cell(1,nargout);
if nargout,
   [varargout{:}] = ss(varargin{:});
else
   pck(varargin{:});
end
% 
% ----- End of ROBUST/LTI/PRIVATE/PCK.M ---- 15-Nov-2002 %
