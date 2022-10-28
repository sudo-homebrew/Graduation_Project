function  error( msgID, varargin )
%This class is for internal use only. It may be removed in the future.

%ERROR Internal error message function with codegen compatibility. Supports
%   up to two custom argument holes (as defined in the message catalog). 
%   varargin is either one or two char array(s).
%   NOTE this method is NOT for top-level function codegen.

%#codegen
    
%   Copyright 2016 The MathWorks, Inc.


rb = 'robotics';
component = 'robotmanip';
narginchk(1,4);
if nargin == 1
    if coder.target('MATLAB')
        error(message([rb ':' component ':' msgID]));
    else
        coder.internal.error([rb ':' component ':' msgID]);
    end
elseif nargin == 2
    if coder.target('MATLAB')
        error(message([rb ':' component ':' msgID], varargin{1}));
    else
        mid = coder.const(msgID);
        coder.internal.error([rb ':' component ':' mid], varargin{1});
    end
elseif nargin == 3
    if coder.target('MATLAB')
        error(message([rb ':' component ':' msgID], varargin{1}, varargin{2}));
    else
        mid = coder.const(msgID);
        coder.internal.error([rb ':' component ':' mid], varargin{1}, varargin{2});
    end
else
    if coder.target('MATLAB')
        error(message([rb ':' component ':' msgID], varargin{1}, varargin{2}, varargin{3}));
    else
        mid = coder.const(msgID);
        coder.internal.error([rb ':' component ':' mid], varargin{1}, varargin{2}, varargin{3});
    end
end
    
    
end

