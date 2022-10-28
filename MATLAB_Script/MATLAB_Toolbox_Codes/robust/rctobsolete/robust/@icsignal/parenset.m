%PARENSET Low level set command for ICSIGNAL objects. Not supported.

% Copyright 2003-2004 The MathWorks, Inc.
function m = parenset(m,L,RHS)

error(['Parenthesis () assignment not supported for ' class(m) 'objects.'])
