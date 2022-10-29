%UMINUS Unitary minus for ICSIGNAL objects.

% Copyright 2003-2004 The MathWorks, Inc.
function out = uminus(A)
out = A;
out.System = -out.System;
