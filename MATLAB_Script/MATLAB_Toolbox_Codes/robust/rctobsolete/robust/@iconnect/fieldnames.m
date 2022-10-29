function n = fieldnames(a)
%

% Copyright 2003-2004 The MathWorks, Inc.

tmp = pvget(a,'PropNames');
n = tmp.GPropNames;