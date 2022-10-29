%MINUS  Subtraction for ICSIGNAL objects

function out = minus(A,B)
% Copyright 2003-2004 The MathWorks, Inc.

if nargin==2
    [a2n,b2n,listn,dimn] = ...
         icsigbin(A.SignalList,A.SignalDim,B.SignalList,B.SignalDim);
    sysout = (A.System*a2n) - (B.System*b2n);
    out = icsignal(sysout,listn,dimn);
end