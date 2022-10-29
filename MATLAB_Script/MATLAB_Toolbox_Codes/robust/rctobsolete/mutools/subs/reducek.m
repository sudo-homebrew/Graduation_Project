% function out = reducek(in,ord,flg)

%   Copyright 1991-2004 MUSYN Inc. and The MathWorks, Inc.

function out = reducek(in,ord,flg)

if nargin == 2
    flg = 'cf';
end

if any(flg=='b')
    inb = sysbal(in);
    out = strunc(inb,ord);
else
    [lcg,sig,rcf] = sncfbal(in);
    out = cf2sys(strunc(rcf,ord));
end