function [out, ia, ic] = uniquetolcg(in, tol)
% This is an internal function and may be removed or modified in a future
% release.

% Copyright 2020 The MathWorks, Inc.

% Code-generation compatible implementation of uniquetol when tolerance is
% absolute.
%#codegen
if coder.target('MATLAB')
    % In MATLAB, simply call unique tol with correct DataScale.
    [out,ia,ic] = uniquetol(in,tol,'DataScale',1);
else
    % Use a for-loop in code generation
    n = numel(in);
    if n > 0
        [inSorted,sortid] = sort(in);
        invsortid = sortid;
        invsortid(invsortid) = 1:n;
        out = inSorted(1);
        ic = zeros(n,1);
        ic(1) = 1;
        ia = 1;
        k = 1;
        count = 1;
        for i = 2:n
            if abs(inSorted(k) - inSorted(i)) > tol
                out = [out;inSorted(i)];
                ia = [ia;i];
                k = i;
                count = count + 1;
            end
            ic(i) = count;
        end
        ia = sortid(ia);
        ic = ic(invsortid);
    else
        out = zeros(0,1,'like',in);
        ia = zeros(0,1);
        ic = zeros(0,1);
    end
end
end