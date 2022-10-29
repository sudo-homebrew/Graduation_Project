function M = intervalmean(I)
% I is a 1-by-2 array, representing an interval.  The left endpoint must be
% >=0 and the right endpoint <=INF. The returned value is always nonzero
% and finite.

%   Copyright 2010-2018 The MathWorks, Inc.

% Use y = atan(log(x)/10) to map unbounded intervals into bounded ones 
% before taking mean. This ensures continuity with respect to the end 
% points and M is close to the geometric mean when I spans a small number
% of decades.
fact = 10;  % controls frequency band [1/f,f] where M close to geometric mean
M = exp(fact*tan(mean(atan(log(I)/fact))));
