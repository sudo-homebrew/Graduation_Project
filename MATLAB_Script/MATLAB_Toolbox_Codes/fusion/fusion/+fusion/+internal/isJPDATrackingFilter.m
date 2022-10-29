function tf = isJPDATrackingFilter(filter)
%isJPDATrackingFilter  Checks if filter is a JPDA tracking filter
% This is an internal function. It may be removed or modified at any time
%
% tf = fusion.internal.isJPDATrackingFilter(filter) returns true if filter is
% one of the tracking filters that the JPDA tracker can work with. It returns
% false otherwise.

% Copyright 2017-2018 The MathWorks, Inc.

%#codegen
tf = isa(filter,'matlabshared.tracking.internal.AbstractJPDAFilter') && ...
    isa(filter,'matlabshared.tracking.internal.AbstractTrackingFilter');