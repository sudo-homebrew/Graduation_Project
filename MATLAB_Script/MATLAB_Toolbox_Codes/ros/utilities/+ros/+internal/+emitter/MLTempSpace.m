classdef MLTempSpace < handle
%This class is for internal use only. It may be removed in the future.

%MLTempSpace provides for any of the nodes to add any temp values

%Copyright 2019 The MathWorks, Inc.


    properties (Constant)
        TEMPSPACENAME = 'TempSpace__'
    end

    properties
        TempSpace = struct();
    end

end
