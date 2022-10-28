classdef ServiceTypeSelector < ros.internal.dlg.ServiceTypeSelector
%This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

    methods (Access = protected)
        function setServicelist(obj)
            obj.ServiceList = rostype.getServiceList;
            assert(numel(obj.ServiceList) > 0, 'Expected non-empty service list');
        end
    end
end
