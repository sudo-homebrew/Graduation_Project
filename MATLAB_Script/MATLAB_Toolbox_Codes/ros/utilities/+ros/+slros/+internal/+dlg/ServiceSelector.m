classdef ServiceSelector < ros.internal.dlg.ServiceSelector
%This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

    properties
        ROSMaster
    end

    methods
        function obj = ServiceSelector()
            obj.ROSMaster = ros.slros.internal.sim.ROSMaster();
            obj.ROSMaster.verifyReachable();
        end
    end

    methods (Access = protected)
        function setServiceListAndTypes(obj)
            obj.ROSMaster.verifyReachable();
            [obj.ServiceList, obj.ServiceTypeList] = obj.ROSMaster.getServiceNamesTypes;

            if numel(obj.ServiceList) < 1
                error(message('ros:slros:svcselector:NoServicesAvailable', ...
                              obj.ROSMaster.MasterURI));
            end
        end
    end
end
