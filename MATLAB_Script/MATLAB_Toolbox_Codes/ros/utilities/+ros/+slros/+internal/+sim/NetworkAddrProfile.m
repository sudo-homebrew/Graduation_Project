classdef NetworkAddrProfile < robotics.utils.internal.PreferenceProfile
%This class is for internal use only. It may be removed in the future.

%  NetworkAddrProfile is a data class that specifies a single network
%  address profile (information about ROS Master and Node host).
%
%  Static methods:
%   getDefaultMasterHost
%   getDefaultNodeHost
%
%  See also: sim.ROSMaster, sim.NetworkAddrStore

%   Copyright 2014-2021 The MathWorks, Inc.

    properties
        MasterUseDefault = true
        MasterHost = 'localhost'
        MasterPort = 11311

        NodeUseDefault = true
        NodeHost = ''
        
        %DomainID ROS 2 domain ID
        DomainID = '';

        %RMWImplementation ROS 2 Middleware Implementation
        RMWImplementation = '';
    end

    methods
        function obj = set.MasterUseDefault(obj, value)
            validateattributes(value, {'logical'}, {'scalar'});
            obj.MasterUseDefault = value;
        end

        function obj = set.MasterHost(obj, name)
            validateattributes(name, {'char'}, {});
            obj.MasterHost = name;
        end

        function obj = set.MasterPort(obj, port)
            validateattributes(port, {'numeric'}, {'scalar'});
            obj.MasterPort = port;
        end

        function obj = set.NodeUseDefault(obj, value)
            validateattributes(value, {'logical'}, {'scalar'});
            obj.NodeUseDefault = value;
        end

        function obj = set.NodeHost(obj, name)
            validateattributes(name, {'char'}, {});
            obj.NodeHost = name;
        end

        function obj = set.DomainID(obj, value)
            if isempty(value)
                obj.DomainID = '';
            else
                validateattributes(value, {'numeric'}, {'scalar', 'integer', 'nonnegative'});
                obj.DomainID = value;
            end
        end
        
        function out = getDefaultDomainID(~)
            out = ros.internal.utilities.getDefaultDomainID;
        end

        function obj = set.RMWImplementation(obj, rmwImpl)
            if isempty(rmwImpl)
                obj.RMWImplementation = '';
            else
                validateattributes(rmwImpl, {'char', 'string'},{'scalartext'});
                obj.RMWImplementation = rmwImpl;
            end
        end
        
        function out = getDefaultRMWImplementation(~)
            out = ros.internal.utilities.getDefaultRMWImplementation;
        end
    end

    methods(Static)
        function out = getDefaultMasterHost()
            out = 'localhost';
        end

        function out = getDefaultNodeHost()
            addrs = ros.internal.Net.getAllIPv4Addresses;
            if numel(addrs) > 0
                out = addrs(1).ipAddress;
            else
                out = '';
            end
        end
    end

end
