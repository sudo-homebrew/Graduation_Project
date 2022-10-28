classdef GazeboPreferenceProfile < robotics.gazebo.internal.MATLABInterface.sim.PreferenceProfile
%This class is for internal use only. It may be removed in the future.

%  GazeboPreferenceProfile is a data class that specifies interface to
%  store Gazebo network setting in the MATLAB preference
%
%  See also: robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceStore

%   Copyright 2019-2020 The MathWorks, Inc.

    properties
        %MasterUseDefault flags whether the profile is using default
        %setting or not
        MasterUseDefault = true

        %MasterHost keeps track of the user setting on the Gazebo host name
        MasterHost = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceProfile.getDefaultHost

        %MasterPort keeps track of the user setting on the Gazebo host port
        MasterPort = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceProfile.getDefaultPort

        %SimulationTimeStep sets the time step used to step Gazebo simulator
        SimulationTimeStep = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceProfile.getDefaultSimulationTimeStep

        %SimulationTimeout sets the maximum timeout for waiting for Gazebo
        SimulationTimeout = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceProfile.getDefaultSimulationTimeout
    end

    methods
        function obj = set.MasterUseDefault(obj, value)
            validateattributes(value, {'logical'}, {'scalar'}, 'GazeboPreference', 'GazeboUseDefault');
            obj.MasterUseDefault = value;
        end

        function obj = set.MasterHost(obj, name)
            validateattributes(name, {'char'}, {'scalartext'}, 'GazeboPreference', 'GazeboHost');
            obj.MasterHost = name;
        end

        function obj = set.MasterPort(obj, port)
            validateattributes(port, {'numeric'}, {'scalar'}, 'GazeboPreference', 'GazeboPort');
            obj.MasterPort = port;
        end

        function obj = set.SimulationTimeStep(obj, timestep)
            validateattributes(timestep, {'numeric'}, {'scalar', 'nonnan', '>', 0}, 'GazeboPreference', 'SimulationTimeStep');
            obj.SimulationTimeStep = timestep;
        end

        function obj = set.SimulationTimeout(obj, timeout)
        %set.SimulationTimeout also validates the simulation timeout
        %range

        % simulation time out must be greater or equal to 0.001 as the
        % minimum time out we support is 1 ms
        % simulation time out must be less than 60 seconds, as the
        % maximum time out we support is 65535 ms
            validateattributes(timeout, {'numeric'}, {'scalar', 'nonnan', '>=', 0.001, '<=', 600}, 'GazeboPreference', 'SimulationTimeout');
            obj.SimulationTimeout = timeout;
        end

        function host = get.MasterHost(obj)
        %get.MasterHost returns the default value when MasterUseDefault
        %is true, otherwise return the stored value
            if obj.MasterUseDefault
                host = obj.getDefaultHost();
            else
                host = obj.MasterHost;
            end
        end

        function port = get.MasterPort(obj)
        %get.MasterPort returns the default value when MasterUseDefault
        %is true, otherwise return the stored value
            if obj.MasterUseDefault
                port = obj.getDefaultPort();
            else
                port = obj.MasterPort;
            end
        end

        function s = getPropsStruct(obj)
        %getPropsStruct override the default getPropsStruct method to
        %save the actual master host and port values

        % since the getters for master host and port automatically
        % adjust the output based on master use default flag, during
        % the save process, the default implementation cannot read the
        % actual value stored in this profile if MasterUseDefault is
        % set to true
            actualUseDefault = obj.MasterUseDefault;
            obj.MasterUseDefault = false;

            % call base class method to copy the properties
            s = getPropsStruct@robotics.gazebo.internal.MATLABInterface.sim.PreferenceProfile(obj);

            % Copy the actual MasterUseDefault to struct
            s.MasterUseDefault = actualUseDefault;
        end
    end

    methods (Static)
        function host = getDefaultHost()
        %getDefaultHost Single source the default Gazebo plugin server
        %host name
            host = 'localhost';
        end

        function port = getDefaultPort()
        %getDefaultHost Single source the default Gazebo plugin server
        %host port
            port = 14581;
        end

        function timeStep = getDefaultSimulationTimeStep()
        %getDefaultHost Single source the default Gazebo simulation
        %time step

            timeStep = 0.001;
        end

        function timeout = getDefaultSimulationTimeout()
        %getDefaultSimulationTimeout Single source the default Gazebo simulation
        %timeout

            timeout = 1;
        end
    end

end
