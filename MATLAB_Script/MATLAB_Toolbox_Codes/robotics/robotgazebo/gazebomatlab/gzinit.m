function gzinit(varargin)
%GZINIT Initialize connection setting for Gazebo Co-Simulation MATLAB
%Interface
%   GZINIT initializes connection settings and checks connectivity with
%   Gazebo plugin running on localhost and port 14581. This syntax sets
%   response timeout to 1 second.
%
%   GZINIT(HostIP) initializes connection settings and checks connectivity
%   with Gazebo plugin at the hostname or IP address specified by HostIP.
%   This syntax uses 14581 as the default port number and sets response
%   timeout to 1 second.
%
%   GZINIT(HostIP,HostPort) initializes connection settings and checks
%   connectivity with Gazebo plugin at the specified HostIP and the port
%   number specified by HostPort. The response timeout is set to 1 second.
%
%   GZINIT(HostIP,HostPort,Timeout) initializes connection settings and
%   checks connectivity with Gazebo plugin at the specified HostIP and
%   HostPort. The response timeout is specified by Timeout in seconds.
%
%   Note:
%       The HostIP is the hostname or IP address of the Gazebo plugin
%       installed machine. The HostPort is the 'portNumber' mentioned in
%       the Gazebo '.world' file. The Timeout is the period of waiting for
%       a response between client and server. Timeout value must be set
%       higher for poor network connectivity.
%
%   Example:
%      % Launch 'multiSensorPluginTest.world' from installed Gazebo server
%      % plugin before using following example
%
%      % Initialize connection setting with IP address '172.18.250.191' ,
%      % default HostPort and Timeout
%      GZINIT("172.18.250.191");
%
%      % Initialize connection setting with IP address '172.18.250.191' ,
%      % '14581' HostPort and default Timeout
%      GZINIT("172.18.250.191",14581);
%
%      % Initialize connection setting with IP address '172.18.250.191' ,
%      % '14581' HostPort and 10 second Timeout
%      GZINIT("172.18.250.191",14581,10);
%
%   See also gzworld, gzmodel, gzlink, gzjoint

%   Copyright 2020 The MathWorks, Inc.

    narginchk(0,3);

    profileStore = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceStore;

    if nargin > 0

        profile = profileStore.getProfile();
        profile.MasterUseDefault = false;
        validateattributes(varargin{1}, {'string', 'char'}, {'scalartext'}, 'gzinit', 'HostIP');
        profile.MasterHost = convertStringsToChars(varargin{1});
        if(nargin > 1)
            validateattributes(varargin{2}, {'numeric'}, ...
                               {'integer', 'positive', 'scalar', '<=', 65536}, 'gzinit', 'HostPort');
            profile.MasterPort = varargin{2};
            if(nargin > 2)
                validateattributes(varargin{3}, {'numeric'}, ...
                                   {'scalar', 'positive'}, 'gzinit', 'Timeout');
                profile.SimulationTimeout = varargin{3};
            else
                % Default time-out
                profile.SimulationTimeout = profile.getDefaultSimulationTimeout;
            end
        else
            % Default plugin port number
            profile.MasterPort = profile.getDefaultPort;
            % Default time-out
            profile.SimulationTimeout = profile.getDefaultSimulationTimeout;
        end

    else
        % Default connection settings
        profile = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceProfile;
    end

    profileStore.setProfile(profile)
    profileStore.updateStore();

    % Test connection
    GazeboClient = robotics.internal.GazeboClient;
    GazeboClient.connect(profile.MasterHost, profile.MasterPort, profile.SimulationTimeout*1000);
    % shutdown client
    GazeboClient.shutdown();

end
