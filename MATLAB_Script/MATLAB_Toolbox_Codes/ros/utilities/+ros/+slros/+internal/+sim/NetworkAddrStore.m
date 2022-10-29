classdef NetworkAddrStore < robotics.utils.internal.PreferenceStore
%This class is for internal use only. It may be removed in the future.

%  NetworkAddrStore is an interface for managing ROS network address
%  profiles (currently, only a single "Standard" profile is supported).
%  This information is stored in MATLAB Preferences, and  applies to
%  all Simulink models.
%
%  Methods:
%    updateStore  - Update MATLAB preferences with modified information
%
%    getProfile   - Get Standard profile (if it exists) else return default
%    hasProfile   - Return true if a Standard profile exists
%    setProfile   - Set the Standard profile
%    clearProfile - Remove profile from MATLAB preferences
%
%  See also: sim.ROSMaster, sim.NetworkAddrProfile

%   Copyright 2014-2021 The MathWorks, Inc.

    properties(Constant)
        PreferenceGroup = ros.slros.internal.Constants.PreferenceGroup
        PreferenceName = 'ROS_NetworkAddress_Profiles'
        ProfileName = 'Standard'
        ProfileCorruptErrorMsgID = 'ros:slros:netaddr:CorruptedProfile'
    end

    methods (Access=protected)
        function profile = getDefaultProfile(~)
        %getDefaultProfile Returns a default constructed profile to
        %fill
            profile = ros.slros.internal.sim.NetworkAddrProfile();
        end
    end


    methods (Static,Hidden)
        % Used for testing
        function removePrefs()
            import ros.slros.internal.sim.NetworkAddrStore
            robotics.utils.internal.PreferenceStore.removePrefs(...
                NetworkAddrStore.PreferenceGroup, NetworkAddrStore.PreferenceName...
                );
        end

    end

end
