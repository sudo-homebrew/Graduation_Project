classdef GazeboPreferenceStore < robotics.gazebo.internal.MATLABInterface.sim.PreferenceStore
%This class is for internal use only. It may be removed in the future.

%GAZEBOPREFERENCESTORE stores Gazebo preference in MATLAB preference by
%specifying the unique combination of preference group and name values

%   Copyright 2020 The MathWorks, Inc.

    properties(Constant)
        %PreferenceGroup The preference group in MATLAB preference setting
        PreferenceGroup = 'Robotics_System_Toolbox'

        %PreferenceName The sub-preference group in MATLAB preference setting
        PreferenceName = 'Gazebo_CoSim_NetworkAddress_Profiles'

        %ProfileName Distinguish between different profiles. Currently only
        %a single Standard profile is supported
        ProfileName = 'Standard'

        %ProfileCorruptErrorMsgID Error ID to report when the profile is
        %corrupted
        ProfileCorruptErrorMsgID = 'robotics:robotslgazebo:preference:CorruptedProfile'
    end

    methods (Access=protected)
        function profile = getDefaultProfile(~)
        %getDefaultProfile Returns a default constructed profile to
        %fill
            profile = robotics.gazebo.internal.MATLABInterface.sim.GazeboPreferenceProfile;
        end
    end

end
