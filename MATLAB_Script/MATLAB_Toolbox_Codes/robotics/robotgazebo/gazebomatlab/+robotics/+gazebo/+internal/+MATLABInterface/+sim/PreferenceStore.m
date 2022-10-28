classdef PreferenceStore < handle
%This class is for internal use only. It may be removed in the future.

%  PreferenceStore is an interface for managing preference profiles
%  (currently, only a single "Standard" profile is supported). This
%  information is stored in MATLAB Preferences, and  applies to all
%  Simulink models.
%
%  Methods:
%    updateStore  - Update MATLAB preferences with modified information
%
%    getProfile   - Get Standard profile (if it exists) else return default
%    hasProfile   - Return true if a Standard profile exists
%    setProfile   - Set the Standard profile
%    clearProfile - Remove profile from MATLAB preferences
%
%  See also: robotics.gazebo.internal.MATLABInterface.sim.PreferenceProfile

%   Copyright 2020 The MathWorks, Inc.

    properties(Abstract, Constant)
        PreferenceGroup
        PreferenceName
        ProfileName
        ProfileCorruptErrorMsgID
    end


    properties(Access=private)
        ProfileMap
        DataIsModified = false
    end


    methods

        % In 15a, The user can have a single profile, or not at all.
        function obj = PreferenceStore()
            obj.ProfileMap = obj.loadPrefs();
            assert( isa(obj.ProfileMap, 'containers.Map') );
            obj.DataIsModified = false;
        end


        function profile = getProfile(obj)
            if obj.ProfileMap.isKey(obj.ProfileName)
                profile = obj.ProfileMap(obj.ProfileName);
                assert(isa(profile, 'robotics.gazebo.internal.MATLABInterface.sim.PreferenceProfile'));
            else
                profile = obj.getDefaultProfile();
                profile.ProfileName = obj.ProfileName;
            end
        end


        function out= hasProfile(obj)
            out = obj.ProfileMap.isKey(obj.ProfileName);
        end


        function setProfile(obj, profile)
            validateattributes(profile, {'robotics.gazebo.internal.MATLABInterface.sim.PreferenceProfile'}, {'scalar'});
            profile.ProfileName = obj.ProfileName;
            obj.ProfileMap(obj.ProfileName) = profile;
            obj.DataIsModified = true;
        end


        function clearProfile(obj)
            if obj.ProfileMap.isKey(obj.ProfileName)
                obj.ProfileMap.remove(obj.ProfileName);
                obj.DataIsModified = true;
            end
        end


        function updateStore(obj)
            if obj.DataIsModified
                obj.savePrefs(obj.ProfileMap);
            end
        end

    end

    %%
    methods (Abstract, Access = protected)
        profile = getDefaultProfile(obj)
    end

    %%

    methods (Access=private)

        % Don't save the containers.Map with the PreferenceStore objects

        function savePrefs(obj, prefs)
            validateattributes(prefs, {'containers.Map'}, {});
            % Save the map as an array of raw structs
            profileNames = keys(prefs);
            profileStructs = cell(size(profileNames));
            for i=1:numel(profileNames)
                profileStructs{i} = prefs(profileNames{i}).getPropsStruct;
            end
            setpref(obj.PreferenceGroup, obj.PreferenceName, profileStructs);
        end


        function prefs = loadPrefs(obj)
            if ispref(obj.PreferenceGroup, obj.PreferenceName)
                profileStructs = getpref(obj.PreferenceGroup, obj.PreferenceName);
            else
                profileStructs = {};
            end

            if ~(iscell(profileStructs) && all(cellfun(@isstruct, profileStructs)))
                % The stored variable got corrupted somehow
                warning(message(obj.ProfileCorruptErrorMsgID));
                % Overwrite saved prefs with a null entry, so this warning
                % doesn't show up again
                obj.savePrefs(containers.Map);
                profileStructs = [];
            end

            % convert the array of structs to a ProfileName -> NetworkAddrProfile map
            prefs = containers.Map;
            for i=1:numel(profileStructs)
                s = profileStructs{i};
                prefs(s.ProfileName) = ...
                    robotics.gazebo.internal.MATLABInterface.sim.PreferenceProfile.setPropsStruct(s, obj.getDefaultProfile());
            end
        end

    end

    methods (Static, Hidden)

        % Used for testing
        function removePrefs(group, name)
            if ispref(group, name)
                rmpref(group, name);
            end
        end

    end

end
