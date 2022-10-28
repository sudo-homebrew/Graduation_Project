classdef PreferenceProfile
%This class is for internal use only. It may be removed in the future.

%  PreferenceProfile is a data class that specifies interface to store in
%  the preference. User should inherit from this class and add their data
%  as properties

%   Copyright 2020 The MathWorks, Inc.

    properties
        ProfileName = ''
    end

    methods
        function obj = set.ProfileName(obj, name)
            validateattributes(name, {'char'}, {});
            obj.ProfileName = name;
        end
    end

    methods
        % These are used for LOAD and SAVE. We rely on these instead of just
        % default LOAD/SAVE behavior as the information gets saved to
        % MATLAB Preferences. If ROSTbx is not on the path for any reason,
        % this leads to warnings at MATLAB load time & whenever any MATLAB
        % preference is accessed
        function s = getPropsStruct(obj)
            props = properties(obj);
                s = struct();
                for i=1:numel(props)
                    s.(props{i}) = obj.(props{i});
                end
            end
    end

    methods(Static)
        function obj = setPropsStruct(s, emptyProfile)
            obj = emptyProfile;
            props = properties(obj);
                for i=1:numel(props)
                    if isfield(s, props{i})
                        obj.(props{i}) = s.(props{i});
                    end
                end
            end
    end

end
