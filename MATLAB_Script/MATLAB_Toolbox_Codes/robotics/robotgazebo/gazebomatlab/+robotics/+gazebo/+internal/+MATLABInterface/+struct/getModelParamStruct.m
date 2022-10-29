function modelStruct = getModelParamStruct(modelStruct,p)
%This function is for internal use only. It may be removed in the future.
%
% This function creates MATLAB struct based on modelname and parameter name
% and value.

%   Copyright 2020 The MathWorks, Inc.

    Pose = {};
    % check input 'Position' value and add into struct
    if(~isempty(p.Results.Position))
        Pose.position.x = double(p.Results.Position(1));
        Pose.position.y = double(p.Results.Position(2));
        Pose.position.z = double(p.Results.Position(3));
    end
    % check input 'Orientation' value and add into struct
    if(~isempty(p.Results.Orientation))
        Pose.orientation.w = double(p.Results.Orientation(1));
        Pose.orientation.x = double(p.Results.Orientation(2));
        Pose.orientation.y = double(p.Results.Orientation(3));
        Pose.orientation.z = double(p.Results.Orientation(4));
    end
    % check 'Pose' struct into final struct
    if(~isempty(Pose))
        modelStruct.pose = Pose;
    end
    % check input 'EnableWind' value and add into struct
    if(~isempty(p.Results.EnableWind))
        if( strcmp(p.Results.EnableWind,"on"))
            modelStruct.enable_wind = true;
        else
            modelStruct.enable_wind = false;
        end
    end
    % check input 'SelfCollide' value and add into struct
    if(~isempty(p.Results.SelfCollide))
        if( strcmp(p.Results.SelfCollide,"on"))
            modelStruct.self_collide = true;
        else
            modelStruct.self_collide = false;
        end
    end
    % check input 'IsStatic' value and add into struct
    if(~isempty(p.Results.IsStatic))
        if( strcmp(p.Results.IsStatic,"on"))
            modelStruct.is_static = true;
        else
            modelStruct.is_static = false;
        end
    end

end
