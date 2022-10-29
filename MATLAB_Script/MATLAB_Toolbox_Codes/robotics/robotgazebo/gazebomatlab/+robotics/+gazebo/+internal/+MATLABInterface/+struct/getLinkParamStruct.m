function Link = getLinkParamStruct(linkName,p)
%This function is for internal use only. It may be removed in the future.
%
% This function creates MATLAB struct based on linkname and parameter name
% and value.

%   Copyright 2020 The MathWorks, Inc.

    Link.name = linkName;
    % check input 'Position' value and add into struct
    if(~isempty(p.Results.Position))
        Link.pose.position.x = double(p.Results.Position(1));
        Link.pose.position.y = double(p.Results.Position(2));
        Link.pose.position.z = double(p.Results.Position(3));
    end
    % check input 'Orientation' value and add into struct
    if(~isempty(p.Results.Orientation))
        Link.pose.orientation.w = double(p.Results.Orientation(1));
        Link.pose.orientation.x = double(p.Results.Orientation(2));
        Link.pose.orientation.y = double(p.Results.Orientation(3));
        Link.pose.orientation.z = double(p.Results.Orientation(4));
    end
    % check input 'Mass' value and add into struct
    if(~isempty(p.Results.Mass))
        Link.inertial.mass = double(p.Results.Mass);
    end
    % check input 'ProductOfInertia' value and add into struct
    if(~isempty(p.Results.ProductOfInertia))
        Link.inertial.ixy = double(p.Results.ProductOfInertia(1));
        Link.inertial.ixz = double(p.Results.ProductOfInertia(2));
        Link.inertial.iyz = double(p.Results.ProductOfInertia(3));
    end
    % check input 'PrincipalMoments' value and add into struct
    if(~isempty(p.Results.PrincipalMoments))
        Link.inertial.ixx = double(p.Results.PrincipalMoments(1));
        Link.inertial.iyy = double(p.Results.PrincipalMoments(2));
        Link.inertial.izz = double( p.Results.PrincipalMoments(3));
    end
    % check input 'IsStatic' value and add into struct
    if(~isempty(p.Results.IsStatic))
        if( strcmp(p.Results.IsStatic,"on"))
            Link.is_static = true;
        else
            Link.is_static = false;
        end
    end
    % check input 'Canonical' value and add into struct
    if(~isempty(p.Results.Canonical))
        if( strcmp(p.Results.Canonical,"on"))
            Link.canonical = true;
        else
            Link.canonical = false;
        end
    end
    % check input 'SelfCollide' value and add into struct
    if(~isempty(p.Results.SelfCollide))
        if( strcmp(p.Results.SelfCollide,"on"))
            Link.self_collide = true;
        else
            Link.self_collide = false;
        end
    end
    % check input 'Gravity' value and add into struct
    if(~isempty(p.Results.Gravity))
        if( strcmp(p.Results.Gravity,"on"))
            Link.gravity = true;
        else
            Link.gravity = false;
        end
    end
    % check input 'Kinematic' value and add into struct
    if(~isempty(p.Results.Kinematic))
        if( strcmp(p.Results.Kinematic,"on"))
            Link.kinematic = true;
        else
            Link.kinematic = false;
        end
    end
    % check input 'EnableWind' value and add into struct
    if(~isempty(p.Results.EnableWind))
        if( strcmp(p.Results.EnableWind,"on"))
            Link.enabled_wind = true;
        else
            Link.enabled_wind = false;
        end
    end

end
