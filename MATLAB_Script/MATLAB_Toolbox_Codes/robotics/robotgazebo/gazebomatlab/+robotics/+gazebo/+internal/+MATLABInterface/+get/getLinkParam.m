function [param, paramString] = getLinkParam(modelName, linkName, varargin)
%This function is for internal use only. It may be removed in the future.
%
% This function connects with Gazebo and request model parameter value
% based on modelname and linkname. Further, creates MATLAB struct and
% string of output as per received model-link parameters.

%   Copyright 2020 The MathWorks, Inc.

% output parameter struct
    param.modelname = modelName;
    param.linkname = linkName;
    % output parameter string map. Contains parameter name and value in string form
    paramString = containers.Map('KeyType','char','ValueType','char');

    gazeboClient = robotics.internal.GazeboClient;

    % connect to Gazebo
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(gazeboClient);
    modelParam = gazeboClient.getGazeboModelParam(modelName,true,linkName);
    gazeboClient.shutdown();

    % error-out if requested model is not available in Gazebo
    if(strcmp(modelParam.message.name,"") ||...
       ~strcmp(modelParam.message.name,modelName))
        error(message('robotics:robotgazebo:gzsupport:InvalidModelName',...
                      modelName,'gzmodel("list")'));
    end

    % search input link struct
    modelLinkNames = [modelParam.message.links.name];
    % error-out if requested link is not available in Gazebo model
    if(isempty(modelLinkNames))
        instMsg = [ 'gzlink("list","' modelName,'")'];
        error(message('robotics:robotgazebo:gzsupport:InvalidLinkName',...
                      linkName, modelName, instMsg));
    end
    linkParamStruct = modelParam.message.links(1);
    % create output struct and printable string for 'Position' values
    if contains("Position",varargin{:})
        param.Position = [ linkParamStruct.pose.position.x,...
                           linkParamStruct.pose.position.y,...
                           linkParamStruct.pose.position.z];
        paramString('Position') = [...
            'x : ', num2str(linkParamStruct.pose.position.x),' , ',...
            'y : ', num2str(linkParamStruct.pose.position.y),' , ',...
            'z : ', num2str(linkParamStruct.pose.position.z)];
    end
    % create output struct and printable string for 'Orientation' values
    if contains("Orientation",varargin{:})
        param.Orientation = [ linkParamStruct.pose.orientation.w,...
                            linkParamStruct.pose.orientation.x,...
                            linkParamStruct.pose.orientation.y,...
                            linkParamStruct.pose.orientation.z];
        paramString('Orientation') = [...
            'w : ', num2str(linkParamStruct.pose.orientation.w),' , ',...
            'x : ', num2str(linkParamStruct.pose.orientation.x),' , ',...
            'y : ', num2str(linkParamStruct.pose.orientation.y),' , ',...
            'z : ', num2str(linkParamStruct.pose.orientation.z)];
    end
    % create output struct and printable string for 'Mass' value
    if contains("Mass",varargin{:})
        param.Mass = linkParamStruct.inertial.mass;
        paramString('Mass') =  num2str(linkParamStruct.inertial.mass);
    end
    % create output struct and printable string for 'ProductOfInertia' values
    if contains("ProductOfInertia",varargin{:})
        param.ProductOfInertia = [ linkParamStruct.inertial.ixy,...
                            linkParamStruct.inertial.ixz, ...
                            linkParamStruct.inertial.iyz];
        paramString('ProductOfInertia') = [...
            'ixy : ', num2str(linkParamStruct.inertial.ixy),' , ',...
            'ixz : ', num2str(linkParamStruct.inertial.ixz),' , ',...
            'iyz : ', num2str(linkParamStruct.inertial.iyz)];
    end
    % create output struct and printable string for 'PrincipalMoments' values
    if contains("PrincipalMoments",varargin{:})
        param.PrincipalMoments = [ linkParamStruct.inertial.ixx, ...
                            linkParamStruct.inertial.iyy,...
                            linkParamStruct.inertial.izz];
        paramString('PrincipalMoments') = [...
            'ixx : ', num2str(linkParamStruct.inertial.ixx),' , ',...
            'iyy : ', num2str(linkParamStruct.inertial.iyy),' , ',...
            'izz : ', num2str(linkParamStruct.inertial.izz)];
    end
    % create output struct and printable string for 'SelfCollide' value
    if contains("SelfCollide",varargin{:})
        param.SelfCollide = linkParamStruct.self_collide;
        if(param.SelfCollide)
            paramString('SelfCollide') = 'On';
        else
            paramString('SelfCollide') = 'Off';
        end
    end
    % create output struct and printable string for 'EnableWind' value
    if contains("EnableWind",varargin{:})
        param.EnableWind = linkParamStruct.enabled_wind;
        if(param.EnableWind)
            paramString('EnableWind') = 'On';
        else
            paramString('EnableWind') = 'Off';
        end
    end
    % create output struct and printable string for 'Kinematic' value
    if contains("Kinematic",varargin{:})
        param.Kinematic = linkParamStruct.kinematic;
        if(param.Kinematic)
            paramString('Kinematic') = 'On';
        else
            paramString('Kinematic') = 'Off';
        end
    end
    % create output struct and printable string for 'Gravity' value
    if contains("Gravity",varargin{:})
        param.Gravity = linkParamStruct.gravity;
        if(param.Gravity)
            paramString('Gravity') = 'On';
        else
            paramString('Gravity') = 'Off';
        end
    end
    % create output struct and printable string for 'IsStatic' value
    if contains("IsStatic",varargin{:})
        param.IsStatic = linkParamStruct.is_static;
        if(param.IsStatic)
            paramString('IsStatic') = 'On';
        else
            paramString('IsStatic') = 'Off';
        end
    end
    % create output struct and printable string for 'Canonical' values
    if contains("Canonical",varargin{:})
        param.Canonical = linkParamStruct.canonical;
        if(param.Canonical)
            paramString('Canonical') = 'On';
        else
            paramString('Canonical') = 'Off';
        end
    end

end
