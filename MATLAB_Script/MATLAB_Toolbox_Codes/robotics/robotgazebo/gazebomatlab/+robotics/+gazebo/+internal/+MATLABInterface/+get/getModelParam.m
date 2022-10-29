function [param, paramString] = getModelParam(modelName, varargin)
%This function is for internal use only. It may be removed in the future.
%
% This function connects with Gazebo and request model parameter value
% based on modelname. Further, creates MATLAB struct and string of output
% as per received model-joint parameters.

%   Copyright 2020 The MathWorks, Inc.

% output parameter struct
    param.modelname = modelName;
    % output parameter string map. Contains parameter name and value in string form
    paramString = containers.Map('KeyType','char','ValueType','char');

    gazeboClient = robotics.internal.GazeboClient;

    % connect to Gazebo
    robotics.gazebo.internal.MATLABInterface.utils.connectClientUsingProfile(gazeboClient);
    modelParam = gazeboClient.getGazeboModelParam(modelName,true,'');
    gazeboClient.shutdown();

    % error-out if requested model is not available in Gazebo
    if(strcmp(modelParam.message.name,"") ||...
       ~strcmp(modelParam.message.name,modelName))
        error(message('robotics:robotgazebo:gzsupport:InvalidModelName',...
                      modelName,'gzmodel("list")'));
    end

    modelParamStruct = modelParam.message;
    % create output struct and printable string for 'Position' values
    if contains("Position",varargin{:})
        param.Position = [ modelParamStruct.pose.position.x, ...
                           modelParamStruct.pose.position.y, ...
                           modelParamStruct.pose.position.z];
        paramString('Position') = [...
            'x : ', num2str(modelParamStruct.pose.position.x),' , ',...
            'y : ', num2str(modelParamStruct.pose.position.y),' , ',...
            'z : ', num2str(modelParamStruct.pose.position.z)];
    end
    % create output struct and printable string for 'Orientation' values
    if contains("Orientation",varargin{:})
        param.Orientation = [ modelParamStruct.pose.orientation.w,...
                            modelParamStruct.pose.orientation.x,...
                            modelParamStruct.pose.orientation.y,...
                            modelParamStruct.pose.orientation.z];
        paramString('Orientation') = [...
            'w : ', num2str(modelParamStruct.pose.orientation.w),' , ',...
            'x : ', num2str(modelParamStruct.pose.orientation.x),' , ',...
            'y : ', num2str(modelParamStruct.pose.orientation.y),' , ',...
            'z : ', num2str(modelParamStruct.pose.orientation.z)];
    end
    % create output struct and printable string for 'SelfCollide' value
    if contains("SelfCollide",varargin{:})
        param.SelfCollide = modelParamStruct.self_collide;
        if(param.SelfCollide)
            paramString('SelfCollide') = 'On';
        else
            paramString('SelfCollide') = 'Off';
        end
    end
    % create output struct and printable string for 'EnableWind' value
    if contains("EnableWind",varargin{:})
        param.EnableWind = modelParamStruct.enable_wind;
        if(param.EnableWind)
            paramString('EnableWind') = 'On';
        else
            paramString('EnableWind') = 'Off';
        end
    end
    % create output struct and printable string for 'IsStatic' value
    if contains("IsStatic",varargin{:})
        param.IsStatic = modelParamStruct.is_static;
        if(param.IsStatic)
            paramString('IsStatic') = 'On';
        else
            paramString('IsStatic') = 'Off';
        end
    end

end
