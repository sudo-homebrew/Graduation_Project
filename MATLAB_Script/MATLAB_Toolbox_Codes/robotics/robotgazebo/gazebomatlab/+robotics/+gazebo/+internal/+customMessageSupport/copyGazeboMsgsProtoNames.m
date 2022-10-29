function [protoHeaderNames] = copyGazeboMsgsProtoNames(folderPath, gazeboMessageList, gazeboVersionName, messageNameList)
%This function is for internal use only. It may be removed in the future.
%
% This function copies Gazebo Message '.proto' files into folder
% Based on user input Gazebo Message name, corresponding '.proto' files
% and its dependent files names are retrieved. Further, these files are
% copied into user created folder.

%   Copyright 2020-2021 The MathWorks, Inc.

%% folder path of gazebo message '.proto' files
    switch gazeboVersionName
      case "Gazebo 9"
        gazeboMessagePath = fullfile(matlabroot,'toolbox','robotics','robotgazebo','gazebomatlab','gazebomsgs','gazebo9');
      case "Gazebo 10"
        gazeboMessagePath = fullfile(matlabroot,'toolbox','robotics','robotgazebo','gazebomatlab','gazebomsgs','gazebo10');
      case "Gazebo 11"
        gazeboMessagePath = fullfile(matlabroot,'toolbox','robotics','robotgazebo','gazebomatlab','gazebomsgs','gazebo11');
    end

    % '.proto' file list
    protoFilelist = dir(fullfile(gazeboMessagePath,'*.proto'));
    protoFilelist = {protoFilelist.name};

    %% check already available gazebo '.proto' files in the folder and delete ( Clean folder )

    % find already available gazebo message files
    protoFilePresentList = dir(fullfile(folderPath,'*.proto'));
    protoFileDeletList = intersect(protoFilelist,{protoFilePresentList.name});

    % delete already available gazebo message files
    for idx = 1:length(protoFileDeletList)
        src = fullfile( folderPath, protoFileDeletList{idx});
        delete(src);
    end

    %% proto read C++ API
    gazeboMessageHelper = robotics.internal.GazeboClient;

    % list of required '.proto' file names
    protoFileNamesList = {};

    for msgId = 1:length(gazeboMessageList)

        % get '.proto' file name based on input gazebo message name
        protoFileName = getProtoFileNames(char(gazeboMessageList(msgId)), protoFilelist, messageNameList);
        % get dependent '.proto' file names
        protoHeaderNames = gazeboMessageHelper.getProtoHeaderNames(gazeboMessagePath,protoFileName);
        % get nested dependent '.proto' file names
        protoHeaderNames = getNestedProtoFileNames(gazeboMessageHelper,gazeboMessagePath,protoHeaderNames,protoHeaderNames);

        % remove duplicate '.proto' file names
        protoHeaderNames = unique( protoHeaderNames);

        % store original '.proto' file name
        protoFileNamesList{end+1} = protoFileName;
        % store dependent '.proto' file names
        for index = 1:length(protoHeaderNames)
            protoFileNamesList{end+1} = protoHeaderNames{index};
        end
    end

    % remove duplicate '.proto' file names
    protoFileNamesList = unique( protoFileNamesList );

    %% copy files into folder path
    for idx = 1:length(protoFileNamesList)
        src = fullfile( gazeboMessagePath, protoFileNamesList{idx});
        dst = fullfile( folderPath , protoFileNamesList{idx});
        copyfile(src, dst,'f');
    end

end

%% return dependent '.proto' file names
function [protoHeaderNames] = getNestedProtoFileNames(gazeboMessageHelper,...
                                                      gazeboMessagePath,protoHeaderNames,newProtoHeaderNames)

    for idx =1 : length(newProtoHeaderNames)
        % '.proto' file name
        protoFileName = newProtoHeaderNames{idx};
        % get dependent '.proto' file names
        protoHeaderNamesTemp = gazeboMessageHelper.getProtoHeaderNames(gazeboMessagePath,protoFileName);
        protoHeaderNamesTemp = unique( protoHeaderNamesTemp);

        if(~isempty(protoHeaderNamesTemp))
            % copy file names
            for pIdx = 1:length(protoHeaderNamesTemp)
                protoHeaderNames{end+1} = protoHeaderNamesTemp{pIdx};
            end

            % get nested dependent '.proto' file names
            newProtoHeaderNamesTemp = getNestedProtoFileNames(gazeboMessageHelper,...
                                                              gazeboMessagePath,protoHeaderNames,protoHeaderNamesTemp);
            newProtoHeaderNamesTemp = unique( newProtoHeaderNamesTemp);

            if(~isempty(newProtoHeaderNamesTemp))
                % copy file names
                for pIdx = 1:length(newProtoHeaderNamesTemp)
                    protoHeaderNames{end+1} = newProtoHeaderNamesTemp{pIdx};
                end
            end

        end
    end

end

%% returns '.proto' file name based in input message name
function [protoFileName] = getProtoFileNames(gazeboMessageName, protoFilelist, messageNameList)

% search input message name in the list and return '.proto' file name
    idx = strcmp(gazeboMessageName,messageNameList);
    protoFileName = protoFilelist{idx};

end
