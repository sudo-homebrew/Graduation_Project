function [list] = getGazeboMsgsList
%This function is for internal use only. It may be removed in the future.
%
% This function returns gazebo message list in gazebo format "gazebo.msgs.*"
%

%   Copyright 2020 The MathWorks, Inc.

    gazeboMessagePath = fullfile(matlabroot,'toolbox','robotics','robotgazebo','gazebomatlab','gazebomsgs','gazebo9');

    protoFilelist = dir(fullfile(gazeboMessagePath,'*.proto'));
    protoFilelist = {protoFilelist.name};

    % proto read C++ API
    gazeboMessageNameReader = robotics.internal.GazeboClient;

    list = {};

    for idx = 1:length(protoFilelist)
        protoFileName = protoFilelist{idx};
        messageFullName = gazeboMessageNameReader.getMessageFullName(gazeboMessagePath,protoFileName);
        list{end+1} = messageFullName{1};
    end

end
