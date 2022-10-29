function createGazeboPlugin(customMsgStruct,destFolder)
%This function is for internal use only. It may be removed in the future.
%
%This function generates GazeboPlugin.cpp file.
%

%   Copyright 2020 The MathWorks, Inc.

    %% create Co-Sim Plugin source file

    % read template of Co-Sim Plugin cpp file
    GazeboPluginLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                        '+robotics','+gazebo','+internal','+customMessageSupport','+templates','GazeboPlugin.cpp.tmpl'));

    coSimNameSpace = "robotics::gazebotransport::";

    GazeboPluginHeaderString = '';
    GazeboPluginHeaderInitString = '';

    for idx=1:length(customMsgStruct)
        %% retrieve proto message name with namespace in Simulink message format
        [messsageName, ~ ] = robotics.gazebo.internal.customMessageSupport.getMsgNameWithNamespace...
            (customMsgStruct{idx}.messageNameSpaceList,customMsgStruct{idx}.messageBaseName,'Simulink');

        % add CustomMessageHandler header definitions #include .....
        cppHandlerHeaderName = join([messsageName,'CustomMessageHandler.hpp'],"");
        cppHandlerHeaderFullName = join(['#include "gazebotransport/gazebocustom/gazebocustommsghandler/',cppHandlerHeaderName,'"',newline],"");
        GazeboPluginHeaderString = join([GazeboPluginHeaderString,cppHandlerHeaderFullName],"");

        % add register custom msg handler definition
        messageType = join(['"','gazebo_msgs/custom/',customMsgStruct{idx}.simulinkMessageName,'"'],"");
        handlerDef = join(["this->m_customDispatch->registerCustomHandler(std::make_shared<",coSimNameSpace,messsageName,"CustomMessageHandler>(this->m_node),",messageType,");",newline],"");
        GazeboPluginHeaderInitString = join([GazeboPluginHeaderInitString,handlerDef],"");

    end

    % add header and msg handler definitions string in the Co-Sim Plugin cpp template string
    GazeboPluginLog =  strrep(GazeboPluginLog,'{% msgInfo.MessageHandlerHeaderName %}',GazeboPluginHeaderString);
    GazeboPluginLog =  strrep(GazeboPluginLog,'{% msgInfo.MessageHandlerHeaderInit %}',GazeboPluginHeaderInitString);

    % create GazeboPlugin.cpp file
    robotics.gazebo.internal.customMessageSupport.createFile(GazeboPluginLog, destFolder);

end
