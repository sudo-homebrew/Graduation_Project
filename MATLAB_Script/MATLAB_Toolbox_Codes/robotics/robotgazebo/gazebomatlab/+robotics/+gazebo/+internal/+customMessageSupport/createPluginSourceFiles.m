function createPluginSourceFiles(customMsgStruct,PluginUtilsPath)
%This function is for internal use only. It may be removed in the future.
%
%This function generates utilities required on the plugin side.
%

%   Copyright 2019-2020 The MathWorks, Inc.

    %% create Co-Sim Plugin source file

    robotics.gazebo.internal.customMessageSupport.createGazeboPlugin(....
        customMsgStruct,fullfile(PluginUtilsPath,'src','GazeboPlugin.cpp'));

    %% create custom plugin template
    customPluginLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                        '+robotics','+gazebo','+internal','+customMessageSupport','+templates','customMessagePlugin.cpp.tmpl'));

    header_name = [];
    message_pointer_name = [];
    subscriberPtrList = [];
    publisherPtrList = [];
    subscriberNodeList = [];
    publisherNodeList = [];
    subscribeCallBackList = [];
    publisherCommandList = [];

    for idx=1:length(customMsgStruct)

        file_name = erase(customMsgStruct{idx}.protoFileName,'.proto');

        %% retrieve proto message name with namespace in C++ message format
        [messageNamenamesapce, ~ ]  = robotics.gazebo.internal.customMessageSupport.getMsgNameWithNamespace...
            (customMsgStruct{idx}.messageNameSpaceList, customMsgStruct{idx}.messageBaseName,'C++');

        %% retrieve proto message name with namespace in Simulink message format
        [message_name_namesapce, ~ ]  = robotics.gazebo.internal.customMessageSupport.getMsgNameWithNamespace...
            (customMsgStruct{idx}.messageNameSpaceList, customMsgStruct{idx}.messageBaseName,'Simulink');

        header_name = join([header_name, join(['#include <',file_name,'.pb.h>'],""), newline ],'');

        message_pointer_name = join([message_pointer_name,join(['typedef const boost::shared_ptr<const ',...
                            messageNamenamesapce,'> ',...
                            string(message_name_namesapce),'Ptr;'  ],""),newline],'');

        topicNamePrefix = ['gazebo/default/',message_name_namesapce,'/'];

        subscriberPtrList = join([subscriberPtrList,join(['transport::SubscriberPtr commandSubscriber',lower(message_name_namesapce),'0;'],""),newline],'');

        publisherPtrList = join([publisherPtrList,join(['gazebo::transport::PublisherPtr commandPublisher',lower(message_name_namesapce),'0;'],""),newline],'');

        subscriberNodeList = join([subscriberNodeList,join(['commandSubscriber',lower(message_name_namesapce),'0 = node->Subscribe("',topicNamePrefix,'test_subscriber0',...
                            '", &customMessagePlugin::subscribeCallback',lower(message_name_namesapce),'0',', this);'],""),newline],'');

        publisherNodeList = join([publisherNodeList,join(['commandPublisher',lower(message_name_namesapce),'0 = node->Advertise<',messageNamenamesapce,'>',...
                            '("',topicNamePrefix,'test_publisher','");'],""),newline],'');

        publisherCommandList = join([publisherCommandList,join(['//',messageNamenamesapce,'  ',lower(message_name_namesapce),'_msg;',newline,...
                            '//commandPublisher',lower(message_name_namesapce),'0->Publish(',lower(message_name_namesapce),'_msg',');',newline,newline],"")],'');

        subscribeCallBackList = join([subscribeCallBackList,join(['void subscribeCallback',lower(message_name_namesapce),'0',...
                            '(',message_name_namesapce,'Ptr ','&msg)',newline,'{',newline,...
                            '  //std::cout << msg->DebugString() << std::endl;',newline,'}',newline,newline],"")],'');

    end

    customPluginLog =  strrep(customPluginLog,'{% msgInfo.ProtoMessageHeaders %}',header_name);
    customPluginLog =  strrep(customPluginLog,'{% msgInfo.ProtoMessagePtrDef %}',message_pointer_name);
    customPluginLog =  strrep(customPluginLog,'{% msgInfo.GazeboSubscriberPtrList %}',subscriberPtrList);
    customPluginLog =  strrep(customPluginLog,'{% msgInfo.GazeboPublisherPtrList %}',publisherPtrList);
    customPluginLog =  strrep(customPluginLog,'{% msgInfo.GazeboSubscriberNodeDef %}',subscriberNodeList);
    customPluginLog =  strrep(customPluginLog,'{% msgInfo.GazeboPublisherNodeDef %}',publisherNodeList);
    customPluginLog =  strrep(customPluginLog,'{% msgInfo.PublisherCommand %}',publisherCommandList);
    customPluginLog =  strrep(customPluginLog,'{% msgInfo.SubscriberCallback %}',subscribeCallBackList);


    robotics.gazebo.internal.customMessageSupport.createFile(customPluginLog,fullfile(PluginUtilsPath,'src','customMessagePlugin.cpp'));

    %% create CMakeLists.txt
    cmakeLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                 '+robotics','+gazebo','+internal','+customMessageSupport','+templates','CMakeLists.txt.tmpl'));

    robotics.gazebo.internal.customMessageSupport.createFile(cmakeLog,fullfile(PluginUtilsPath,'src','CMakeLists.txt'));


    %% create custom plugin world plugin
    worldLog = fileread(fullfile(matlabroot, 'toolbox', 'robotics', 'robotgazebo', 'gazebomatlab',...
                                 '+robotics','+gazebo','+internal','+customMessageSupport','+templates','multiSensorCustomPluginTest.world.tmpl'));

    robotics.gazebo.internal.customMessageSupport.createFile(worldLog,fullfile(PluginUtilsPath,'world','multiSensorCustomPluginTest.world'));

end
