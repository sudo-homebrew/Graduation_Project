function cgInfo = getCodegenInfo(topic,messageType,listToAdd, rosver)
%GETCODEGENINFO Return code generation information for

%   Copyright 2020-2021 The MathWorks, Inc.
    if nargin < 4
        rosver = 'ros';
    end

    [msgStructGen,~,~,msgInfo] = ros.codertarget.internal.getEmptyCodegenMsg(messageType,rosver);

    cgInfo.CppHeader = msgInfo.includeHeader;
    cgInfo.MsgClass = msgInfo.msgCppClassName;
    cgInfo.MsgClassPtr = [msgInfo.msgCppClassName '*'];
    if isequal(rosver, 'ros')
        cgInfo.CppClass = ['boost::shared_ptr<' msgInfo.msgCppClassName ' const>'];
    else
        cgInfo.CppClass = ['std::shared_ptr<' msgInfo.msgCppClassName '>'];
    end
    cgInfo.CppConstClassRef = ['const ' cgInfo.CppClass '&'];
    cgInfo.MsgStructGen = msgStructGen;

    % Create additional field - CppSvcType for Service messages
    % Services need this specific treatment since the actual header file we
    % want to include must contains both Request and Response definition.
    if strcmp(cgInfo.MsgClass(end-6:end),'Request')
        cgInfo.CppSvcType = cgInfo.MsgClass(1:end-7);
    end

    % Special treatment for ROS 2 Service messages
    if strcmp(rosver,'ros2')
        if strcmp(cgInfo.MsgClass(end-6:end),'Request')
            svcInfo = ros.internal.ros2.getServiceInfo(messageType,messageType(1:end-7),'Request');
            cgInfo.CppSvcType = svcInfo.msgBaseCppClassName;
            cgInfo.MsgClass = svcInfo.msgCppClassName;
        elseif strcmp(cgInfo.MsgClass(end-7:end),'Response')
            svcInfo = ros.internal.ros2.getServiceInfo(messageType,messageType(1:end-8),'Response');
            cgInfo.MsgClass = svcInfo.msgCppClassName;
        end
    end

    hObj = ros.codertarget.internal.ROSMATLABCgenInfo.getInstance;
    if strcmp(listToAdd,'sub')
        % Add new subscriber to subscriber list
        addSubscriber(hObj,topic,messageType,cgInfo.MsgClass);
    elseif strcmp(listToAdd,'pub')
        % Add new publisher to publisher list
        addPublisher(hObj,topic,messageType,cgInfo.MsgClass);
    else
        % Do nothing for other situations since there is no need to track
        % them for now.
    end
end
