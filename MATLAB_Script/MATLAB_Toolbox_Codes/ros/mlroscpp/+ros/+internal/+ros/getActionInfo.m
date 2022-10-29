function msgInfo = getActionInfo(msg,actionType,defType,type)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020 The MathWorks, Inc.

%getMessageInfo returns the prefix name to which _publisher or _subscriber
%can be attached and other valuable information
    if nargin < 4
        type = 'action';
    end
    validateattributes(msg,{'char','string'},{'nonempty'});
    validateattributes(type,{'char','string'},{'nonempty'});
    msg = convertStringsToChars(msg);
    actionType = convertStringsToChars(actionType);

    % For handling the user-facing class of non-generated message classes
    specialMsgClassMap = containers.Map({'ros/Duration', ...
                        'ros/Time'}, ...
                                        {'ros.msg.Duration', ...
                        'ros.msg.Time'});

    [msgInfo.pkgName, msgInfo.actionName] = fileparts(actionType);
    [~, msgInfo.msgName] = fileparts(msg);
    msgInfo.defType = defType;
    msgInfo.baseName = [msgInfo.pkgName, '_', 'msg' '_', msgInfo.msgName];
    msgInfo.baseActionName = [msgInfo.pkgName, '_', type '_', msgInfo.actionName];
    msgInfo.msgCppClassName = [msgInfo.pkgName '::' msgInfo.actionName defType];
    msgInfo.msgBaseCppClassName = [msgInfo.pkgName,'::' msgInfo.actionName];
    msgInfo.msgStructGenFileName = [lower(msgInfo.msgName(1)) msgInfo.msgName(2:end)];
    msgInfo.msgStructGen = ['ros.internal.ros.messages.' msgInfo.pkgName '.' msgInfo.msgStructGenFileName];    
    if isKey(specialMsgClassMap,msg)
        msgInfo.msgClassGen = specialMsgClassMap(msg);
    else
        msgInfo.msgClassGen = ['ros.msggen.' msgInfo.pkgName '.' msgInfo.msgName];
    end

    if isequal(msgInfo.pkgName,'actionlib')
        msgInfo.libNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                            { ...
                                                [msgInfo.pkgName,'_mw.dll'], ...
                                                ['libmw', msgInfo.pkgName,'_mw.dylib'],...
                                                ['libmw', msgInfo.pkgName,'_mw.so'],...
                   });
    else
        msgInfo.libNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                            { ...
                                                [msgInfo.pkgName,'.dll'], ...
                                                ['libmw', msgInfo.pkgName,'.dylib'],...
                                                ['libmw', msgInfo.pkgName,'.so'],...
                   });
    end

    %Note: Path may not be always be available. This might be a custom message
    %Note: getServiceInfo can be called from augment while generating message
    %files for services
    %Hence: We cannot check for the presence of the files.
    %RULE: getServiceInfo should always give a msgInfo data
    %Note: MCR separates toolbox MATLAB files from library files.
    %      toolboxdir('ros') will find the root/mcr/toolbox/ros folder,
    %      while the library files are in root/toolbox/ros/bin,
    %      and interprocess files are in root/sys/ros1.
    %      ctfroot points to the copied app folder, which will not have these
    %RULE: Use matlabroot to find correct path to library files
    dllPath = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
                       'ros1',msgInfo.libNameMap(computer('arch')));
    if isequal(exist(dllPath,'file'),2)
        msgInfo.path = dllPath;
    else
        msgInfo.path = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
                                'ros1','custom_msgs',msgInfo.libNameMap(computer('arch')));
    end

    msgInfo.cppFactoryClass = [msgInfo.pkgName '_'  msgInfo.actionName '_action'];
    
    msgInfo.clientClassName = [msgInfo.baseActionName '_action_client'];
    msgInfo.serverClassName = [msgInfo.baseActionName '_action_server'];
    msgInfo.publisherClassName = [msgInfo.baseName '_publisher'];
    msgInfo.subscriberClassName = [msgInfo.baseName '_subscriber'];

    % incl_message = ros.internal.utilities. ...
    % convertCamelcaseToLowercaseUnderscore(msgInfo.msgName);
    includeHeader = fullfile(msgInfo.pkgName,[msgInfo.actionName 'Action.h']);
    msgInfo.includeHeader = replace(includeHeader,'\','/'); %To be consistent across all platforms
    msgInfo.custom = false;
    msgInfo.srcPath   = [];
    msgInfo.installDir = [];

    %finally if there is a custom message, then overwrite
    reg = ros.internal.CustomMessageRegistry.getInstance('ros');
    msgInfo = reg.updateMessageInfo(msgInfo);
