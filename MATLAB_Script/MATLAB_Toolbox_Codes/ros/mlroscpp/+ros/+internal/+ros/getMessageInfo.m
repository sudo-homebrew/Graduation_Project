function msgInfo = getMessageInfo(msg, registry, type)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2020 The MathWorks, Inc.

%getMessageInfo returns the prefix name to which _publisher or _subscriber
%can be attached and other valuable information

    if nargin < 2
        registry = ros.internal.CustomMessageRegistry.getInstance('ros');
    end
    if nargin < 3
        type = 'msg';
    end

    validateattributes(msg,{'char','string'},{'nonempty'});
    validateattributes(type,{'char','string'},{'nonempty'});
    msg = convertStringsToChars(msg);
    type = lower(convertStringsToChars(type));
    
    msgList = rostype.getMessageList;
    invalidMessageTypes = ros.internal.utilities.getMessageTypesWithCppKeyword;
    if ismember(msg,invalidMessageTypes) && ~ismember(msg,msgList)
        error(message(...
            'ros:utilities:message:MessageNotSupportedInThisRelease',msg));
    end

    [msgInfo.pkgName, msgInfo.msgName] = fileparts(msg);
    msgInfo.defType = '';
    msgInfo.baseName = [msgInfo.pkgName, '_', type '_', msgInfo.msgName];
    msgInfo.msgCppClassName = [msgInfo.pkgName,'::' msgInfo.msgName];
    msgInfo.msgStructGenFileName = [lower(msgInfo.msgName(1)) msgInfo.msgName(2:end)];
    msgInfo.msgStructGen = ['ros.internal.ros.messages.' msgInfo.pkgName '.' msgInfo.msgStructGenFileName];
    msgInfo.msgClassGen = ros.internal.ros.getMsgClass(msg);
    %msgInfo.msg1to2ConverterName = ['ros.internal.ros.converters.' msgInfo.baseName '_1To2_Converter'];
    %msgInfo.msg2to1ConverterName = ['ros.internal.ros.converters.' msgInfo.baseName '_2To1_Converter'];

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
    %Note: getMessageInfo can be called from augment while generating message
    %files
    %Hence: We cannot check for the presence of the files.
    %RULE: getMessageInfo should always give a msgInfo data
    %Note: MCR separates toolbox MATLAB files from library files.
    %      toolboxdir('ros') will find the root/mcr/toolbox/ros folder,
    %      while the library files are in root/toolbox/ros/bin,
    %      and interprocess files are in root/sys/ros1.
    %      ctfroot points to the copied app folder, which will not have these
    %RULE: Use matlabroot to find correct path to library files
    dllPath = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
                       'ros1',msgInfo.libNameMap(computer('arch')));
    if isfile(dllPath)
        msgInfo.path = dllPath;
    else
        msgInfo.path = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
                                'ros1','custom_msgs',msgInfo.libNameMap(computer('arch')));
    end
    msgInfo.publisherClassName = [msgInfo.baseName '_publisher'];
    msgInfo.subscriberClassName = [msgInfo.baseName '_subscriber'];
    msgInfo.isSamePackage = true;

    % incl_message = ros.internal.utilities. ...
    % convertCamelcaseToLowercaseUnderscore(msgInfo.msgName);
    includeHeader = fullfile(msgInfo.pkgName,[msgInfo.msgName '.h']);
    msgInfo.includeHeader = replace(includeHeader,'\','/'); %To be consistent across all platforms
    msgInfo.custom = false;
    msgInfo.srcPath   = [];
    msgInfo.installDir = [];

    %finally if there is a custom message, then overwrite
    msgInfo = registry.updateMessageInfo(msgInfo);
