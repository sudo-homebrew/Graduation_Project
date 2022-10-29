function msgInfo = getServiceInfo(msg,srvType,defType,type)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2020 The MathWorks, Inc.

%getMessageInfo returns the prefix name to which _publisher or _subscriber
%can be attached and other valuable information
if nargin < 4
    type = 'srv';
end
validateattributes(msg,{'char','string'},{'nonempty'});
validateattributes(type,{'char','string'},{'nonempty'});
msg = convertStringsToChars(msg);
srvType = convertStringsToChars(srvType);

srvList = rostype.getServiceList;
invalidServiceTypes = ros.internal.utilities.getServiceTypesWithCppKeyword;

if ismember(srvType,invalidServiceTypes) && ~ismember(srvType,srvList)
    error(message(...
        'ros:utilities:message:ServiceNotSupportedInThisRelease',srvType));
end

% For handling the user-facing class of non-generated message classes
specialMsgClassMap = containers.Map({'ros/Duration', ...
    'ros/Time'}, ...
    {'ros.msg.Duration', ...
    'ros.msg.Time'});

[msgInfo.pkgName, msgInfo.msgName] = fileparts(srvType);
[~, msgInfo.srvName] = fileparts(msg);
msgInfo.defType = defType;
msgInfo.baseName = [msgInfo.pkgName, '_', 'msg' '_', msgInfo.srvName];
msgInfo.baseSrvName = [msgInfo.pkgName, '_', type '_', msgInfo.msgName];
msgInfo.msgCppClassName = [msgInfo.pkgName,'::' msgInfo.msgName '::' defType];
msgInfo.msgBaseCppClassName = [msgInfo.pkgName,'::' msgInfo.msgName];
msgInfo.msgStructGenFileName = [lower(msgInfo.srvName(1)) msgInfo.srvName(2:end)];
msgInfo.msgStructGen = ['ros.internal.ros.messages.' msgInfo.pkgName '.' msgInfo.msgStructGenFileName];
if isKey(specialMsgClassMap,msg)
    msgInfo.msgClassGen = specialMsgClassMap(msg);
else
    msgInfo.msgClassGen = ['ros.msggen.' msgInfo.pkgName '.' msgInfo.srvName];
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

msgInfo.cppFactoryClass = [msgInfo.pkgName '_'  msgInfo.msgName '_service'];

msgInfo.clientClassName = [msgInfo.baseSrvName '_client'];
msgInfo.serverClassName = [msgInfo.baseSrvName '_server'];
msgInfo.publisherClassName = [msgInfo.baseName '_publisher'];
msgInfo.subscriberClassName = [msgInfo.baseName '_subscriber'];

% incl_message = ros.internal.utilities. ...
% convertCamelcaseToLowercaseUnderscore(msgInfo.msgName);
includeHeader = fullfile(msgInfo.pkgName,[msgInfo.msgName '.h']);
msgInfo.includeHeader = replace(includeHeader,'\','/'); %To be consistent across all platforms
msgInfo.custom = false;
msgInfo.srcPath   = [];
msgInfo.installDir = [];

%finally if there is a custom message, then overwrite
reg = ros.internal.CustomMessageRegistry.getInstance('ros');
msgInfo = reg.updateServiceInfo(msgInfo);
