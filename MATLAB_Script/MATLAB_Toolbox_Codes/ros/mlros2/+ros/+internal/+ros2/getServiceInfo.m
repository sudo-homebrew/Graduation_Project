function msgInfo = getServiceInfo(msg,srvType,defType,type)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

%getServiceInfo returns the prefix name to which _service_client or
%_service_server can be attached and other valuable information.
if nargin < 4
    type = 'srv';
end
validateattributes(msg,{'char','string'},{'nonempty'});
validateattributes(type,{'char','string'},{'nonempty'});
msg = convertStringsToChars(msg);
srvType = convertStringsToChars(srvType);

[msgInfo.pkgName, msgInfo.msgName] = fileparts(srvType);
[~, msgInfo.srvName] = fileparts(msg);
msgInfo.defType = defType;
msgInfo.baseName = [msgInfo.pkgName, '_', 'msg' '_', msgInfo.srvName];
msgInfo.baseSrvName = [msgInfo.pkgName, '_', type '_', msgInfo.msgName];
msgInfo.msgCppClassName = [msgInfo.pkgName,'::srv::',msgInfo.msgName,'::',defType];
msgInfo.msgBaseCppClassName = [msgInfo.pkgName,'::srv::' msgInfo.msgName];
msgInfo.msgStructGenFileName = [lower(msgInfo.srvName(1)) msgInfo.srvName(2:end)];
msgInfo.msgStructGen = ['ros.internal.ros2.messages.' msgInfo.pkgName '.' msgInfo.msgStructGenFileName];

msgInfo.libNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
                                    { ...
                                      [msgInfo.pkgName,'.dll'], ...
                                      ['libmw', msgInfo.pkgName,'.dylib'],...
                                      ['libmw', msgInfo.pkgName,'.so'],...
                                    });

%Note: Path may not be always be available. This might be a custom message
%Note: getServiceInfo can be called from augment while generating message
%files for services
%Hence: We cannot check for the presence of the files.
%RULE: getServiceInfo should always give a msgInfo data
%Note: MCR separates toolbox MATLAB files from library files.
%      toolboxdir('ros') will find the root/mcr/toolbox/ros folder,
%      while the library files are in root/toolbox/ros/bin,
%      and interprocess files are in root/sys/ros2.
%      ctfroot points to the copied app folder, which will not have these
%RULE: Use matlabroot to find correct path to library files

msgInfo.path = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
                        msgInfo.libNameMap(computer('arch')));
msgInfo.cppFactoryClass = [msgInfo.pkgName '_'  msgInfo.msgName '_service'];

msgInfo.clientClassName = [msgInfo.baseSrvName '_client'];
msgInfo.serverClassName = [msgInfo.baseSrvName '_server'];
msgInfo.publisherClassName = [msgInfo.baseName '_publisher'];
msgInfo.subscriberClassName = [msgInfo.baseName '_subscriber'];

incl_message = ros.internal.utilities. ...
    convertCamelcaseToLowercaseUnderscore(msgInfo.msgName);
includeHeader = fullfile(msgInfo.pkgName,'srv',[incl_message '.hpp']);
msgInfo.includeHeader = replace(includeHeader,'\','/'); %To be consistent across all platforms
msgInfo.custom = false;
msgInfo.srcPath   = [];
msgInfo.installDir = [];

%finally if there is a custom message, then overwrite
reg = ros.internal.CustomMessageRegistry.getInstance('ros2');
msgInfo = reg.updateServiceInfo(msgInfo);
