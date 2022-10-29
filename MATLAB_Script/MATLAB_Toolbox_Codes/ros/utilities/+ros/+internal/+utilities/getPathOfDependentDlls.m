function dllPath = getPathOfDependentDlls(msgType,rosver)
% This is for internal use only. It will be renoved in future.

reg = ros.internal.CustomMessageRegistry.getInstance(rosver);
customMsgList = reg.getMessageList;
customPkgs = cell.empty;
for i = 1:numel(customMsgList)
    customPkgs{i} = fileparts(customMsgList{i});
end
customPkgs = unique(customPkgs);

%Get the dependent message packages and message names. 
map = containers.Map();
pkgNamesMap = ros.internal.utilities.findMsgDepends(msgType,map,rosver);
pkgNames = pkgNamesMap.keys;

dllPath = {};
for i = 1:numel(pkgNames)
    if ismember(pkgNames{i},customPkgs)
        msgNames = pkgNamesMap(pkgNames{i});
        msgType = [pkgNames{i},'/',msgNames{1}];
        dllPath = [dllPath reg.getMessageInfo(msgType).dllPath]; %#ok<AGROW>
    else
        dllPath = [dllPath getDllPath(pkgNames{i},rosver)]; %#ok<AGROW>
    end
end
dllPath = unique(dllPath);
end

function path = getDllPath(pkgName,rosver)
if isequal(pkgName,'ros')
    pkgName = 'std_msgs';
end
if isequal(pkgName,'actionlib')
    msgInfo.libNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
        { ...
        [pkgName,'_mw.dll'], ...
        ['libmw', pkgName,'_mw.dylib'],...
        ['libmw', pkgName,'_mw.so'],...
        });
else
    msgInfo.libNameMap = containers.Map({'win64','maci64','glnxa64'}, ...
        { ...
        [pkgName,'.dll'], ...
        ['libmw', pkgName,'.dylib'],...
        ['libmw', pkgName,'.so'],...
        });
end
if isequal(rosver,'ros')
    dllPath = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
        'ros1',msgInfo.libNameMap(computer('arch')));
    if isfile(dllPath)
        path = dllPath;
    else
        path = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
            'ros1','custom_msgs',msgInfo.libNameMap(computer('arch')));
    end
else
    path = fullfile(matlabroot,'toolbox','ros','bin',computer('arch'),...
        msgInfo.libNameMap(computer('arch')));
end
end