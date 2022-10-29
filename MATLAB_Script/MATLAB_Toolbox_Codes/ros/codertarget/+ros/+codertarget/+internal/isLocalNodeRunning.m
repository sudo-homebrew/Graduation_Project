function isRunning = isLocalNodeRunning(nodeName)
%This function is for internal use only. It may be removed in the future.
% Check if a ROS node is running on local host.

%   Copyright 2020-2021 The MathWorks, Inc.
if ispc
    appName = [nodeName '.exe'];
else
    appName = nodeName;
end

isRunning = false;
if isequal(computer('arch'),'glnxa64')
    % Output pid and args left justified
    cmd = sprintf('ps axo pid:1,args:1 | grep -E "%s(\\s|$)" | grep -v "grep"',appName);
    [status,result] = system(cmd);
    if status == 0
        % Find out pid's of potential candidates by finding digits at each
        % line beginning.
        % result = '1257 bash\n3022 bash\n12757 bash\n' will return a cell
        % array {'1257','3022','12757'} using extract below
        pidList = extract(result,lineBoundary + digitsPattern);
        % Test comm to make sure it matches appName. In Linux, only
        % first 15 characters are stored in /proc/pid/comm file so make
        % sure we only test against first 15 characters of appName
        pat = appName(1:min(15,length(appName)));
        for k = 1:length(pidList)
            [status,result] = system(sprintf('cat /proc/%s/comm | grep "^%s"',pidList{k},pat));
            if (status == 0) && contains(result,pat)
                isRunning = true;
                break;
            end
        end
    end
elseif isequal(computer('arch'),'maci64')
    % Use ps as pgrep is not available on MAC by default
    cmd = sprintf('ps axo comm | grep "%s" | grep -v "grep"', appName);
    [status,result] = system(cmd);
    % If command fails, return not running
    isRunning = (status == 0) && contains(result,appName);
else
    % win64 - use tasklist
    cmd = sprintf('tasklist /FI "IMAGENAME eq %s" /FO list', appName);
    [status,result] = system(cmd);
    % If command fails, return not running
    isRunning = (status == 0) && contains(result,appName);
end

end
