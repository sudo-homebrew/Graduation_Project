function fileOut = generateSupportFile(varargin)
%generateSupportFile ROS troubleshooting utility.
%   generateSupportFile gathers diagnostic information about the current
%   state of the ROS system and saves the output to a text file 'rossupport.txt'
%   in a temporary folder. The file is then opened in the editor for viewing.
%   If you are connected to a ROS network, this function will also gather
%   information about nodes, topics, services, and parameters.
%
%   generateSupportFile('FILENAME') gathers diagnostic ROS information and saves
%   the results to the text file FILENAME. FILENAME can be either a
%   relative or an absolute path to a file. If the file already exists, it
%   will be overwritten.
%
%   FILEOUT = generateSupportFile(___) returns the full path to the generated
%   file and does not open the file in the editor for viewing.
%
%   Examples:
%      % Generates basic diagnostic information in rossupport.txt
%      ros.internal.generateSupportFile
%
%      % Collect diagnostic information and analyze an active ROS network
%      rosinit 72.82.94.221
%      ros.internal.generateSupportFile
%
%      % Generates information in my_file.txt in the current folder
%      ros.internal.generateSupportFile('my_file.txt')
%
%      % Returns the full path to the report file and does not open the
%      % text file for editing
%      outFilePath = ros.internal.generateSupportFile
%
%    NOTE: This function is for internal use only. It may be removed in the future.

%   Copyright 2014-2021 The MathWorks, Inc.

    narginchk(0,1);

    % Default text file name
    filename = 'rossupport.txt';

    % Generate file path in tempdir by default
    filepath = fullfile(tempdir, filename);

    if nargin == 1
        % The user specified a file name
        filename = varargin{1};

        % Get absolute path to output file
        validateattributes(filename, {'char'}, {'nonempty'}, 'generateSupportFile', 'filename');
        filepath = ros.internal.FileSystem.relativeToAbsolute(filename);

        % Extract actual file name (in case user specified full path)
        [~,file,ext] = fileparts(filepath);
        filename = [file,ext];
    end

    % Deletes text file at FILEPATH if one already exists.
    if ~isempty(dir(filepath))
        try
            delete(filepath);
        catch ex
            error(message('ros:mlros:support:FileDeleteError', filepath));
        end
    end

    % Opens support file for writing
    fid = fopen(filepath,'wt');
    if (fid == -1)
        error(message('ros:mlros:support:FileOpenError', filepath));
    end
    c = onCleanup(@()fclose(fid));

    % Display message to command window.
    disp(message('ros:mlros:support:GeneratingDiagnostic', filename).getString);

    % Variables cr and sp represent strings that are repeatedly called.
    cr = newline;
    sp = '----------';

    % MATLAB, OS, Java, and toolbox version information
    fprintf(fid, evalc('ver'));

    % Print current date and time
    fprintf(fid, [cr, sp, 'DATE/TIME', sp, cr, cr, '%s', cr, cr], datestr(now));

    % Print and check ROS-specific environment variables
    fprintf(fid, [cr, sp, 'ROS-SPECIFIC ENVIRONMENT VARIABLES', sp, cr, cr]);

    rosMasterURI = getenv('ROS_MASTER_URI');
    fprintf(fid, 'ROS_MASTER_URI = %s\n', rosMasterURI);
    try
        % Check for Master now requires a valid URI format
        if ~isempty(rosMasterURI)
            reachable = ros.internal.NetworkIntrospection.isMasterReachable(rosMasterURI, 1);
            if reachable
                fprintf(fid, ['\t' 'Master is reachable.' cr]);
            else
                fprintf(fid, ['\t' 'Master is not reachable.' cr]);
            end
        end
    catch ex
        fprintf(fid, ['[ROSSUPPORT ERROR] Error checking Master based on environment variable URI.' cr]);
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end

    rosHostname = getenv('ROS_HOSTNAME');
    fprintf(fid, 'ROS_HOSTNAME = %s\n', rosHostname);

    rosIP = getenv('ROS_IP');
    fprintf(fid, 'ROS_IP = %s\n', rosIP);

    rosNamespace = getenv('ROS_NAMESPACE');
    fprintf(fid, 'ROS_NAMESPACE = %s\n', rosNamespace);
    if ~isempty(rosNamespace)
        validNamespace = ros.internal.Namespace.isValidGraphName(rosNamespace);
        if ~validNamespace
            fprintf(fid, ['\t' 'The namespace is not a valid ROS graph name.' cr]);
        end
    end

    defaultCorePort = getenv('ROS_DEFAULT_CORE_PORT');
    fprintf(fid, 'ROS_DEFAULT_CORE_PORT = %s\n', defaultCorePort);

    defaultNodePort = getenv('ROS_DEFAULT_NODE_PORT');
    fprintf(fid, 'ROS_DEFAULT_NODE_PORT = %s\n', defaultNodePort);

    rosDomainID = getenv('ROS_DOMAIN_ID');
    fprintf(fid, 'ROS_DOMAIN_ID = %s\n', rosDomainID);

    rmwImplementation = getenv('RMW_IMPLEMENTATION');
    fprintf(fid, 'RMW_IMPLEMENTATION = %s\n', rmwImplementation);

    ddsProfilesFile = getenv('FASTRTPS_DEFAULT_PROFILES_FILE');
    fprintf(fid, 'FASTRTPS_DEFAULT_PROFILES_FILE = %s\n', ddsProfilesFile);

    % Print and check values for Master URI and NodeHost
    try
        fprintf(fid, [cr, sp, 'GENERATED MASTER URI AND NODEHOST', sp, cr, cr]);
        masterURI = ros.internal.Net.generateMasterURI(true);
        fprintf(fid, 'Generated Master URI: %s\n', masterURI);
        nodeHost = ros.internal.Net.generateNodeHost(masterURI, true);
        fprintf(fid, 'Generated Node Host: %s\n', nodeHost);
    catch ex
        fprintf(fid, ['[ROSSUPPORT ERROR] Error generating Master URI and NodeHost.' cr]);
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end

    % Print ROS Connectivity Information
    fprintf(fid, [cr, sp, 'ROS Connectivity', sp, cr, cr]);
    if ros.internal.Global.isCoreActive
        fprintf(fid, ['Global core is running.' cr]);
        core = ros.internal.Global.getCoreHandle(false);
        fprintf(fid, ['\t' 'Port: ' num2str(core.Port) cr]);
        fprintf(fid, ['\t' 'MasterURI: ' core.MasterURI cr]);
    else
        fprintf(fid, ['Global core is not running.' cr]);
    end

    if ros.internal.Global.isNodeActive
        fprintf(fid, ['Global node is running.' cr]);
        node = ros.internal.Global.getNodeHandle(false);
        fprintf(fid, ['\t' 'Name: ' node.Name cr]);
        fprintf(fid, ['\t' 'MasterURI: ' node.MasterURI cr]);
        fprintf(fid, ['\t' 'NodeURI: ' node.NodeURI cr]);

        % Connected to an active ROS network, so print out some information
        % If any of this fails, continue printing out other information
        try
            % rosnode list
            fprintf(fid, [cr, sp, 'ROS Node Information', sp, cr, cr]);
            nodeList = rosnode('list');
            fprintf(fid, [cr 'Node List:' cr]);
            fprintf(fid, evalc('rosnode(''list'')'));
            for i = 1:numel(nodeList)
                nodeName = nodeList{i};
                fprintf(fid, [cr 'Info for node ' nodeName ':' cr]);
                fprintf(fid, evalc('rosnode(''info'', nodeName)'));
            end

            % rostopic list
            fprintf(fid, [cr, sp, 'ROS Topic Information', sp, cr, cr]);
            topicList = rostopic('list');
            fprintf(fid, ['Topic List:' cr]);
            fprintf(fid, evalc('rostopic(''list'')'));
            for i = 1:numel(topicList)
                topicName = topicList{i};
                fprintf(fid, [cr 'Info for topic ' topicName ':' cr]);
                fprintf(fid, evalc('rostopic(''info'', topicName)'));
            end

            % rosservice list
            fprintf(fid, [cr, sp, 'ROS Service Information', sp, cr, cr]);
            serviceList = rosservice('list');
            fprintf(fid, ['Service List:' cr]);
            fprintf(fid, evalc('rosservice(''list'')'));
            for i = 1:numel(serviceList(:,1))
                serviceName = serviceList{i,1};
                fprintf(fid, [cr 'Info for service ' serviceName ':' cr]);
                fprintf(fid, evalc('rosservice(''info'', serviceName)'));
            end

            % Available parameters
            fprintf(fid, [cr, sp, 'ROS Parameter Information', sp, cr, cr]);
            fprintf(fid, ['Parameter List:' cr]);
            paramTree = rosparam;
            paramList = paramTree.AvailableParameters;
            fprintf(fid, strjoin(paramList,'\n'));

            % rostime
            fprintf(fid, [cr, sp, 'ROS Time Information', sp, cr, cr]);
            fprintf(fid, [cr 'Current ROS Time:' cr]);
            fprintf(fid, evalc('rostime(''now'')'));
            fprintf(fid, [cr 'Current System Time:' cr]);
            fprintf(fid, [evalc('rostime(''now'', ''system'')') cr]);

            fprintf(fid, [cr, sp, 'ROS Tf Information', sp, cr, cr]);

            % Gather transformation tree date for 1 second
            tf = rostf;
            pause(1);

            % Print all available transformations
            fprintf(fid, [cr 'Available Tf frames:' cr]);
            fprintf(fid, [evalc('tf.AvailableFrames') cr]);
            availableFrames = tf.AvailableFrames;
            numAvailableFrames = length(availableFrames);
            for i = 1:numAvailableFrames
                for j = i:numAvailableFrames
                    if i == j
                        continue;
                    end

                    % Get transformation
                    frame1 = availableFrames{i};
                    frame2 = availableFrames{j};
                    transform = tf.getTransform(frame1, frame2);
                    if ~isempty(transform)
                        fprintf(fid, [cr 'Transformation between ' frame1 ' and ' frame2 ':' cr]);
                        fprintf(fid, [transform.showdetails cr]);
                    end
                end
            end

        catch ex
            fprintf(fid, ['[ROSSUPPORT ERROR] Error retrieving ROS network information.' cr]);
            fprintf(fid, strrep(ex.getReport, '\', '\\'));
        end
    else
        fprintf(fid, ['Global node is not running.' cr]);
    end

    try
        % Print information about network interfaces
        fprintf(fid, [cr, sp, 'NETWORK INTERFACES', sp, cr, cr]);
        ipAddresses = ros.internal.Net.getAllIPv4Addresses(true);
        for i = 1:length(ipAddresses)
            s = ipAddresses(i);
            fprintf(fid, ['Interface ' num2str(i) ':' cr]);
            fprintf(fid, ['\t', 'IP Address: ' s.ipAddress cr]);
            fprintf(fid, ['\t', 'Host Name: ' s.hostName cr]);
            fprintf(fid, ['\t', 'Interface Name: ' s.interfaceName cr]);
            fprintf(fid, ['\t', 'Subnet Length: ' num2str(s.subnetLength) cr]);
        end
    catch ex
        fprintf(fid, ['[ROSSUPPORT ERROR] Error retrieving network interfaces.' cr]);
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end


    try
        % Get Python and code generation related information
        fprintf(fid, [cr, sp, 'PYTHON AND CODEGEN INFORMATION', sp, cr, cr]);
        % Boost shipped with product, but environment variable can override
        boostRootEnv = getenv('BOOST_ROOT');
        fprintf(fid, 'BOOST_ROOT = %s\n', boostRootEnv);
        % Catkin tools installed into Python virtual environment, but
        % environment variable can override
        catkinPrefixPathEnv = getenv('CATKIN_PREFIX_PATH');
        fprintf(fid, 'CATKIN_PREFIX_PATH = %s\n', catkinPrefixPathEnv);
        myPython2Venv = getenv('MY_PYTHON2_VENV');
        % Overriding location of Python virtual environment - only for testing
        fprintf(fid, 'MY_PYTHON2_VENV = %s\n', myPython2Venv);
        % Also check PYTHONPATH captured in all environment variables
        pythonHomeEnv = getenv('PYTHONHOME');
        fprintf(fid, 'PYTHONHOME = %s\n', pythonHomeEnv);
        % Which tools will MATLAB use
        fprintf(fid, 'Mex Compiler Configuration:\n');
        compilerConfig = evalc('disp(mex.getCompilerConfigurations(''C++''))');
        fprintf(fid, '%s', compilerConfig);
        catkinPrefixPath = ros.internal.getCatkinPrefixPath;
        fprintf(fid, 'Using catkin prefix path: %s\n', catkinPrefixPath);
        boostRoot = ros.internal.getBoostRootPath;
        fprintf(fid, 'Using boost root: %s\n', boostRoot);
        fprintf(fid, 'pyenv:\n');
        pyenvInfo = evalc('disp(pyenv)');
        fprintf(fid, '%s', pyenvInfo);
        % System definitions for executables (locations and versions)
        if ispc
            whereVerb = 'where';
            cmakeVerionVerb = 'cmake --version';
        else
            whereVerb = 'which';
            cmakeVerionVerb = 'bash -c ''LD_LIBRARY_PATH= cmake --version''';
        end
        [~, wherePython] = system([whereVerb ' python']);
        fprintf(fid, '%s python = \n%s\n', whereVerb, wherePython);
        [~, wherePython2] = system([whereVerb ' python2']);
        fprintf(fid, '%s python2 = \n%s\n', whereVerb, wherePython2);
        [~, pythonCmdVersion] = system('python --version');
        fprintf(fid, 'python --version = %s\n', pythonCmdVersion);
        [~, python2CmdVersion] = system('python2 --version');
        fprintf(fid, 'python2 --version = %s\n', python2CmdVersion);
        [~, whereCmake] = system([whereVerb ' cmake']);
        fprintf(fid, '%s cmake = \n%s\n', whereVerb, whereCmake);
        [~, cmakeCmdVersion] = system(cmakeVerionVerb);
        fprintf(fid, 'cmake --version = %s\n', cmakeCmdVersion);
    catch ex
        fprintf(fid, '[ROSSUPPORT ERROR] Error retrieving Python and codegen information.\n');
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end


    try
        % Get information about registered custom messages
        fprintf(fid, [cr, sp, 'CUSTOM MESSAGE INFORMATION', sp, cr, cr]);
        customMsgRegistry = ros.internal.CustomMessageRegistry.getInstance('ros');
        msgList = getMessageList(customMsgRegistry);
        srvList = getServiceList(customMsgRegistry);
        % Blank lists simply mean no custom messages/services registered
        fprintf(fid, 'Custom Messages:\n');
        fprintf(fid, '%s\n', msgList{:});
        fprintf(fid, 'Custom Services:\n');
        fprintf(fid, '%s\n', srvList{:});
    catch ex
        fprintf(fid, '[ROSSUPPORT ERROR] Error retrieving custom message registration.\n');
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end


    % The following information is more general and not ROS specific
    % Print all environment variables that are defined
    fprintf(fid, [cr, sp, 'ALL ENVIRONMENT VARIABLES', sp, cr, cr]);
    if ispc
        cmd = 'set';
    else
        cmd = 'env';
    end
    [~,out] = system(cmd);
    vars = regexp(strtrim(out), '^(.*)=(.*)$', 'tokens', ...
                  'lineanchors', 'dotexceptnewline');
    vars = vertcat(vars{:});
    m = containers.Map(upper(vars(:,1)), vars(:,2));
    keys = m.keys;
    for i = 1:numel(keys)
        fprintf(fid, '%s = %s\n', keys{i}, m(keys{i}));
    end

    % MATLABROOT directory
    fprintf(fid, [cr, sp, 'MATLAB ROOT DIRECTORY', sp, cr, cr]);
    if exist('ctfroot', 'builtin')
        mlRoot = ctfroot;
    else
        mlRoot = matlabroot;
    end
    fprintf(fid, ['\t%s', cr], mlRoot);

    % Preferences
    try
        fprintf(fid, [cr, sp, 'PREFERENCES', sp, cr, cr]);
        fprintf(fid, 'Preferences Directory:\n');
        fprintf(fid, ['\t%s', cr], prefdir);
        fprintf(fid, 'ROS_Toolbox Preferences:\n');
        rosPrefStruct = getpref('ROS_Toolbox');
        fprintf(fid, '%s', evalc('disp(rosPrefStruct)'));
        if isfield(rosPrefStruct, 'ROS_NetworkAddress_Profiles')
            networkProfiles = rosPrefStruct.ROS_NetworkAddress_Profiles;
            for k = 1:numel(networkProfiles)
                profileInfo = evalc('disp(networkProfiles{k})');
                fprintf(fid, 'ROS_NetworkAddress_Profiles %d:\n%s', k, profileInfo);
            end
        end
    catch ex
        fprintf(fid, ['[ROSSUPPORT ERROR] Error retrieving preferences information.' cr]);
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end

    % MATLAB path
    fprintf(fid, [cr, sp, 'MATLAB PATH', sp, cr, cr]);
    fprintf(fid, '%s', evalc('path'));

    % MATLAB Java classpath
    fprintf(fid, [cr, sp, 'JAVA CLASSPATH', sp, cr, cr]);
    fprintf(fid, '%s', evalc('javaclasspath'));


    % CPU information
    try
        fprintf(fid, [cr, sp, 'CPU/SYSTEM INFORMATION', sp, cr, cr]);
        if ispc
            fprintf(fid, [cr, sp, sp, 'CPU INFO', sp, sp, cr, cr]);
            try
                NET.addAssembly('mscorlib');
                localMachineRoot = Microsoft.Win32.Registry.LocalMachine;
                cpuParent = localMachineRoot.OpenSubKey('HARDWARE\DESCRIPTION\SYSTEM\CentralProcessor');
                processors = cpuParent.GetSubKeyNames();
                totalLogicalCPUs = cpuParent.SubKeyCount();
                for cpuid = 1:totalLogicalCPUs
                    fprintf(fid, ['CPU %d: %s' cr], cpuid, cpuParent.OpenSubKey(processors(cpuid)).GetValue('ProcessorNameString').char);
                end
            catch %#ok<CTCH>
            end

            fprintf(fid, [cr, sp, sp, 'SYSTEM INFO', sp, sp, cr, cr]);
            [~, systemInfo]=system('systeminfo');
            fprintf(fid, sprintf('%s', strrep(systemInfo, '\', '\\')));
        elseif ismac
            fprintf(fid, [cr, sp, sp, 'SYSTEM_PROFILER', sp, sp, cr, cr]);
            [~, systemInfo]=system('system_profiler SPHardwareDataType SPNetworkDataType SPEthernetDataType SPFireWireDataType SPFirewallDataType SPUSBDataType');
            fprintf(fid, sprintf('%s', systemInfo));

            fprintf(fid, [cr, sp, sp, 'SYSCTL', sp, sp, cr, cr]);
            [~, systemInfo]=system('sysctl -a kern.ipc.maxsockbuf');
            fprintf(fid, sprintf('%s', systemInfo));
            [~, systemInfo]=system('sysctl -a net.inet.udp.recvspace');
            fprintf(fid, sprintf('%s', systemInfo));
        else % Linux
            fprintf(fid, [cr, sp, sp, '/PROC/CPUINFO', sp, sp, cr, cr]);
            [~, systemInfo]=system('cat /proc/cpuinfo');
            fprintf(fid, sprintf('%s', systemInfo));

            fprintf(fid, [cr, sp, sp, '/LIB/LIBC.SO.6', sp, sp, cr, cr]);
            [~, systemInfo]=system('/lib/libc.so.6 -version');
            fprintf(fid, sprintf('%s', systemInfo));

            fprintf(fid, [cr, sp, sp, '/PROC/SYS/KERNEL/TAINTED', sp, sp, cr, cr]);
            [~, systemInfo]=system('cat /proc/sys/kernel/tainted');
            fprintf(fid, sprintf('%s', systemInfo));

            fprintf(fid, [cr, sp, sp, 'SYSCTL', sp, sp, cr, cr]);
            [~, systemInfo]=system('sysctl net.core.rmem_default');
            fprintf(fid, sprintf('%s', systemInfo));
            [~, systemInfo]=system('sysctl net.core.rmem_max');
            fprintf(fid, sprintf('%s', systemInfo));

            fprintf(fid, [cr, sp, sp, 'DMESG for USB', sp, sp, cr, cr]);
            [~, systemInfo]=system('dmesg | grep -i usb');
            fprintf(fid, sprintf('%s', systemInfo));
        end
    catch ex
        fprintf(fid, ['[ROSSUPPORT ERROR] Error retrieving CPU and system information.' cr]);
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end

    % Dynamic loader path information:
    %       on Windows this is PATH
    %       on Linux this is LD_LIBRARY_PATH
    %       on Mac OS X this is LD_LIBRARY_PATH, DYLD_LIBRARY_PATH, or
    %           DYLD_FALLBACK_LIBRARY_PATH
    try
        fprintf(fid, [cr, sp, 'DYNAMIC LOADER PATH', sp, cr, cr]);
        fprintf(fid, [cr, sp, sp, 'PATH', sp, sp, cr, cr]);
        fprintf(fid, '%s', strrep(getenv('PATH'), pathsep, cr));
        fprintf(fid, [cr, sp, sp, 'LD_LIBRARY_PATH', sp, sp, cr, cr]);
        fprintf(fid, '%s', [strrep(getenv('LD_LIBRARY_PATH'), pathsep, cr), cr]);
        fprintf(fid, [cr, sp, sp, 'DYLD_LIBRARY_PATH', sp, sp, cr, cr]);
        fprintf(fid, '%s', [strrep(getenv('DYLD_LIBRARY_PATH'), pathsep, cr), cr]);
        fprintf(fid, [cr, sp, sp, 'DYLD_FALLBACK_LIBRARY_PATH', sp, sp, cr, cr]);
        fprintf(fid, '%s', [strrep(getenv('DYLD_FALLBACK_LIBRARY_PATH'), pathsep, cr), cr]);
    catch ex
        fprintf(fid, ['[ROSSUPPORT ERROR] Error retrieving dynamic path information.' cr]);
        fprintf(fid, strrep(ex.getReport, '\', '\\'));
    end


    % End of the test
    fprintf(fid, [cr, sp, sp,'END ROS SUPPORT', sp, sp, cr]);
    fprintf(fid, [cr, 'This information has been saved in the text file: ', cr, ...
                  '%s', cr], filepath);
    fprintf(fid, [cr, 'If any errors occurred, please e-mail this information to:', cr, ...
                  'support@mathworks.com', cr]);

    % Display text output to user
    if (nargout == 0)
        if ~isdeployed
            edit(filepath);
        end
    else
        fileOut = which(filepath);
    end

    disp(message('ros:mlros:support:GenerationComplete').getString);
    disp(' ');
    disp(message('ros:mlros:support:PrivacyNotice', filepath).getString);

end

% LocalWords:  FILEOUT sp NODEHOST lineanchors dotexceptnewline mscorlib systeminfo SPUSB SYSCTL
% LocalWords:  sysctl kern ipc maxsockbuf recvspace CPUINFO cpuinfo LIBC libc rmem DMESG dmesg usb
