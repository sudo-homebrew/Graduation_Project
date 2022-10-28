function gazebogenmsg(folderPath,varargin)
%GAZEBOGENMSG Generate dependencies for Gazebo custom message support
%  GAZEBOGENMSG(FOLDERPATH) generates the required dependencies for Gazebo
%  Custom Message Support. FOLDERPATH is the path to the folder that contains
%  the definitions of the custom messages ('.proto' files). This function
%  expects one or more '.proto' files in a single folder.
%
%  GAZEBOGENMSG(FOLDERPATH, NAME, VALUE) allows user to use Gazebo built-in
%  message. It generates the required dependencies based on selected message
%  FOLDERPATH is the path to the folder that may contains custom message
%  '.proto' file.
%
%  Name-Value:
%
%  GazeboVersion     - Select Gazebo message version from list specified
%                      as a string. E.g. "Gazebo 9" or "Gazebo 10" or
%                      "Gazebo 11"
%  GazeboMessageList - Select Gazebo message name from list, specified as
%                      string or string array. To get the list of valid Gazebo
%                      messages, press the Tab key after the GazeboMessageList
%                      argument name. The list expands, and you can scroll
%                      to choose the values. E.g. "gazebo.msgs.Image" or
%                      ["gazebo.msgs.Image", "gazebo.msgs.IMU"
%
%  After calling the GAZEBOGENMSG function, execute the commands presented
%  at the end of the function call to add the install folder to the search
%  path.
%
%  Required steps to use the Gazebo Custom Message functionality after
%  calling the GAZEBOGENMSG function,
%    1. Call the PACKAGEGAZEBOPLUGIN function to package the plugin.
%    2. Copy, install and run the plugin on the Gazebo machine.
%    3. Use Gazebo Publish Simulink(R) block to send the custom messages
%       to the Gazebo machine.
%    4. Use Gazebo Subscribe Simulink(R) block to receive the custom
%       messages from the Gazebo machine.
%
%  Note :
%     It is safe to call the GAZEBOGENMSG function multiple times before
%     loading the generated dependencies into any Simulink model. All
%     messages under FOLDERPATH will be rebuilt on each function call.
%
%  Example:
%       % User-defined Custom Message:
%       %   Create a folder in a local directory
%           folderPath = fullfile(pwd,'customMessage');
%           mkdir(folderPath)
%       %   Create one or more '.proto' file inside the folder and
%       %   define protobuf custom message fields
%           messageDefinition = {'message MyPose'
%                               '{'
%                               '   required double x = 1;'
%                               '   required double y = 2;'
%                               '   required double z = 3;'
%                               '}'};
%           fileID = fopen(fullfile(folderPath,'MyPose.proto'),'w');
%           fprintf(fileID,'%s\n',messageDefinition{:}) ;
%           fclose(fileID);
%       %   Run the gazebogenmsg function and specify the folder path
%           gazebogenmsg(folderPath)
%       %   The following commands are presented at the end of the
%       %   function call
%           addpath(fullfile(folderPath,'install'))
%           savepath
%
%       % Gazebo Built-in Message:
%       %   Create a folder in a local directory
%           folderPath = fullfile(pwd,'customMessage');
%           mkdir(folderPath)
%       %   Specify the folder path,select gazebo message name
%       %   and gazebo version (optional)
%           gazebogenmsg(folderPath,"GazeboVersion","Gazebo 9",...
%           "GazeboMessageList","gazebo.msgs.Image")
%       %   The following commands are presented at the end of the
%       %   function call
%           addpath(fullfile(folderPath,'install'))
%           savepath
%
%   References:
%       [1] Protobuf ("proto2" type) Message Definition Guideline
%           https://developers.google.com/protocol-buffers/docs/proto
%
%   See also packageGazeboPlugin.

%   Copyright 2019-2021 The MathWorks, Inc.

    narginchk(1,5);

    disp(message('robotics:robotgazebo:gazebogenmsg:ValidatingMessage').getString);

    folderPath = convertStringsToChars(folderPath);

    validateattributes(folderPath, {'char'}, {'scalartext', 'nonempty'}, 'gazebogenmsg', 'folderPath');

    if (~exist(folderPath,'dir'))
        error(message('robotics:robotgazebo:gazebogenmsg:InvalidInputFolderPath', folderPath));
    end

    p = inputParser;
    addParameter(p,'GazeboMessageList', {});
    addParameter(p,'GazeboVersion', {});
    parse(p,varargin{:});

    % argument validation and retrieve gazebo message '.proto' files names and version
    if( ~isempty(p.Results.GazeboMessageList) )

        if(isempty(p.Results.GazeboVersion))
            % default gazebo version
            gazeboVersionName = "Gazebo 9";
        else
            validateattributes(p.Results.GazeboVersion, {'char','string'}, {'scalartext', 'nonempty'},...
                               'gazebogenmsg', 'GazeboVersion');

            % get input gazebo version name
            gazeboVersionName = p.Results.GazeboVersion;
            % get expected gazebo version name
            expectedGazeboVersionName = robotics.gazebo.internal.customMessageSupport.getGazeboVersionList;
            % error out if input gazebo version is not matched with
            % expected version
            if(~nnz(strcmp(gazeboVersionName,expectedGazeboVersionName)))
                errorMsg = char(join(expectedGazeboVersionName,' or '));
                error(message('robotics:robotgazebo:gazebogenmsg:InvalidGazeboVersion', errorMsg));
            end

        end

        validateattributes(p.Results.GazeboMessageList, {'string'}, {'nonempty','row'},...
                           'gazebogenmsg', 'GazeboMessageList');

        % get input gazebo message list
        gazeboMessageList = p.Results.GazeboMessageList;

        % get expected gazebo message list
        expectedGazeboMessageList= robotics.gazebo.internal.customMessageSupport.getGazeboMsgsList;

        % validate selected message type
        for msgId = 1: length(gazeboMessageList)
            gazeboMessageName = gazeboMessageList{msgId};
            if(~nnz(strcmp(gazeboMessageName,expectedGazeboMessageList)))
                error(message('robotics:robotgazebo:gazebogenmsg:InvalidGazeboMessage'));
            end
        end
        % copy selected gazebo message to input folder
        robotics.gazebo.internal.customMessageSupport.copyGazeboMsgsProtoNames...
            (folderPath, gazeboMessageList, gazeboVersionName, expectedGazeboMessageList);
    end

    % check shared library loaded and get shared library name
    sharedLibName = robotics.gazebo.internal.customMessageSupport.validateAndGetLibraryName();

    % validate installed and selected compiler
    robotics.gazebo.internal.customMessageSupport.compilerValidation();

    % clean and create required folders and returns structure of folder path
    folderPathList = robotics.gazebo.internal.customMessageSupport.createRequiredFolders(folderPath);

    % read proto files list
    protoList = dir(fullfile(folderPath,'*.proto'));

    if(isempty(protoList))
        error(message('robotics:robotgazebo:gazebogenmsg:ProtoFileNotFound', folderPath));
    end

    % create proto and its cpp and headers
    robotics.gazebo.internal.customMessageSupport.createCustomProtoFiles(protoList,folderPathList.protoFilesFolderPath);

    % first create all simulink messages for .proto files
    for pIdx = 1: length(protoList)
        % read proto files and retrieve message map and name
        result{pIdx} = robotics.gazebo.internal.customMessageSupport.readProtoFiles(folderPathList.protoFilesFolderPath,protoList(pIdx).name);
    end

    % retrieve required message details from multiple proto structures
    customDetails = robotics.gazebo.internal.customMessageSupport.getMesageDetails(result);

    % validate same message name defined in multiple files
    robotics.gazebo.internal.customMessageSupport.validateProtoMessageconflict(customDetails);

    for Idx = 1 : length(result)
        % create simulink messages
        robotics.gazebo.internal.customMessageSupport.createSimulinkMessages(result{Idx}, customDetails, folderPathList.simulinkMsgFolderPath);
    end

    % install simulink messages
    robotics.gazebo.internal.customMessageSupport.installSimulinkMessages(folderPath);

    % modify protobuf headers
    robotics.gazebo.internal.customMessageSupport.modifyProtoHeaders(folderPathList.protoFilesFolderPath);

    disp(message('robotics:robotgazebo:gazebogenmsg:BuildingLibMessage').getString);

    % create shared library for protobuf generated source files
    robotics.gazebo.internal.customMessageSupport.createProtoSharedLib(folderPathList,sharedLibName);

    for Idx = 1 : length(customDetails.utilities)

        disp(message('robotics:robotgazebo:gazebogenmsg:BuildingMexMessage',customDetails.utilities{Idx}.protoFileName).getString)

        % create simulink to packet MEX source file
        sourceFileName = robotics.gazebo.internal.customMessageSupport.createSimulinkToPacketMEXSourceFile(customDetails.utilities{Idx},...
                                                          folderPathList.simulinkToPacketFolderPath);
        % build MEX of source file
        robotics.gazebo.internal.customMessageSupport.buildMEXSourceFile(sourceFileName,folderPathList,sharedLibName);

        % create packet to simulink MEX source file
        sourceFileName = robotics.gazebo.internal.customMessageSupport.createPacketToSimulinkMEXSourceFile(customDetails.utilities{Idx},...
                                                          folderPathList.packetToSimulinkFolderPath);
        % build MEX of source file
        robotics.gazebo.internal.customMessageSupport.buildMEXSourceFile(sourceFileName,folderPathList,sharedLibName);

        % create custom message handler files
        robotics.gazebo.internal.customMessageSupport.createCppCustomHandlersSourceFiles(customDetails.utilities{Idx},...
                                                          folderPathList.customMsgHandlerFolderPath);

    end

    disp(message('robotics:robotgazebo:gazebogenmsg:BuildingUtilMessage').getString);

    % create plugin utility files
    robotics.gazebo.internal.customMessageSupport.createPluginSourceFiles(customDetails.utilities,...
                                                      folderPathList.pluginUtilitiesFolderPath);

    % create simulink utility files
    robotics.gazebo.internal.customMessageSupport.createSimulinkUtilitiesFiles(customDetails.utilities,...
                                                      folderPathList.simulinkUtilitiesFolderPath, sharedLibName);

    robotics.gazebo.internal.customMessageSupport.installSimulinkUtilities(folderPathList);

    disp(message('robotics:robotgazebo:gazebogenmsg:DoneMessage').getString);

    displayNextSteps(fullfile(folderPath,'install'));

end

function displayNextSteps(msgFolder)
%displayNextSteps Displays explanation of next user steps
%   User should follow the displayed steps to use Gazebo Custom
%   Message Support.

    addPathString = ['addpath(''' msgFolder ''')'];
    savePathString = 'savepath';

    disp(' ')
    disp(message('robotics:robotgazebo:gazebogenmsg:NextStepsMessage').getString);
    disp(' ')
    disp(addPathString);
    disp(savePathString);

end
