function varargout = rosWriteCameraInfo(msg, params, varargin)
%rosWriteCameraInfo Write data from stereoParameters or cameraParameters
%structure to ROS message.
%   MSGOUT = rosWriteCameraInfo(MSG,CAMERAPARAMS) writes data from the
%   camera parameter structure, CAMERAPARAMS, to a sensor_msgs/CameraInfo
%   message structure, MSG.
%
%   [MSGOUT1,MSGOUT2] = rosWriteCameraInfo(MSG,STEREOPARAMS) writes data
%   from the stereo parameter structure, STEREOPARAMS, to
%   sensor_msgs/CameraInfo messages MSG1 and MSG2. If the stereo parameters
%   do not correspond to a rectified camera, you must call
%   rectifyStereoImages before using this syntax.
%
%   Write Camera Parameters to ROS Message:
%   % Create a set of calibration images.
%   images = imageDatastore(fullfile(toolboxdir("vision"),"visiondata", ...
%     "calibration","mono"));
%   imageFileNames = images.Files;
%
%   % Detect calibration pattern.
%   [imagePoints,boardSize] = detectCheckerboardPoints(imageFileNames);
%
%   % Generate world coordinates of the corners of the squares.
%   squareSize = 29; % millimeters
%   worldPoints = generateCheckerboardPoints(boardSize,squareSize);
%
%   % Calibrate the camera.
%   I = readimage(images,1);
%   imageSize = [size(I,1),size(I,2)];
%   params = estimateCameraParameters(imagePoints,worldPoints, ...
%                                     ImageSize=imageSize);
%
%   % Create a sensor_msgs/CameraInfo message
%   msg = rosmessage("sensor_msgs/CameraInfo","DataFormat","struct");
%
%   % Write the params data to the msg
%   msg = rosWriteCameraInfo(msg,toStruct(params));
%
%
%   Write Stereo Parameters to ROS Messages:
%   % Specify images containing a checkerboard for calibration
%   imageDir = fullfile(toolboxdir("vision"),"visiondata", ...
%       "calibration","stereo");
%   leftImages = imageDatastore(fullfile(imageDir,"left"));
%   rightImages = imageDatastore(fullfile(imageDir,"right"));
%
%   % Detect the checkerboards
%   [imagePoints,boardSize] = detectCheckerboardPoints(...
%       leftImages.Files,rightImages.Files);
%
%   % Specify world coordinates of checkerboard keypoints
%   squareSizeInMillimeters = 108;
%   worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMillimeters);
%
%   % Read in the images
%   I1 = readimage(leftImages,1);
%   I2 = readimage(rightImages,1);
%   imageSize = [size(I1, 1),size(I1, 2)];
%
%   % Calibrate the stereo camera system
%   stereoParams = estimateCameraParameters(imagePoints,worldPoints, ...
%                                           ImageSize=imageSize);
%
%   % Rectify the images using "full" output view
%   [J1_full,J2_full] = rectifyStereoImages(I1,I2,stereoParams, ...
%     "OutputView","full");
%
%   % Create a sensor_msgs/CameraInfo message
%   msg = rosmessage("sensor_msgs/CameraInfo","DataFormat","struct");
%
%   % Write the stereoParms data to the msg1 and ms2
%   [msg1,msg2] = rosWriteCameraInfo(msg,toStruct(stereoParams));


%   Copyright 2021 The MathWorks, Inc.
%#codegen

% Validate Input argument and create local fields for ROS or ROS 2
% sensor_msgs/CameraInfo

% Ensure the generated code is not inlined
    coder.inline('never');

    validateattributes(msg, {'struct'},{'scalar'},'rosWriteCameraInfo');
    validateattributes(params, {'struct'},{'scalar'},'rosWriteCameraInfo');

    % Code generation only supports sensor_msgs/CameraInfo as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType, 'sensor_msgs/CameraInfo'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosWriteCameraInfo','sensor_msgs/CameraInfo');
    end

    if isfield(params, 'RectificationParams') && ~params.RectificationParams.Initialized
        coder.internal.error('ros:utilities:camerainfo:callRectifyFirst', 'stereoParams')
    end

    if isfield(msg, 'Height')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    varargout = specialMsgUtil.writeCameraInfo(params, msg, msg, varargin{:});
end
