function map = getSpecialMapForROS
%getSpecialMap returns the map which contains data structure for 'time', 'duration' and 'Header'.

%   Copyright 2020-2021 The MathWorks, Inc.

    dataTypes = {'time','duration','Header'};
    structForTime = getStructForTime;
    structForDuration = getStructForDuration;
    structForHeader = getStructForHeader;
    dataStructure = {structForTime,structForDuration,structForHeader};

    map = containers.Map(dataTypes,dataStructure);
    map('special_config_handler') = @addSpecialConfig;
end

function timeStruct = getStructForTime
%getStructForTime returns the datastructure for 'time'.

    timeStruct.MessageType = 'ros/Time';

    timeStruct.msgFields.sec.MLdataType = 'uint32';
    timeStruct.msgFields.sec.CPPdataType = 'uint32_t';
    timeStruct.msgFields.sec.ROSdataType = 'uint32';

    timeStruct.msgFields.nsec.MLdataType = 'uint32';
    timeStruct.msgFields.nsec.CPPdataType = 'uint32_t';
    timeStruct.msgFields.nsec.ROSdataType = 'uint32';

    timeStruct.msgFields.sec.count = 0;
    timeStruct.msgFields.sec.constantValue = NaN;
    timeStruct.msgFields.sec.defaultValue = NaN;
    timeStruct.msgFields.sec.varsize = 0;
    timeStruct.msgFields.sec.maxstrlen = NaN;

    timeStruct.msgFields.nsec.count = 0;
    timeStruct.msgFields.nsec.constantValue = NaN;
    timeStruct.msgFields.nsec.defaultValue = NaN;
    timeStruct.msgFields.nsec.varsize = 0;
    timeStruct.msgFields.nsec.maxstrlen = NaN;

    timeStruct.count = 0;
    timeStruct.constantValue = NaN;
    timeStruct.defaultValue = NaN;
    timeStruct.varsize = 0;
end

function durationStruct = getStructForDuration
%getStructForDuration returns the datastructure for 'duration'.

    durationStruct.MessageType = 'ros/Duration';

    durationStruct.msgFields.sec.MLdataType = 'int32';
    durationStruct.msgFields.sec.CPPdataType = 'int32_t';
    durationStruct.msgFields.sec.ROSdataType = 'int32';

    durationStruct.msgFields.nsec.MLdataType = 'int32';
    durationStruct.msgFields.nsec.CPPdataType = 'int32_t';
    durationStruct.msgFields.nsec.ROSdataType = 'int32';

    durationStruct.msgFields.sec.count = 0;
    durationStruct.msgFields.sec.constantValue = NaN;
    durationStruct.msgFields.sec.defaultValue = NaN;
    durationStruct.msgFields.sec.varsize = 0;
    durationStruct.msgFields.sec.maxstrlen = NaN;

    durationStruct.msgFields.nsec.count = 0;
    durationStruct.msgFields.nsec.constantValue = NaN;
    durationStruct.msgFields.nsec.defaultValue = NaN;
    durationStruct.msgFields.nsec.varsize = 0;
    durationStruct.msgFields.nsec.maxstrlen = NaN;

    durationStruct.count = 0;
    durationStruct.constantValue = NaN;
    durationStruct.defaultValue = NaN;
    durationStruct.varsize = 0;
end

function headerStruct = getStructForHeader
%getStructForHeader returns the datastructure for 'Header'.

    headerStruct.MessageType = 'std_msgs/Header';

    ROSInstallPrefixPath = ros.internal.ros.getROSInstallPrefixPath;
    headerStruct.filePath = fullfile(ROSInstallPrefixPath,'share','std_msgs','msg','Header.msg');

    headerStruct.msgFields.seq.MLdataType = 'uint32';
    headerStruct.msgFields.seq.CPPdataType = 'uint32_t';
    headerStruct.msgFields.seq.ROSdataType = 'uint32';

    headerStruct.msgFields.seq.count = 0;
    headerStruct.msgFields.seq.constantValue = NaN;
    headerStruct.msgFields.seq.defaultValue = NaN;
    headerStruct.msgFields.seq.varsize = 0;
    headerStruct.msgFields.seq.maxstrlen = NaN;

    headerStruct.msgFields.stamp = getStructForTime;

    headerStruct.msgFields.frame_id.MLdataType = 'string';
    headerStruct.msgFields.frame_id.CPPdataType = 'std::string';
    headerStruct.msgFields.frame_id.ROSdataType = 'string';

    headerStruct.msgFields.frame_id.count = 0;
    headerStruct.msgFields.frame_id.constantValue = NaN;
    headerStruct.msgFields.frame_id.defaultValue = NaN;
    headerStruct.msgFields.frame_id.varsize = 0;
    headerStruct.msgFields.frame_id.maxstrlen = NaN;

    headerStruct.count = 0;
    headerStruct.constantValue = NaN;
    headerStruct.defaultValue = NaN;
    headerStruct.varsize = 0;
end

function addSpecialConfig(parser)
    % ROS1 byte equivalent in Cpp is int8_t and
    % ROS1 char equivalent in Cpp is uint8_t
    % ref: http://wiki.ros.org/msg
    parser.CPPTypeConverter('byte') = 'int8_t';
    parser.MLTypeConverter('byte') = 'int8';
    parser.CPPTypeConverter('char') = 'uint8_t';
    parser.MLTypeConverter('char') = 'uint8';
end