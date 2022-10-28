classdef ImageTopicTable < ros.slros2.internal.TopicTable
%This class is for internal use only. It may be removed in the future.

%ImageTopicTable

%   Copyright 2021 The MathWorks, Inc.

    properties (Constant)
        %AdditionalColumns
        AdditionalColumns = {'ImageEncoding','MaximumImageSize'}

        %ValidMessageTypes
        ValidMessageTypes = {'sensor_msgs/Image', 'sensor_msgs/CompressedImage'}

        %NoTopicsAvailableMsgId - MsgID to use if there are no topics
        %   advertised for any of the ValidMessageTypes.
        NoTopicsAvailableMsgId = ...
            'ros:slros2:topicselector:NoImageTopicsAvailable'
    end

    methods
        function obj = ImageTopicTable(varargin)
        %ImageTopicTable Constructor
        %   OBJ = ImageTopicTable(NODE) creates an ImageTopicTable using
        %   the ros2node object, NODE. The table will stop
        %   updating and kill all remaining subscribers if 30 seconds
        %   elapse without a message being received by any subscriber.
        %   If no image topics are found on the ROS 2 network, the
        %   constructor will error out.
        %
        %   OBJ = ImageTopicTable(NODE, infoChangeFcn) creates an
        %   ImageTopicTable that invokes infoChangeFcn each time the
        %   table is updated.
        %
        %   OBJ = ImageTopicTable(NODE, infoChangeFcn, TIMEOUT) creates
        %   an ImageTopicTable that stops updating and kills all
        %   subscribers if TIMEOUT seconds elapse without a message
        %   being received by any subscriber.
        %
        %   OBJ = ImageTopicTable(NODE, infoChangeFcn, TIMEOUT,
        %   allowEmptyTable) creates an ImageTopicTable with zero rows
        %   if allowEmptyTable is true and no image topics are found.
            obj = obj@ros.slros2.internal.TopicTable(varargin{:});
        end

        function [status, enc, imgSize] = extractData(obj,msg)
            enc = obj.UnknownValueString;
            imgSize = obj.UnknownValueString;
            status = message('ros:slros2:topicselector:Valid').string;
            if isfield(msg,'format') % CompressedImage
                format = msg.format;
                parts = strsplit(format, ';');
                if numel(parts) ~= 2
                    status = message('ros:mlros:image:FormatNoSemicolon', ...
                                     format).string;
                    return;
                end
                enc = lower(parts{1});
                if ~ismember(enc, {'rgb8', 'rgba8', 'bgr8', 'bgra8','mono8'})
                    status = message('ros:mlros:image:InvalidEncoding').string;
                    return
                end
                try
                    img = rosReadImage(msg);
                    imgSize = sprintf('[%d, %d]', size(img,1), size(img,2));
                catch ME
                    status = ME.message;
                end
            else % Image
                imgSize = sprintf('[%d, %d]', msg.height, msg.width);
                enc = lower(msg.encoding);
                if ~obj.isSupportedEncoding(enc)
                    status = message('ros:mlros:image:InvalidEncoding').string;
                end
            end
        end

        function out = isSupportedEncoding(~, enc)
        %isSupportedEncoding Check if this encoding is supported
            out = ros.slros2.internal.block.ReadImage.ImageEncodingSet.isAllowedValue(enc);
        end
    end
end
