classdef ReadData < ros.slros.internal.block.ReadLogFileBase
    % This class is for internal use only. It may be removed in the future.
    
    %Play back data from a supported logfile
    %
    %   H = ros.slros2.internal.block.ReadData creates a system
    %   object, H, that accesses a supported logfile and plays back data
    %   from a topic.
    %
    %   This system object is intended for use with the MATLAB System
    %   block. In order to access the rosbag2 playback functionality from
    %   MATLAB, see ros2bag.

    %   Copyright 2020-2021 The MathWorks, Inc.
    
    properties (Nontunable)
        %LogfileType Logfile type
        %   Type of logfile will be displayed on front of the block
        LogfileType = 'rosbag';
    end    
    
    properties(Access=protected)
        %BusUtilObj Bus utility object used with Simulink bus conversion
        BusUtilObj = ros.slroscpp.internal.bus.Util;
    end
    
    properties(Constant,Access=protected)
        %TimeFactor Multiplication-factor for converting Simulink time to
        %match Logfile time stamps
        % For ROSBAG2 the time stamps are in nanoseconds, uint64
        %
        TimeFactor = 1;
    end    
    
    properties (Access = protected, Transient)
        %DataObject Object containing or allowing for access to all logfile data
        DataObject = ros.slros.internal.block.ReadData.getEmptyDataObject();

        %DataSelection Object containing or allowing access to applicable logfile data
        % (contained in topic, after offset, and before duration completes
        DataSelection = ros.slros.internal.block.ReadData.getEmptyDataObject();


        %Converter Converts from logfile messages to struct for output to bus
        Converter = ros.slroscpp.internal.sim.ROSMsgToBusStructConverter.empty();

    end
    
    methods (Access=protected)
        function ret = convertMessage(obj,msg)
            ret = convert(obj.Converter, toROSStruct(msg));
        end
    end    
    
    methods (Static,Hidden)
        function ret = isValidLogdataObject(logObj)
            ret = isa(logObj,'ros.BagSelection');
        end
        
        function ret = getOutputDatatypeString(msgType,modelName)
            ros.slroscpp.internal.bus.Util.createBusIfNeeded(msgType,modelName);
            [~,ret] = ros.slroscpp.internal.bus.Util.rosMsgTypeToDataTypeStr(msgType,modelName);
        end
        
        function clearBusesOnModelClose(block)
            ros.slros.internal.bus.clearBusesOnModelClose(block);
        end
        
        function ret = getBlockIcon()
            ret = 'rosicons.robotlib_readdata';
        end
        
        function ret = getLogFileExtension()
            ret = '*.bag';
        end
        
        function ret = getEmptyMessage(msgType)
            ret = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
        end        
        
        function ret = getEmptyDataObject()
            ret = ros.BagSelection.empty();
        end
                
        function ret = getBusConverterObject(msgType,modelName)
            ret = ros.slroscpp.internal.sim.ROSMsgToBusStructConverter(msgType,...
                modelName);
        end
    end
end
