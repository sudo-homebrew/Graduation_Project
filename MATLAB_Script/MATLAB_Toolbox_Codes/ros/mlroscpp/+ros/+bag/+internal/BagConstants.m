classdef BagConstants
    %This class is for internal use only. It may be removed in the future.
    
    %BagConstants This class gathers constants used for rosbag parsing
    %   These constants are encountered when parsing the rosbag file
    %   format. Only the version 2.0 format is supported. Please see 
    %   http://wiki.ros.org/Bags/Format/2.0 for more information on the
    %   defined constants.
    
    %   Copyright 2014-2018 The MathWorks, Inc.
    
    properties(Constant)        
        % Header Fields
        
        %OpFieldName - The record type identifier
        OpFieldName                 = 'op';
        
        %TopicFieldName - Topic on which messages were received
        TopicFieldName              = 'topic';
        
        %VerFieldName - The version number for index and chunks
        VerFieldName                = 'ver';
        
        %CountFieldName - Number of messages in preceding chunk
        CountFieldName              = 'count';
        
        %IndexPosFieldName - Offset of first record after the chunk section
        IndexPosFieldName           = 'index_pos';
        
        %ConnectionCountFieldName - Number of unique connections in the file
        ConnectionCountFieldName    = 'conn_count';
        
        %ChunkCountFieldName - Number of chunk records in the file
        ChunkCountFieldName         = 'chunk_count';
        
        %ConnectionFieldName - Connection ID
        ConnectionFieldName         = 'conn';
        
        %CompressionFieldName - Compression type for the data
        CompressionFieldName        = 'compression';
        
        %SizeFieldName - Size (in bytes) of the uncompressed chunk
        SizeFieldName               = 'size';
        
        %TimeFieldName - Time at which the message was received
        TimeFieldName               = 'time';
        
        %StartTimeFieldName - Timestamp of first message in the chunk
        StartTimeFieldName          = 'start_time';
        
        %EndTimeFieldName - Timestamp of last message in the chunk
        EndTimeFieldName            = 'end_time';
        
        %ChunkPosFieldName - Offset of the chunk record
        ChunkPosFieldName           = 'chunk_pos';
        
        %OffsetFieldName - Offset of message data record
        %   This offset is in the uncompressed chunk data.
        OffsetFieldName             = 'offset';
        
        % Op Field Values
        
        %OpMsgData - Stores the serialized message data
        OpMsgData       = hex2dec('02');
        
        %OpFileHeader - Stores information about the entire bag
        %   Information includes the offset to the first index data record, 
        %   and the number of chunks and connections.
        OpFileHeader    = hex2dec('03');
        
        %OpIndexData - Stores an index of messages in a single connection
        %   This is in respect to the preceding chunk.
        OpIndexData     = hex2dec('04');
        
        %OpChunk - Stores connection and message records
        %   The records can be uncompressed or compressed.
        OpChunk         = hex2dec('05');
        
        %OpChunkInfo - Stores information about messages in a chunk
        OpChunkInfo     = hex2dec('06');
        
        %OpConnection - Stores the header of a ROS connection
        %   Information includes topic name and full text of the message 
        %   definition.
        OpConnection    = hex2dec('07');
        
        % Op String Values
        
        %OpStrMsgData - String corresponding to OpMsgData value
        OpStrMsgData    = 'MessageData';
        
        %OpStrFileHeader - String corresponding to OpFileHeader value
        OpStrFileHeader = 'FileHeader';
        
        %OpStrIndexData - String corresponding to OpIndexData value
        OpStrIndexData  = 'IndexData';
        
        %OpStrChunk - String corresponding to OpChunk value
        OpStrChunk      = 'Chunk';
        
        %OpStrChunkInfo - String corresponding to OpChunkInfo value
        OpStrChunkInfo  = 'ChunkInfo';
        
        %OpStrConnection - String corresponding to OpConnection value
        OpStrConnection = 'Connection';
        
        %OpStrConnectionHeader - String corresponding to OpConnectionHeader value
        OpStrConnectionHeader = 'ConnectionHeader';
        
        %FileHeaderLength - Bytes reserved for file header record (4KB)
        FileHeaderLength    = 4096;
        
        %ChunkInfoVersion - Version of the file chunk information
        ChunkInfoVersion        = 1;
        
        %ConnectionIndexVersion - Version of the connection index
        ConnectionIndexVersion  = 1;
        
        %CompressionNone - Indicates an uncompressed rosbag
        CompressionNone     = 'none';
        
        %CompressionBz2 - Indicates a compressed rosbag with bzip2
        %   Individual messages are compressed, not the rosbag as a whole.
        CompressionBz2      = 'bz2';                       
        
        %FieldType - Rosbag field names mapped to data type
        FieldType = ros.bag.internal.BagConstants.populateFieldTypes;         
        
        %OpMap - Rosbag Op code mapped to string designation
        OpMap = ros.bag.internal.BagConstants.populateOpCodes;
    end
    
    methods(Static, Access = private)
        function fieldType = populateFieldTypes
            %populateFieldTypes Create mapping from field names to types
            %   The types are MATLAB data types used to read the
            %   corresponding fields from the rosbag.
            
            %   Currently this mapping is implemented with the
            %   containers.Map class. Implementations with Java Hashtable 
            %   and HashMap were also tried, but performed more slowly.
            
            fieldType = containers.Map;
            
            % Setup known field lengths
            fieldType(ros.bag.internal.BagConstants.ChunkCountFieldName)        = 'int32';
            fieldType(ros.bag.internal.BagConstants.ConnectionCountFieldName)   = 'int32';
            fieldType(ros.bag.internal.BagConstants.IndexPosFieldName)          = 'int64';
            fieldType(ros.bag.internal.BagConstants.OpFieldName)                = 'uint8';
            fieldType(ros.bag.internal.BagConstants.SizeFieldName)              = 'int32';
            fieldType(ros.bag.internal.BagConstants.ConnectionFieldName)        = 'int32';
            fieldType(ros.bag.internal.BagConstants.TimeFieldName)              = 'int64';
            fieldType(ros.bag.internal.BagConstants.VerFieldName)               = 'int32';
            fieldType(ros.bag.internal.BagConstants.CountFieldName)             = 'int32';
            fieldType(ros.bag.internal.BagConstants.OffsetFieldName)            = 'int32';
            fieldType(ros.bag.internal.BagConstants.ChunkPosFieldName)          = 'int64';
            fieldType(ros.bag.internal.BagConstants.StartTimeFieldName)         = 'int64';
            fieldType(ros.bag.internal.BagConstants.EndTimeFieldName)           = 'int64';              
        end
        
        function opCodes = populateOpCodes
            %populateOpCodes Create mapping from op codes to strings
            %   The op code is represented by a single byte, but the map
            %   returned from this function establishes a correspondence
            %   between this byte and a corresponding string.
            
            % Types of records (mapped by op code)
            opCodes = containers.Map('KeyType', 'int32', 'ValueType', 'char');
            opCodes(ros.bag.internal.BagConstants.OpMsgData)      = ros.bag.internal.BagConstants.OpStrMsgData;
            opCodes(ros.bag.internal.BagConstants.OpFileHeader)   = ros.bag.internal.BagConstants.OpStrFileHeader;
            opCodes(ros.bag.internal.BagConstants.OpIndexData)    = ros.bag.internal.BagConstants.OpStrIndexData;
            opCodes(ros.bag.internal.BagConstants.OpChunk)        = ros.bag.internal.BagConstants.OpStrChunk;
            opCodes(ros.bag.internal.BagConstants.OpChunkInfo)    = ros.bag.internal.BagConstants.OpStrChunkInfo;
            opCodes(ros.bag.internal.BagConstants.OpConnection)   = ros.bag.internal.BagConstants.OpStrConnection;
        end
    end
    
end

