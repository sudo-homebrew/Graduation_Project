classdef ReadLogFileBase < matlab.System & ros.internal.mixin.ROSInternalAccess
    %This class is for internal use only. It may be removed in the future.

    %Play back data from a supported logfile
    %
    %   This is abstract system object and it is intended for use with the MATLAB System
    %   block.
    %   In order to access the rosbag playback functionality from
    %   MATLAB, see ROSBAG.
    %   In order to access the rosbag2 playback functionality from
    %   MATLAB, see ROS2BAG.

    %   Copyright 2021 The MathWorks, Inc.

    % Public, non-tunable properties
    properties (Nontunable)
        %Topic Topic name
        %   Subset of data within the logfile to play back
        Topic = '/my_topic';

        %SampleTime Sample time
        SampleTime = -1;
    end
    
    properties (Nontunable,Abstract)
        %LogfileType Logfile type
        %   Type of logfile will be displayed on front of the block
        LogfileType   
    end
    
    properties(Abstract,Access=protected)
        %BusUtilObj Bus utility object used with Simulink bus conversion
        BusUtilObj
    end
    
    properties(Constant,Abstract,Access=protected)
        %TimeFactor Multiplication-factor for converting Simulink time to
        %match Logfile time stamps
        TimeFactor
    end
    
    properties (Abstract, Access = protected, Transient)
        %DataObject Object containing or allowing for access to all logfile data
        DataObject

        %DataSelection Object containing or allowing access to applicable logfile data
        % (contained in topic, after offset, and before duration completes
        DataSelection

        %Converter Converts from logfile messages to struct for output to bus
        Converter
    end
    
    % The following should ideally not show up in the MATLAB System block
    % dialog. However, setting them as 'Hidden' will prevent them from
    % being accessible via set_param and get_param.
    properties (Nontunable)
        %ModelName Model name
        %   Model name is used to access workspace and define busses
        ModelName = '';

        %MsgType Message type
        %   Type of each data point within selected topic
        MsgType = '';
    end

    properties (Access = protected, Transient)
        %PlaybackOffset When to start the playback from, compared to logfile start time
        % NaN indicates unset value, default to play back whole file
        PlaybackOffset = NaN;

        %PlaybackDuration How long to continue playback for in the logfile
        % NaN indicates unset value, default to play back whole file
        PlaybackDuration = NaN;

        %MsgIdx Index of next message that has not yet been output
        MsgIdx = 1;

        %LastConvertedMsg Most recent message converted and output
        % Blank message of correct type until first message is output
        LastConvertedMsg = [];

        %SampleTimeHandler Object for validating sample time settings
        SampleTimeHandler

        %IsControllableSampleTime Indicates if controllable sample time is used
        IsControllableSampleTime
    end

    methods (Static,Hidden,Abstract)
        ret = isValidLogdataObject(logObj)
        ret = getOutputDatatypeString(msgType,modelName)
        clearBusesOnModelClose(block)
        ret = getBlockIcon()
        ret = getEmptyDataObject()
        ret = getBusConverterObject(msgType,modelName)
        ret = getLogFileExtension()
        ret = getEmptyMessage(msgType)
    end
    
    methods (Access=protected,Abstract)
        ret = convertMessage(obj,msg)
    end
    
    methods
        %% Constructor
        function obj = ReadLogFileBase(varargin)
        %ReadData Construct a system object for logfile playback

        % Set properties used for sample time
            obj.SampleTimeHandler = robotics.slcore.internal.block.SampleTimeImpl;
            obj.IsControllableSampleTime = false;

            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end

        %% Setters, and Getters for dependent properties
        function set.Topic(obj, val)
            validateattributes(val, ...
                               {'char'}, {'scalartext'}, '', 'Topic');
            obj.Topic = val;
        end

        function set.ModelName(obj, val)
            validateattributes(val, ...
                               {'char'}, {'scalartext'}, '', 'ModelName')
            obj.ModelName = val;
        end

        function set.MsgType(obj, val)
            validateattributes(val, ...
                               {'char'}, {'scalartext'}, '', 'MsgType')
            obj.MsgType = val;
        end

        function set.SampleTime(obj, sampleTime)
        %set.SampleTime Validate sample time specified by user

            validateattributes(sampleTime, {'numeric'}, ...
                               {'nonempty', 'real', 'nonnan'}, '', 'SampleTime');

            % Structural check: Error out if more than two elements are specified
            coder.internal.errorIf(numel(sampleTime) > 2, ...
                                   'shared_robotics:robotslcore:sampletime:InvalidSampleTimeNeedScalar');

            % Make sure the sample time input is always a row vector
            sampleTimeRow = double(reshape(sampleTime, 1, []));

            % Special handling for sample times allowed by this block, but
            % not handled by sample time handler
            if sampleTimeRow(1) == -2
                % Variable - signal to rate get to set sample time
                obj.SampleTime = -2;
            elseif numel(sampleTimeRow) == 2 && sampleTimeRow(1) > 0 && ...
                    ~isinf(sampleTimeRow(1)) && ...
                    ~isempty(regexp(sprintf('%d', sampleTimeRow(2)), '^-2\d*$', 'once'))
                % Controllable - signal to rate get to set sample time type
                % and to set gaps between execution dynamically
                obj.SampleTime = sampleTimeRow(1);
                obj.IsControllableSampleTime = true; %#ok<MCSUP>
            else
                if sampleTimeRow(1) == 0
                    % Continuous - effectively the same as fixed in minor
                    % step for this block
                    sampleTimeRow = [0 1];
                end
                obj.SampleTime = validate(obj.SampleTimeHandler, sampleTimeRow); %#ok<MCSUP>
            end
        end

    end

    methods(Access = protected)
        %% Helper functions
        function refreshDataObject(obj)
        %refreshDataObject Extract logfile data object from workspace
        %   Attempts to retrieve the object containing or allowing
        %   access to the logfile data from the specified workspace. If
        %   the object in the workspace is missing or invalid, an empty
        %   data object will be stored.

        % Default value
            dataobj = obj.getEmptyDataObject();
            dataobjVal = ros.slros.internal.block.ReadDataBlockMask.getDataFromWS(...
                ros.slros.internal.block.ReadDataBlockMask.DataObjectVarName, ...
                dataobj, obj.ModelName);

            if obj.isValidLogdataObject(dataobjVal)
                dataobj = dataobjVal;
            end

            obj.DataObject = dataobj;
        end

        function refreshPlaybackWindow(obj)
        %refreshPlaybackWindow Extract time-window for data playback from workspace
        %   Attempts to retrieve the values for the offset of the data
        %   playback start time and the duration of playback from the
        %   specified workspace. If one or both of the values in the
        %   workspace are missing or invalid, they will default to
        %   playing back the entire data file.

        % Makes use of data object, so make it available if possible
            refreshDataObject(obj)

            % NaN or missing values indicate to use default
            offset = NaN;
            duration = NaN;

            offsetVal = ros.slros.internal.block.ReadDataBlockMask.getDataFromWS(...
                ros.slros.internal.block.ReadDataBlockMask.PlaybackOffsetVarName, ...
                offset, obj.ModelName);
            durationVal = ros.slros.internal.block.ReadDataBlockMask.getDataFromWS(...
                ros.slros.internal.block.ReadDataBlockMask.PlaybackDurationVarName, ...
                duration, obj.ModelName);

            % Take valid values from model workspace variables
            if isnumeric(offsetVal) && isscalar(offsetVal) && ...
                    ~isinf(offsetVal) && ~isnan(offsetVal)
                offset = offsetVal;
            end
            if isnumeric(durationVal) && isscalar(durationVal) && ...
                    ~isinf(durationVal) && ~isnan(durationVal)
                duration = durationVal;
            end

            % Set to defaults if NaN or invalid values
            if ~isempty(obj.DataObject)
                if isnan(offset)
                    offset = 0;
                end
                if isnan(duration)
                    duration = obj.DataObject.EndTime-obj.DataObject.StartTime;
                end
            end

            obj.PlaybackOffset = offset;
            obj.PlaybackDuration = duration;
        end

        function refreshDataSelection(obj)
        %refreshDataSelection Select data for playback from this block
        %   Extract the object containing or allowing access to the
        %   data relevant to the specified topic, and contained within
        %   the playback window.

        % Requires both the data object to be loaded and the playback
        % window to be specified, so ensure that is the case
            refreshPlaybackWindow(obj)

            % Error if there are issues with the data or topic
            if isempty(obj.DataObject)
                error(message('ros:slros:readlog:InvalidLogData'))
            end
            if isempty(obj.Topic)
                error(message('ros:slros:readlog:NoTopicName', mfilename('class')))
            end
            if ~contains(obj.Topic, obj.DataObject.AvailableTopics.Row)
                error(message('ros:slros:readlog:InvalidTopicName', obj.Topic, mfilename('class')))
            end

            % Determine playback window based on full logfile
            if isnan(obj.PlaybackOffset)
                startTime = obj.DataObject.StartTime;
            else
                startTime = obj.DataObject.StartTime+obj.PlaybackOffset;
            end
            if isnan(obj.PlaybackDuration)
                endTime = obj.DataObject.EndTime;
            else
                endTime = startTime+obj.PlaybackDuration;
            end

            % Downselect the data
            obj.DataSelection = select(obj.DataObject, ...
                                       'Topic', obj.Topic, ...
                                       'Time', [startTime endTime]);
        end


        %% Simulation functions
        function resetImpl(obj)
        %resetImpl Perform setup for running model with block
        %   Reset states that must be reverted to initial values before
        %   a re-run of the model. Prepare for first time-step.
        %   Will error if not in a Simulink model.

        % Ensure that the most up-to-date data is selected and checked
            refreshDataSelection(obj)

            % Set up to output messages in bus format
            obj.MsgIdx = 1;
            obj.Converter = obj.getBusConverterObject(obj.MsgType, obj.ModelName);
            obj.LastConvertedMsg = obj.Converter.convert(obj.getEmptyMessage(obj.MsgType));

            % Determine number of time steps before first message
            % Only valid for controllable sample time
            if obj.IsControllableSampleTime
                if isempty(obj.DataSelection)
                    stepsToNextMsg = inf;
                else
                    nextMsgTimestamp = obj.DataSelection.MessageList{obj.MsgIdx, 'Time'};
                    timeToNextMsg = nextMsgTimestamp-(obj.DataObject.StartTime+obj.PlaybackOffset);
                    stepsToNextMsg = ceil(timeToNextMsg/obj.SampleTime);
                end
                stepsToNextMsg = clipNumTicks(stepsToNextMsg);
                setNumTicksUntilNextHit(obj, stepsToNextMsg);
            end
        end

        function [isNew, msg] = stepImpl(obj)
        %stepImpl Determine output for single time step
        %   MSG will always output the data from the "most recent"
        %   message stored in the logfile. The "most recent" message is
        %   determined by the current simulation time translated into
        %   logfile time.
        %   ISNEW will indicate if the MSG has not been output at a
        %   previous time step.
        %   Will error if not in a Simulink model.

            simTime = get_param(obj.ModelName, 'SimulationTime');

            % No more messages to output
            % Requires that DataSelection has downselected to only include
            % those messages within the specified duration
            if obj.MsgIdx > obj.DataSelection.NumMessages
                % Set output
                isNew = false;
                msg = obj.LastConvertedMsg;

                % Set maximum time steps before block is called again
                % Only valid for controllable sample time
                if obj.IsControllableSampleTime
                    setNumTicksUntilNextHit(obj, clipNumTicks(inf));
                end

                % Still messages left to output
            else
                currentLogTime = simTime*obj.TimeFactor + obj.PlaybackOffset + obj.DataObject.StartTime;
                nextMsgTimestamp = obj.DataSelection.MessageList{obj.MsgIdx, 'Time'};

                % Still not at time to output new message
                if nextMsgTimestamp > currentLogTime
                    % Set output
                    isNew = false;
                    msg = obj.LastConvertedMsg;

                    % Set time steps to next message
                    % Only valid for controllable sample time
                    % obj.SampleTime will be base rate for CST
                    if obj.IsControllableSampleTime
                        timeToNextMsg = nextMsgTimestamp-currentLogTime;
                        stepsToNextMsg = ...
                            ceil(double(timeToNextMsg)/(obj.TimeFactor*obj.SampleTime));
                        stepsToNextMsg = clipNumTicks(stepsToNextMsg);
                        setNumTicksUntilNextHit(obj, stepsToNextMsg);
                    end

                    % Time to output new message
                else
                    % Determine which message to read
                    % Must be at least the current index
                    while obj.MsgIdx < obj.DataSelection.NumMessages && ...
                            obj.DataSelection.MessageList{obj.MsgIdx+1, 'Time'} < currentLogTime
                        obj.MsgIdx = obj.MsgIdx+1;
                    end

                    % Prepare message for output
                    msgRaw = readMessages(obj.DataSelection, obj.MsgIdx);
                    msgRaw = msgRaw{1};
                    msg = obj.convertMessage(msgRaw);
                    isNew = true;
                    obj.LastConvertedMsg = msg;

                    % Prepare for next message
                    obj.MsgIdx = obj.MsgIdx+1;

                    % Next time step required at least to change IsNew
                    % Only valid for controllable sample time
                    if obj.IsControllableSampleTime
                        setNumTicksUntilNextHit(obj, 1);
                    end
                end

                % Special case for constant sample time
                % Maintain message output for rest of simulation
                if isinf(obj.SampleTime(1))
                    obj.MsgIdx = obj.DataSelection.NumMessages+1;
                end
            end
        end

        function varargout = isOutputFixedSizeImpl(~, ~)
        %isOutputFixedSizeImpl Indicates if specified output can change size
            varargout = {true, true};
        end

        function varargout = getOutputSizeImpl(~)
        %getOutputSizeImpl Indicates size of all output signals
            varargout = {[1,1], [1,1]};
        end

        function varargout = getOutputDataTypeImpl(obj)
        %getOutputDataTypeImpl Indicates datatype of all output signals

        % Create the bus definition if required
            outputDataTypeStr = '';
            if ~isempty(obj.ModelName)
                if ~isempty(obj.MsgType)
                    outputDataTypeStr = obj.getOutputDatatypeString(obj.MsgType,obj.ModelName);
                else
                    % Occurs if topic has not been selected
                    error(message('ros:slros:readlog:InvalidTopicName', obj.Topic, mfilename('class')))
                end
            end

            varargout = {'logical', outputDataTypeStr};
        end

        function varargout = isOutputComplexImpl(~)
        %isOutputComplexImpl Indicates complex state of all output signals
            varargout = {false,false};
        end

        function N = getNumInputsImpl(~)
        %getNumInputsImpl Indicates number of input ports to block
            N = 0;
        end

        function N = getNumOutputsImpl(~)
        %getNumOutputsImpl Indicates number of output ports from block
            N = 2;
        end

        function st = getSampleTimeImpl(obj)
        %getSampleTimeImpl Returns specification of sample time to use for block
        %   Sample time specification indicates both the type of sample
        %   time that will be used, based on the SampleTime parameter of
        %   the block, but also the value of the sample time, if
        %   applicable.

            if isequal(obj.SampleTime, -2)          % Variable
                                                    % getSampleTimeImpl is called before resetImpl, so need to
                                                    % refresh the data selection for this sample time case
                refreshDataSelection(obj)
                tGap = ros.slros.internal.block.ReadDataBlockMask.getMinNonzeroMsgGap(obj.DataSelection);
                if isempty(tGap)                    % Only one message time
                                                    % Allow solver to decide sample time
                    st = createSampleTime(obj, 'Type', 'Inherited');
                else
                    % Choose sample time to get all possible messages
                    st = createSampleTime(obj, ...
                                          'Type', 'Discrete', 'SampleTime', tGap);
                end
            elseif obj.IsControllableSampleTime     % Controllable
                st = createSampleTime(obj, 'Type', 'Controllable', ...
                                      'TickTime', obj.SampleTime);
            else
                st = obj.SampleTimeHandler.createSampleTimeSpec();
            end
        end

        function flag = isInactivePropertyImpl(~, ~)
        %isInactivePropertyImpl Indicates which properties should not be displayed in mask or Command Window

        % Allow all properties that are hidden in the mask to be
        % considered an active property to avoid warnings
            flag = false;
        end
    end

    methods(Static, Access = protected)
        %% Simulation customization functions
        function simMode = getSimulateUsingImpl(~)
        %getSimulateUsingImpl Indicate simulation mode
        % This block does not support codegen
            simMode = 'Interpreted execution';
        end

        function isVisible = showSimulateUsingImpl
        %showSimulateUsingImpl Indicate if "Simulate using" parameter should display on block mask
            isVisible = false;
        end
    end
end


%% Helper functions
function nTicks = clipNumTicks(nTicks)
%clipNumTicks Ensure that the number of ticks does not exceed the max
%   Maximum specified by limits on discrete variable sample time.
    overflowVal = double(intmax('int32')-1);
    underflowVal = 1;
    nTicks = min(nTicks,overflowVal);
    nTicks = max(nTicks,underflowVal);
end
