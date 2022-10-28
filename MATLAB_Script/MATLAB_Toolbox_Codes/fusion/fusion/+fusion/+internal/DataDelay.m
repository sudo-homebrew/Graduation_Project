classdef DataDelay < matlab.System
    % This class is internal and may be modified or removed in a future
    % releaes

    %DataDelay  A genearl data delay inducing object
    %
    % DataDelay properties:
    %   DelayAll          - If true, all data gets delayed
    %   SensorIndices     - A list of sensor indices to apply the time delay
    %   Capacity          - Maximum number of detections the object stores
    %   DelaySource       - Choose the source of delay
    %   DelayDistribution - Choose the type of delay to apply
    %   DelayParameters   - Parameters controlling the delay

    % Copyright 2021 The MathWorks, Inc.
    %#codegen

    properties(Nontunable)
        %DelayAll  If true, all data gets delayed
        DelayAll
    end

    properties(Nontunable)
        %SensorIndices  A list of sensor index values to delay
        SensorIndices

        %Capacity Maximum number of data objects to buffer
        Capacity
    end
    
    properties(Nontunable)
        %DelaySource  Choose the source of delay
        DelaySource

        %DelayDistribution Choose the type of delay to apply
        DelayDistribution
    end

    properties
        %DelayParameters
        DelayParameters
    end

    properties (Access = {?DataDelay, ?matlab.unittest.TestCase})
        %pCurrentTime Keep the current timestamp
        pCurrentTime 
        
        %pDataTimeStamps Timestamps of all data items
        pDataTimeStamps

        %pSensorIndices Sensor indices of all data items
        pSensorIndices

        %pDataLag The amount of delay each data item has 
        pDataLag

        %pData The stored buffer of data items
        pData

        %pSampleData One sample of the data
        pSampleData

        %pNumCurrentDataItems The current number of data elements in the buffer
        pNumCurrentDataItems
    end
    
    properties(Nontunable, Access = protected)
        %pStaticMemory A flag. True if the capacity is finite
        pStaticMemory (1,1) logical

        %pInputInCell A flag. True if the input data is in cell array
        pInputInCell (1,1) logical

        %pInputDelay A flag. True if DelaySource is 'Input'
        pInputDelay (1,1) logical
    end

    methods
        function obj = DataDelay(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function varargout = stepImpl(obj, newData, time, varargin)
            validateattributes(time,{'double','single'},{'>=',obj.pCurrentTime});
            numNewDataElements = unpackNewData(obj, newData);
            applyDelay(obj, numNewDataElements, varargin{:});
            delayedData = packDelayedData(obj, time);
            argout = cell(1,nargout);
            argout{1} = delayedData;
            if nargout > 1
                argout{2} = obj.pNumCurrentDataItems;
            end
            if nargout > 2
                argout{3} = prepareInfo(obj);
            end
            varargout = argout;
            obj.pCurrentTime = time;
        end

        function setupImpl(obj, newData, time, varargin)
            % Perform one-time calculations, such as computing constants
            if iscell(newData)
                d = {newData{1}};
                obj.pInputInCell = true;
            else
                d = {newData(1)};
                obj.pInputInCell = false;
            end

            obj.pStaticMemory = isfinite(obj.Capacity);
            obj.pInputDelay = strcmpi(obj.DelaySource,'Input');
            
            if obj.pStaticMemory
                obj.pData = repmat(d,obj.Capacity,1);
                obj.pDataTimeStamps = zeros(obj.Capacity,1,'like',time);
                obj.pDataLag = zeros(obj.Capacity,1,'like',time);
                obj.pSensorIndices = zeros(obj.Capacity,1,'uint32');
            else
                coder.varsize('d',[Inf,1],[1,0]);
                obj.pData = d;
                t = zeros(0,1,'like',time);
                coder.varsize('t',[Inf,1],[1,0]);
                obj.pDataTimeStamps = t;
                obj.pDataLag = t;
                i = zeros(0,1,'uint32');
                coder.varsize('i',[Inf,1],[1,0]);
                obj.pSensorIndices = i;
            end
            obj.pSampleData = d;
            obj.pCurrentTime = zeros(1,1,'like',time);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.pCurrentTime = zeros(1,1,'like',obj.pDataTimeStamps);
            if obj.pStaticMemory
                obj.pData = repmat(obj.pSampleData,obj.Capacity,1);
                obj.pDataTimeStamps(:) = 0;
                obj.pDataLag(:) = 0;
                obj.pSensorIndices(:) = 0;
            else
                obj.pData = repmat(obj.pSampleData, 0, 1);
                obj.pDataTimeStamps = zeros(0,1,'like',obj.pDataTimeStamps);
                obj.pDataLag = zeros(0,1,'like',obj.pDataTimeStamps);
                obj.pSensorIndices = zeros(0,1,'uint32');
            end
            obj.pNumCurrentDataItems = uint32(0);
        end

        function flag = isInputDataTypeMutableImpl(~,~)
            % Return false if input data type cannot change
            % between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            num = 2;
            if strcmpi(obj.DelaySource,'Input')
                num = num + 1;
            end
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            if wasLocked
                obj.pCurrentTime = s.pCurrentTime;
                obj.pDataTimeStamps = s.pDataTimeStamps;
                obj.pSensorIndices = s.pSensorIndices;
                obj.pDataLag = s.pDataLag;
                obj.pData = s.pData;
                obj.pStaticMemory = s.pStaticMemory;
                obj.pNumCurrentDataItems = s.pNumCurrentDataItems;
                obj.pSampleData = s.pSampleData;
                obj.pInputInCell = s.pInputInCell;
                obj.pInputDelay = s.pInputDelay;
            end

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            if isLocked(obj)
                s.pCurrentTime = obj.pCurrentTime;
                s.pDataTimeStamps = obj.pDataTimeStamps;
                s.pSensorIndices = obj.pSensorIndices;
                s.pDataLag = obj.pDataLag;
                s.pData = obj.pData;
                s.pStaticMemory = obj.pStaticMemory;
                s.pNumCurrentDataItems = obj.pNumCurrentDataItems;
                s.pSampleData = obj.pSampleData;
                s.pInputInCell = obj.pInputInCell;
                s.pInputDelay = obj.pInputDelay;
            end
        end
        
        function numNewDataElements = unpackNewData(obj, newData)
            % This method takes the new data and stores it with the
            % existing data.
            
            currentNumDataElements = obj.pNumCurrentDataItems;
            numNewDataElements = numel(newData);
            numData = currentNumDataElements + numNewDataElements;

            if obj.pStaticMemory
                coder.internal.assert(numData <= obj.Capacity, ...
                    'fusion:objectDetectionDelay:capacityExceeded',obj.Capacity);
                for i = 1:numNewDataElements
                    if obj.pInputInCell
                        obj.pData{currentNumDataElements + i} = newData{i};
                    else
                        obj.pData{currentNumDataElements + i} = newData(i);
                    end
                end
            else
                obj.pDataTimeStamps = vertcat(obj.pDataTimeStamps(:),zeros(numNewDataElements,1,'like',obj.pDataTimeStamps));
                obj.pSensorIndices = vertcat(obj.pSensorIndices(:),zeros(numNewDataElements,1,'like',obj.pSensorIndices));
                if coder.target('MATLAB')
                    if obj.pInputInCell
                        obj.pData = vertcat(obj.pData(:), newData(:));
                    else
                        obj.pData = vertcat(obj.pData(:), num2cell(newData(:)));
                    end
                else
                    data = repmat(obj.pSampleData, numData, 1);
                    for i = 1:currentNumDataElements
                        data{i} = obj.pData{i};
                    end
                    for i = 1:numNewDataElements
                        if obj.pInputInCell
                            data{currentNumDataElements + i} = newData{i};
                        else
                            data{currentNumDataElements + i} = newData(i);
                        end
                    end
                    obj.pData = data;
                end
            end
            
            for i = 1:numNewDataElements
                obj.pDataTimeStamps(currentNumDataElements + i) = getTimeStamp(obj,newData,i);
                obj.pSensorIndices(currentNumDataElements + i) = getSensorIndex(obj,newData,i);
            end
            obj.pNumCurrentDataItems(1) = numData;
        end
        
        function applyDelay(obj, numNewDataElements, varargin)
            % Add delay information for each new data element
            
            % Check if any data element is from a valid sensor
            toDelay = delayableDataBySensor(obj,numNewDataElements);

            newDataLag = zeros(numNewDataElements,1,'like',obj.pDataLag);
            if any(toDelay,'all')
                if obj.pInputDelay
                    newDataLag(:) = applyDelayFromInput(obj,newDataLag,varargin{1});
                else
                    switch obj.DelayDistribution
                        case 'Constant'
                            newDataLag(:) = repmat(obj.DelayParameters(1),numNewDataElements,1);
                        case 'Uniform'
                            newDataLag(:) = obj.DelayParameters(1) + ...
                                (obj.DelayParameters(2) - obj.DelayParameters(1)) .* rand(numNewDataElements,1);
                        case 'Normal'
                            newDataLag(:) = obj.DelayParameters(1) + obj.DelayParameters(2) .* randn(numNewDataElements,1);
                    end
                end
                newDataLag = toDelay .* newDataLag;
            end

            % Store data delay in memory
            if obj.pStaticMemory
                obj.pDataLag(obj.pNumCurrentDataItems-numNewDataElements+1:obj.pNumCurrentDataItems) = newDataLag;
            else
                obj.pDataLag = vertcat(obj.pDataLag,newDataLag(:));
            end
        end

        function newDataLag = applyDelayFromInput(~,newDataLag,timeDelay)
            if isscalar(timeDelay)
                newDataLag(:) = timeDelay;
            else
                newDataLag = timeDelay;
            end
        end
        
        function tf = delayableDataBySensor(obj,numNewDataElements)
            % Checks which data is delayable
            if obj.DelayAll
                tf = true(numNewDataElements,1);
            else
                tf = ismember(obj.pSensorIndices(obj.pNumCurrentDataItems-numNewDataElements+1:obj.pNumCurrentDataItems,1),...
                    obj.SensorIndices);
            end
        end
        
        function delayedData = packDelayedData(obj, time)
            % This method releases the delayed data and removes them from
            % the kept buffered data
            
            toOutput = (time - obj.pDataTimeStamps) >= obj.pDataLag;
            toOutput = toOutput(1:obj.pNumCurrentDataItems);
            numKept = sum(~toOutput);
            if coder.target('MATLAB')
                % The output depends on the type of input
                if obj.pInputInCell % Return a cell arrau
                    delayedData = obj.pData(toOutput,1);
                else % Return a regular array
                    if any(toOutput)
                        delayedData = [obj.pData{toOutput,1}]';
                    else
                        delayedData = repmat(obj.pSampleData{1},0,1);
                    end
                end

                % Keep the remaining data items
                if obj.pStaticMemory 
                    % Push remaining data items to the beginning of list.
                    % No care for the rest because obj.pNumCurrentDataItems
                    % keeps track of how many are really used
                    obj.pData(1:numKept) = obj.pData(~toOutput);
                else
                    % Reduce list to only kept
                    obj.pData = obj.pData(~toOutput);
                end
            else
                % Codegen needs a special care for cell arrays
                delayedData = packDelayedDataCG(obj,toOutput);
            end

            % Update regular arrays that coder knows how to handle
            if obj.pStaticMemory
                obj.pDataTimeStamps(1:numKept) = obj.pDataTimeStamps(~toOutput);
                obj.pDataLag(1:numKept) = obj.pDataLag(~toOutput);
                obj.pSensorIndices(1:numKept) = obj.pSensorIndices(~toOutput);
            else
                obj.pDataTimeStamps = obj.pDataTimeStamps(~toOutput);
                obj.pDataLag = obj.pDataLag(~toOutput);
                obj.pSensorIndices = obj.pSensorIndices(~toOutput);
            end

            % Keep track of current number of data items in the buffer
            obj.pNumCurrentDataItems(1) = numKept;
        end

        function delayedData = packDelayedDataCG(obj,toOutput)
            % Codegen needs a special care for cell arrays
            numToOutput = matlabshared.tracking.internal.fusion.codegen.StrictSingleCoderUtilities.IntLogicalSum(toOutput);
            assert(numToOutput <= obj.Capacity);
            if obj.pInputInCell % Deliver output in cell like input
                delayedData = repmat(obj.pSampleData,numToOutput,1);
            else % Deliver output in a regular array like the input
                delayedData = repmat(obj.pSampleData{1},numToOutput,1);
            end
            idx = 0;
            for i = 1:obj.pNumCurrentDataItems
                if toOutput(i)
                    idx = idx + 1;
                    if obj.pInputInCell % Deliver output in cell like input
                        delayedData{idx} = obj.pData{i};
                    else % Deliver output in a regular array like the input
                        delayedData(idx) = obj.pData{i};
                    end
                else
                    obj.pData{i-idx} = obj.pData{i};
                end
            end
        end

        function info = prepareInfo(obj)
            numData = obj.pNumCurrentDataItems;
            if numData > 0
                info = struct(...
                    'DetectionTime',obj.pDataTimeStamps(1:numData)', ...
                    'Delay', obj.pDataLag(1:numData)', ...
                    'DeliveryTime', (obj.pDataTimeStamps(1:numData) + obj.pDataLag(1:numData))');
            else
                info = struct(...
                    'DetectionTime',zeros(1,0,'like',obj.pDataTimeStamps), ...
                    'Delay', zeros(1,0,'like',obj.pDataTimeStamps), ...
                    'DeliveryTime', zeros(1,0,'like',obj.pDataTimeStamps));
            end
        end
        
        function timeStamp = getTimeStamp(obj, newData, i)
            if obj.pInputInCell
                timeStamp = newData{i}.Time;
            else
                timeStamp = newData(i).Time;
            end
        end

        function index = getSensorIndex(obj,newData,i)
            if obj.pInputInCell
                index = newData{i}.SensorIndex;
            else
                index = newData(i).SensorIndex;
            end
        end
    end
end