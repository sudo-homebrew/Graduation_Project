classdef(Hidden) OSPASimulinkBase < handle    
    % This is an internal class and may be removed or modified in a future
    % release.
    
    %OSPASimulinkBase Base class for the shared properties and functions related to OSPA and GOSPA Metric Simulink implementation.  
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties(Abstract)        
        % TrackExtractorFcn Property to store function handle or function
        % name to extract tracks from track bus.
        TrackExtractorFcn
        
        % TruthExtractorFcn Property to store function handle or function
        % name to extract truths from truth bus.
        TruthExtractorFcn
        
        %TrackFormat Property to store the supported track bus formats.
        TrackFormat
        
        %TruthFormat Property to store the supported truth bus formats.
        TruthFormat
    end
    
    properties (Access= protected)
        %A logical array to select from different components of output.
        pOutputSelector
        
        %Property to store the track extractor function.
        pTrackExtractorFcn;
        
        %Property to store the truth extractor function.
        pTruthExtractorFcn;
    end
               
    methods (Access = protected)
        %getInputFromBus Function to extract track and truth inputs from
        %the data bus.
        function [tracks,truths] = getInputFromBus(obj,varargin)
            tracks = obj.pTrackExtractorFcn(varargin{1});
            truths = obj.pTruthExtractorFcn(varargin{2});
        end
        
        %validateBusInputs Function to validate the bus input for tracks and
        %truths
        function validateBusInputs(~,varargin)            
            
            %Error if block input is not a struct.
            coder.internal.assert(isstruct(varargin{1}) , ...
                'fusion:simulink:OSPASimulinkBase:InvalidTrackInput');            
            coder.internal.assert(isstruct(varargin{2}) , ...
                'fusion:simulink:OSPASimulinkBase:InvalidTruthInput');            
        end
        
        %validateTrackAndTruthExtractorFcn Function to validate the track
        %and truth extractors.
        function validateTrackAndTruthExtractorFcn(obj)
            %TrackFormat
            if strcmpi(obj.TrackFormat,'custom')
                %Error if TrackFormat is selected as custom and
                %TrackExtractorFcn is not defined.
                coder.internal.assert(coder.internal.is_defined(obj.TrackExtractorFcn),'fusion:simulink:OSPASimulinkBase:EmptyTrackExtractor');
                if isa(obj.TrackExtractorFcn,'function_handle')
                    obj.pTrackExtractorFcn = obj.TrackExtractorFcn;
                else
                    obj.pTrackExtractorFcn = str2func(obj.TrackExtractorFcn);
                end
            else
                %Get appropriate track extractor function based on
                %TrackFormat.
                obj.pTrackExtractorFcn = fusion.simulink.internal.getTrackExtractor(obj.TrackFormat);
            end
            
            %TruthFormat
            if strcmpi(obj.TruthFormat,'custom')
                %Error if TruthFormat is selected as custom and
                %TruthExtractorFcn is not defined.
                coder.internal.assert(coder.internal.is_defined(obj.TruthExtractorFcn),'fusion:simulink:OSPASimulinkBase:EmptyTruthExtractor');
                if isa(obj.TruthExtractorFcn,'function_handle')
                    obj.pTruthExtractorFcn = obj.TruthExtractorFcn;
                else
                    obj.pTruthExtractorFcn = str2func(obj.TruthExtractorFcn);
                end
            else
                %Get appropriate truth extractor function based on
                %TruthFormat.
                obj.pTruthExtractorFcn = fusion.simulink.internal.getTruthExtractor(obj.TruthFormat);
            end
        end
        
        %getOSPAInputNames Function to get the block input port names.
        function inputNames = getOSPAInputNames(~,HasAssignmentInput)            
            trackInput = getString(message('fusion:simulink:OSPASimulinkBase:TrackInput'));
            truthInput = getString(message('fusion:simulink:OSPASimulinkBase:TruthInput'));            
            assignmentInput = getString(message('fusion:simulink:OSPASimulinkBase:AssignmentInput'));                                    
            if HasAssignmentInput
                inputNames = {trackInput, truthInput,assignmentInput};      
            else
                inputNames = {trackInput, truthInput};      
            end                               
        end
        
        %getOSPAInputNum Function to get the number of inputs in the block.
        function num = getOSPAInputNum(~,HasAssignmentInput)
            num = 2+HasAssignmentInput;
        end        
        
        %checkInactiveProperty Function to check if a property is inactive. 
        function flag = checkInactiveProperty(obj, prop)                       
            %TruthExtractorFcn inactive unless TruthFormat is selected as custom.
            flag = (strcmpi(prop,'TruthExtractorFcn') &&  ~strcmpi(obj.TruthFormat,'custom'));
            %TruthExtractorFcn inactive unless TrackFormat is selected as custom.
            flag = flag || (strcmpi(prop,'TrackExtractorFcn') &&  ~strcmpi(obj.TrackFormat,'custom'));
        end
        
        %checkInputSizeMutability Function to check if input size is mutable.        
        function flag = checkInputSizeMutability(~,index)
            %Track and Truth inputs are fixed size buses and Assignmnet
            %input is a variable size Nx2 matrix.
            if isequal(index,3)
                flag = true;
            else
                flag = false;
            end
        end
    end
    
    methods (Access = protected)
        function loadProtectedProperties(obj, s, wasLocked)
            if wasLocked
                obj.pOutputSelector     = s.pOutputSelector;
                obj.pTrackExtractorFcn  = s.pTrackExtractorFcn;
                obj.pTruthExtractorFcn  = s.pTruthExtractorFcn;
            end
        end
        
        function s = saveProtectedProperties(obj,sObj)            
            s = sObj;
            if isLocked(obj)
                s.pOutputSelector     = obj.pOutputSelector;
                s.pTrackExtractorFcn  = obj.pTrackExtractorFcn;
                s.pTruthExtractorFcn  = obj.pTruthExtractorFcn;                
            end
        end
        
        function  copyProtectedProperties(obj,c)            
            c.pOutputSelector     = obj.pOutputSelector;
            c.pTrackExtractorFcn  = obj.pTrackExtractorFcn;
            c.pTruthExtractorFcn  = obj.pTruthExtractorFcn;            
        end      
    end
    
    methods (Static, Access = protected)
        function section = getInputBusFormatSection
            properties = {'TrackFormat','TruthFormat', ...
                'TrackExtractorFcn','TruthExtractorFcn'};
            
            section = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','BusFormatSection',properties);
        end        
    end
    
    methods(Static,Hidden)
        function props = matlabCodegenNontunableProperties(~)
            props = {'TrackExtractorFcn' , 'TruthExtractorFcn' , 'TrackFormat' , 'TruthFormat', ...
                        'pOutputSelector' , 'pTrackExtractorFcn' , 'pTruthExtractorFcn'};
        end
    end
end

