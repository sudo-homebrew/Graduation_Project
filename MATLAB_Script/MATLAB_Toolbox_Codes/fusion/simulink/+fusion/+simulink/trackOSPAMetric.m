classdef (StrictDefaults, Hidden)trackOSPAMetric < trackOSPAMetric & fusion.simulink.OSPASimulinkBase
    
    % trackOSPAMetric OSPA metric computes the optimal subpattern assignment metric 
    % between a set of tracks and the known truths. An OSPA metric contains 
    % localization, cardinality, and labelling error components.
    %
    % This is the Simulink implementation of trackOSPAMetric.
    %
    % See also: trackOSPAMetric
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen

    % Public, non-tunable properties

    properties (Nontunable)

        % EnableLocalizationOSPAOutput Enables localization OSPA component in output
        %
        %   Default : false
        EnableLocalizationOSPAOutput (1, 1) logical = false;

        % EnableCardinalityOSPAOutput Enables cardinal OSPA component in output
        %
        %   Default : false
        EnableCardinalityOSPAOutput (1, 1) logical = false;

        % EnableLabelingOSPAOutput Enables labeling OSAP component in output
        %
        %   Default : false
        EnableLabelingOSPAOutput (1, 1) logical = false;
    end
    
    properties(Nontunable)
        % TrackExtractorFcn Property to store function handle or function
        % name to extract tracks from track bus.
        %
        %   Default : ''
        TrackExtractorFcn = '' ;
        
        % TruthExtractorFcn Property to store function handle or function
        % name to extract truths from truth bus.
        %
        %   Default : ''
        TruthExtractorFcn = '' ;
        
        %TrackFormat Property to store supported track formats.
        %
        %   Default : 'objectTrack'
        TrackFormat = 'objectTrack';
        
        %TruthFormat Property to store supported truth formats.
        %
        %   Default : 'Platform'
        TruthFormat = 'Platform';
    end        
    
    properties (Nontunable)
        % CustomWeights Custom window weights for Simulink block.
        %
        % Note: These are added because CustomWindowWeights (MATLAB
        % property) does not have a default. When a property doesn't have a
        % default value, System objects do not trigger calls to
        % validatePropertiesImpl when "OK" or "Apply" is clicked. In
        % Simulink, a default of 0 is used. This default will throw an
        % error like MATLAB if not specified correctly. 
        CustomWeights = 1;
    end

    properties (Hidden, Constant)
        TrackFormatSet = matlab.system.StringSet({'objectTrack','custom'});
        TruthFormatSet = matlab.system.StringSet({'Platform','Actor','custom'});
    end   

    methods
        function obj = trackOSPAMetric(varargin)         
            obj = obj@trackOSPAMetric(varargin{:});            
            setProperties(obj,nargin,varargin{:});
        end

        function set.CustomWeights(obj, val)
            obj.CustomWeights = val;
            obj.CustomWindowWeights = val;
        end
    end

    methods(Access = protected)
        function setupImpl(obj, varargin)                       
            %Extract input from bus
            [tracks,truths] = getInputFromBus(obj,varargin{:});

            setupImpl@trackOSPAMetric(obj,tracks,truths);
        end

        function varargout = stepImpl(obj, varargin)            

            %Extract input from bus
            [tracks,truths] = getInputFromBus(obj,varargin{:}); 

            numOutput = 4; %Total number of outputs.
            outputComponents = cell(1,numOutput);
            
            [outputComponents{:}] = stepImpl@trackOSPAMetric(obj,tracks,truths,varargin{3:end}); 
            
            %Select output components.            
            varargout={outputComponents{obj.pOutputSelector}};             
        end

        function validateInputsImpl(obj,varargin)
           %Validate input to the block and verify the fields.
            validateBusInputs(obj,varargin{:});
            [tracks,truths] = getInputFromBus(obj,varargin{:});
            validateTracks(obj, tracks);
            validateTruths(obj, truths);            
            if obj.HasAssignmentInput
                %GOSPA metric expects the TrackIds and TruthIds in assignment                 
                %inputs to be unique but simulink sends an Nx2 matrix with 
                %all zeros to initilize the link so the validation fails.
                %Following check is to avoid that.
                if all(varargin{3}(:) == 0)
                    assignments = zeros(0,2,'double');
                else
                    assignments=varargin{3};
                end                
                validateAssignments(obj,assignments);
            end            
        end               
        
        function validatePropertiesImpl(obj) 
            %validate Track and Truth extractors.
            validateTrackAndTruthExtractorFcn(obj);
            
            %The track and truth identifier functions are not exposed to
            %the user in Simulink. Appropriate identifiers are selected
            %internally as per selected track or truth format.
            
            %When TrackFormat is set as 'objectTrack' or 'custom', default
            %track identifier function is used. When using custom track bus format the
            %track identifier property is expected as 'TrackID', similar to 'objectTrack'
            %format. Users can convert the track identifier field within the custom
            %track extractor function if required.
            obj.pTrackIdentifierFcn = obj.pBuiltIns.getTrackIdentifier;
            
            %When TrackFormat is set as 'Actor', actor identifier function
            %is used and when it is set to 'Platform' or 'custom', default
            %identifier function is used. When using custom truth bus format
            %the truth identifier field is expected as 'PlatformID', similar
            %to 'Platform' format. Users can convert the truth identifier
            %field within the custom truth extratcor function if required.
            if strcmpi(obj.TruthFormat,'Actor')
                obj.pTruthIdentifierFcn = @fusion.simulink.internal.actorIdentifier;
            else
                %For Platform and Custom
                obj.pTruthIdentifierFcn = obj.pBuiltIns.getTruthIdentifier;
            end
            
            %validate distance function
            validateDistanceFcn(obj);
            
            %validate OSPA properties
            validateOSPAProperties(obj);
           
            %validate OSPA2 properties
            validateOSPA2Properties(obj);

            %Set a logical array to select from different output
            %componenets.
            setOutputSelector(obj);
        end

        function validateOSPA2Properties(obj)
            % With OSPA(2) metric, the HasAssignmentInput and Labeling
            % error output port must be false
            if strcmpi(obj.Metric,'OSPA(2)')
                coder.internal.assert(~obj.HasAssignmentInput,'fusion:simulink:trackOSPAMetric:OSPA2AssignmentInputDisabled');
                coder.internal.assert(~obj.EnableLabelingOSPAOutput,'fusion:simulink:trackOSPAMetric:OSPA2LabelingOutputDisabled');
            end
            validateOSPA2Properties@trackOSPAMetric(obj);
        end

        function flag = isInputSizeMutableImpl(obj,index)            
            flag = checkInputSizeMutability(obj,index);
        end  

        function icon = getIconImpl(~)            
            icon = getString(message('fusion:simulink:trackOSPAMetric:OSPAIcon'));
        end
        
        function varargout = getInputNamesImpl(obj)            
            varargout = getOSPAInputNames(obj,obj.HasAssignmentInput);
        end

        function num = getNumInputsImpl(obj)
            num = getOSPAInputNum(obj,obj.HasAssignmentInput);
        end
        
        function varargout = getOutputNamesImpl(obj)                        
            outputNames = { getString(message('fusion:simulink:trackOSPAMetric:OSPAOutput')), ...                      
                            getString(message('fusion:simulink:trackOSPAMetric:LocalizationOSPAOutput')), ...                
                            getString(message('fusion:simulink:trackOSPAMetric:CardinalityOSPAOutput')), ...
                            getString(message('fusion:simulink:trackOSPAMetric:LabelingOSPAOutput'))};                  
            varargout = outputNames(obj.pOutputSelector);            
        end

        function numOutput = getNumOutputsImpl(obj)            
            numOutput = 1 + obj.EnableLocalizationOSPAOutput + ...
                obj.EnableCardinalityOSPAOutput + obj.EnableLabelingOSPAOutput;                
        end

        function varargout = getOutputSizeImpl(~)
            [varargout{1:nargout}] = deal([1 1]); 
        end

        function varargout = getOutputDataTypeImpl(~)
            [varargout{1:nargout}] = deal('double');
        end

        function varargout = isOutputComplexImpl(~)            
            [varargout{1:nargout}] = deal(false);
        end

        function varargout = isOutputFixedSizeImpl(~)
            [varargout{1:nargout}] = deal(true);
        end  
        
        function flag = isInactivePropertyImpl(obj, prop)                                       
            flag = checkInactiveProperty(obj,prop);
            flag = flag || isInactivePropertyImpl@trackOSPAMetric(obj,prop);
        end

        function tf = isInactiveOSPAProperty(obj, prop)
            % In Simulink, HasAssignmentInput cannot be disabled as it
            % controls an output port. An error will be thrown in validate
            % properties if its enabled.
            tf = strcmpi(obj.Metric,'OSPA(2)') && any(strcmpi(prop,{'LabelingError'}));
        end
        
        function tf = isInactiveOSPA2Property(obj, prop)
            tf = isInactiveOSPA2Property@trackOSPAMetric(obj, prop);
            tf = tf || (strcmpi(obj.Metric,'OSPA') || strcmpi(obj.WindowWeights,'auto')) && strcmpi(prop,'CustomWeights');
        end
    end
    
    methods (Access = protected)
        function loadObjectImpl(obj, s, wasLocked)
            % Set private and protected properties
            loadObjectImpl@trackOSPAMetric(obj,s,wasLocked)
            loadProtectedProperties(obj,s,wasLocked);
        end
        
        function s = saveObjectImpl(obj)
            sObj = saveObjectImpl@trackOSPAMetric(obj);
            s = saveProtectedProperties(obj,sObj);
        end
        function c = cloneImpl(obj)
            c = cloneImpl@trackOSPAMetric(obj);
            copyProtectedProperties(obj,c);
        end      
    end
    
    methods (Access=private)        
        function setOutputSelector(obj) 
            %Set a logical array to select from different output
            %componenets. First component is always available and rest of 
            %the componenets are selected as per user input.
            obj.pOutputSelector = [true,obj.EnableLocalizationOSPAOutput,...
                obj.EnableCardinalityOSPAOutput,obj.EnableLabelingOSPAOutput];                                  
        end            
    end
    
    % Block icon and dialog customizations
    methods (Static, Access = protected)
        function header = getHeaderImpl
            %BLock header and title text.
            header = matlab.system.display.Header(...
                'trackOSPAMetric', ...
                'Title', getString(message('fusion:block:ospaMetricTitle')), ...
                'Text',  getString(message('fusion:block:ospaMetricDesc')));
        end
      
        function groups = getPropertyGroupsImpl
            %Block property sections and groups.
            OSPAProperties = {'Metric','CutoffDistance','Order'};
            OSPAPropertySection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','OSPAPropertySection',OSPAProperties,...
                {1,'simulink:trackOSPAMetric'});

            DistanceProperties = {'Distance','MotionModel','DistanceFcn'};
            DistancePropertySection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','DistancePropertySection',DistanceProperties);

            LabelingProperties = {'LabelingError'};
            LabelingPropertySection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','LabelingPropertySection',LabelingProperties);            

            OSPA2Properties = {'WindowLength','WindowSumOrder','WindowWeights','WindowWeightExponent','CustomWeights'};
            OSPA2Section = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackOSPAMetric','OSPA2Section',OSPA2Properties);

            OutputProperties = {'EnableLocalizationOSPAOutput','EnableCardinalityOSPAOutput','EnableLabelingOSPAOutput'};
            OutputSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackOSPAMetric','OutputSection',OutputProperties);

            InputProperties = {'HasAssignmentInput'};
            InputSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','InputSection',InputProperties);
            
            InputBusFormatSection = fusion.simulink.OSPASimulinkBase.getInputBusFormatSection();

            PropertyGroup = matlab.system.display.SectionGroup(...
           'Title',getString(message('fusion:simulink:OSPASimulinkBase:TitlePropertyGroup')), ...
           'Sections',[OSPAPropertySection,DistancePropertySection,LabelingPropertySection,OSPA2Section]);

            PortSettingGroup = matlab.system.display.SectionGroup(...
           'Title',getString(message('fusion:simulink:OSPASimulinkBase:TitlePortSettingGroup')), ...
           'Sections',[InputSection,OutputSection,InputBusFormatSection]);

            groups = [PropertyGroup,PortSettingGroup];
        end
   end
    % Is allowed in system block method.
    methods (Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = true;    
        end
    end
end 
