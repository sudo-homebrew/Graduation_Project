classdef (StrictDefaults, Hidden)trackGOSPAMetric < trackGOSPAMetric & fusion.simulink.OSPASimulinkBase
    
    % trackGOSPAMetric GOSPA metric computes the generalized optimal subpattern assignment metric
    % between a set of tracks and the known truths. A GOSPA metric contains switching, localization,
    % missed target, and false track error components.
    %
    % This is the Simulink implementation of trackGOSPAMetric.
    %
    % See also: trackGOSPAMetric
    
    % Copyright 2020 The MathWorks, Inc.

    %#codegen
 
    % Public, non-tunable properties
    properties (Nontunable)

        % EnableGOSPAOutput Enables GOSPA component in output
        %
        %   Default : false
        EnableGOSPAOutput (1, 1) logical = false;

        % EnableSwitchingGOSPAOutput Enables switching GOSPA component in output
        %
        %   Default : false
        EnableSwitchingGOSPAOutput (1, 1) logical = false;

        % EnableLocalizationGOSPAOutput Enables localization GOSPA component in output
        %
        %   Default : false
        EnableLocalizationGOSPAOutput (1, 1) logical = false;

        % EnableMissedTargetGOSPAOutput Enables missed target GOSPA component in output
        %
        %   Default : false
        EnableMissedTargetGOSPAOutput (1, 1) logical = false;

        % EnableFalseTrackGOSPAOutput Enables false track GOSPA component in output
        %
        %   Default : false
        EnableFalseTrackGOSPAOutput (1, 1) logical = false;
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
    
    properties (Hidden, Constant)
        TrackFormatSet = matlab.system.StringSet({'objectTrack','custom'});
        TruthFormatSet = matlab.system.StringSet({'Platform','Actor','custom'});
    end

    methods
        function obj = trackGOSPAMetric(varargin)
            obj = obj@trackGOSPAMetric(varargin{:});            
            setProperties(obj,nargin,varargin{:}); 
        end        
    end

    methods(Access = protected)
        function setupImpl(obj,varargin)           
            %Extract input from bus
            [tracks,truths] = getInputFromBus(obj,varargin{:});
            
            setupImpl@trackGOSPAMetric(obj,tracks,truths);        
        end

        function varargout = stepImpl(obj, varargin)                       
            %Extract input from bus
            [tracks,truths] = getInputFromBus(obj,varargin{:});                                                
            
            %Localization, missed target and false track components of
            %GOSPA are available only if Alpha is 2.
            numOutput = 3 + (3 * isequal(obj.Alpha,2));
            outputComponents = cell(1,numOutput);
            
            [outputComponents{:}] = stepImpl@trackGOSPAMetric(obj,tracks,truths,varargin{3:end}); 
            
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
            if ~isequal(obj.Alpha,2)
                %Error if Alpha is not 2 and localization, missed target
                %and false track components are selected.
                coder.internal.assert(~obj.EnableLocalizationGOSPAOutput,'fusion:simulink:trackGOSPAMetric:AlphaError');
                coder.internal.assert(~obj.EnableMissedTargetGOSPAOutput,'fusion:simulink:trackGOSPAMetric:AlphaError');
                coder.internal.assert(~obj.EnableFalseTrackGOSPAOutput,'fusion:simulink:trackGOSPAMetric:AlphaError');
            end                         
            
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
                obj.pTruthIdentifierFcn = obj.pBuiltIns.getTruthIdentifier;
            end
            
            %validate distance function
            validateDistanceFcn(obj);
            
            %validate GOSPA properties
            validateGOSPAProperties(obj);
            
            %Set a logical array to select from different output
            %components.
            setOutputSelector(obj);
        end
       
        function flag = isInputSizeMutableImpl(obj,index)            
            flag = checkInputSizeMutability(obj,index);
        end

        function icon = getIconImpl(~)
            icon = getString(message('fusion:simulink:trackGOSPAMetric:GOSPAIcon'));
        end
        
        function varargout = getInputNamesImpl(obj)            
            varargout = getOSPAInputNames(obj,obj.HasAssignmentInput);
        end

        function num = getNumInputsImpl(obj)
            num = getOSPAInputNum(obj,obj.HasAssignmentInput);
        end
                      
        function varargout = getOutputNamesImpl(obj)                                                            
            outputNames = { getString(message('fusion:simulink:trackGOSPAMetric:GOSPAWithSwitchingOutput')), ...            
                            getString(message('fusion:simulink:trackGOSPAMetric:GOSPAOutput')), ...                
                            getString(message('fusion:simulink:trackGOSPAMetric:SwitchingGOSPAOutput')), ...
                            getString(message('fusion:simulink:trackGOSPAMetric:LocalizationGOSPAOutput')), ...
                            getString(message('fusion:simulink:trackGOSPAMetric:MissedTargetGOSPAOutput')), ...
                            getString(message('fusion:simulink:trackGOSPAMetric:FalseTrackGOSPAOutput'))};                            
            varargout = outputNames(obj.pOutputSelector); 
        end                           

        function numOutput = getNumOutputsImpl(obj)            
            numOutput = 1 + obj.EnableGOSPAOutput + obj.EnableLocalizationGOSPAOutput + ...
                obj.EnableSwitchingGOSPAOutput + obj.EnableFalseTrackGOSPAOutput + ...
                obj.EnableMissedTargetGOSPAOutput;                
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
            flag = flag || isInactivePropertyImpl@trackGOSPAMetric(obj,prop);
        end
    end
    
    methods (Access = protected)
        function loadObjectImpl(obj, s, wasLocked)
            % Set private and protected properties
            loadObjectImpl@trackGOSPAMetric(obj,s,wasLocked)
            loadProtectedProperties(obj,s,wasLocked);
        end
        
        function s = saveObjectImpl(obj)
            sObj = saveObjectImpl@trackGOSPAMetric(obj);
            s = saveProtectedProperties(obj,sObj);
        end
        function c = cloneImpl(obj)
            c = cloneImpl@trackGOSPAMetric(obj);
            copyProtectedProperties(obj,c);
        end
    end
    
    methods (Access=private)      
        function setOutputSelector(obj)    
            %Set a logical array to select from different output
            %components.First component is always available and rest of
            %the components are selected as per user input.
            obj.pOutputSelector = [true,obj.EnableGOSPAOutput,obj.EnableSwitchingGOSPAOutput, ...
                obj.EnableLocalizationGOSPAOutput,obj.EnableMissedTargetGOSPAOutput, ...
                obj.EnableFalseTrackGOSPAOutput];                                  
        end
        
    end

    % Block icon and dialog customizations
    methods (Static, Access = protected)
        function header = getHeaderImpl
            %Block header and title text.
            header = matlab.system.display.Header(...
                'trackGOSPAMetric', ...
                'Title', getString(message('fusion:block:gospaMetricTitle')), ...
                'Text',  getString(message('fusion:block:gospaMetricDesc')));
        end
      
        function groups = getPropertyGroupsImpl
            %Block property sections and groups.
            GOSPAProperties = {'CutoffDistance','Order','Alpha'};
            GOSPAPropertySection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','GOSPAPropertySection',GOSPAProperties);

            DistanceProperties = {'Distance','MotionModel','DistanceFcn'};
            DistancePropertySection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','DistancePropertySection',DistanceProperties);

            LabelingProperties = {'SwitchingPenalty'};
            LabelingPropertySection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','LabelingPropertySection',LabelingProperties);

            OutputProperties = {'EnableGOSPAOutput','EnableSwitchingGOSPAOutput','EnableLocalizationGOSPAOutput', ...
                                    'EnableMissedTargetGOSPAOutput','EnableFalseTrackGOSPAOutput'};
            OutputSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:trackGOSPAMetric','OutputSection',OutputProperties);

            InputProperties = {'HasAssignmentInput'};
            InputSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                'simulink:OSPASimulinkBase','InputSection',InputProperties);
            
            InputBusFormatSection = fusion.simulink.OSPASimulinkBase.getInputBusFormatSection();

            PropertyGroup = matlab.system.display.SectionGroup(...
                'Title',getString(message('fusion:simulink:OSPASimulinkBase:TitlePropertyGroup')), ...
                'Sections',[GOSPAPropertySection,DistancePropertySection,LabelingPropertySection]);


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
