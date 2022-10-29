classdef fuserSourceConfiguration < fusion.internal.AbstractFusingConfiguration
%fuserSourceConfiguration  Configuration of a source used with track fuser
%  config = fuserSourceConfiguration(sourceIndex) creates a configuration
%  to be used with a track fuser. You must specify sourceIndex as a
%  positive integer. The other properties of the configuration are set to
%  default values.
%
%  config = fuserSourceConfiguration(...,'Name',value) allows specifying
%  additional properties using name-value pairs. 
%
%  fuserSourceConfiguration properties:
%    SourceIndex                 - Unique identifier for the source system
%    IsInternalSource            - Define if the source is internal to the 
%                                  fuser
%    IsInitializingCentralTracks - Define if this source can initialize a 
%                                  central track
%    LocalToCentralTransformFcn  - A function used to transform a local
%                                  track to a central track
%    CentralToLocalTransformFcn  - A function used to transform a central
%                                  local to a source track
%
% % Example:
% % ========
% % Create a fusion configuration for source number 3
% config = fuserSourceConfiguration(3);
% disp(config)
%
% See also: trackFuser, objectTrack

% Copyright 2019 The MathWorks, Inc.

%#codegen
    properties
        %SourceIndex Unique identifier for the source system
        % Define the source index as a positive integer. 
        %
        % Default: none
        SourceIndex
        
        %IsInternalSource Define if the source is internal to the fuser
        % Set this flag to true if the source is considered as internal to
        % the track fuser. The tracks from an internal source are fused
        % even if they are not self reported. In contrast, set this flag to
        % false to prevent the fuser from fusing tracks that are not self
        % reported (i.e., they are repeated and may cause rumors) by the
        % source.
        % For example, if the fuser is at the vehicle level, a tracking
        % radar installed on this vehicle is considered internal, while
        % another vehicle that reports fused tracks is considered external.
        %
        % Default: true
        IsInternalSource
        
        %IsInitializingCentralTracks Define if this source can initialize a 
        %                            central track
        % Set this flag to true if the tracks coming from this source can
        % initialize a central track. 
        %
        % Default: true
        IsInitializingCentralTracks
    end
    properties(Dependent)
        %LocalToCentralTransformFcn Function to transform a track from
        %                           local to central state space
        % Define the function that transforms a track from source
        % coordinate frame to fuser coordinate frame. 
        %
        % Default: @(track)track
        LocalToCentralTransformFcn
        
        %CentralToLocalTransformFcn Function to transform a track from
        %                           central to local state space
        % Define the function that transforms a track from fuser coordinate
        % frame to source coordinate frame.
        %
        % Default: @(track)track
        CentralToLocalTransformFcn
    end
    
    properties(Access=protected)
        pLocalToCentralTransformFcn
        pCentralToLocalTransformFcn
        pIsTransformToCentralValid = false
        pIsTransformToLocalValid = false
    end
    
    methods % Constructor
        function obj = fuserSourceConfiguration(varargin)
            % Is SourceIndex specified as property or first variable
            locIndexID = fusion.internal.findProp('SourceIndex',varargin{:});
            if ~(locIndexID <= numel(varargin) - 1)
                coder.internal.assert(mod(numel(varargin)+1,2) == 0,...
                    'fusion:fuserSourceConfiguration:PropertyNeededOnConstruction','SourceIndex');
                sourceIndex = varargin{1};
            else
                sourceIndex = varargin{locIndexID+1};
            end
            validateattributes(sourceIndex,{'numeric'},{'scalar','nonsparse',...
                'integer','positive'},mfilename,'SourceIndex',1);
            
            obj.SourceIndex = sourceIndex;
            
            if nargin == 1 || (nargin == 2 && strcmpi(varargin{1},'SourceIndex'))
                args = {};
            elseif mod(nargin,2) == 1
                args = {varargin{2:end}};
            else
                args = {varargin{:}};
            end
            setProperties(obj,args{:});
        end
    end
    
    methods (Hidden)
        % Public methods that must be supported by the object to implement
        % fusion.internal.AbstractFusingConfiguration
        function centralTrack = transformToCentral(obj,localTrack)
            %transformToCentral Transform a source track to a central track
            %  centralTrack = transformToCentral(obj,sourceTrack) transforms
            %  a source track to a central-level track using the
            %  property LocalToCentralTransformFcn.
            
            if ~obj.pIsTransformToCentralValid
                validateTransformToCentral(obj, localTrack);
            end
            centralTrack = obj.LocalToCentralTransformFcn(localTrack);
        end
        
        function localTrack = transformToLocal(obj,centralTrack)
            %transformToCentral Transform a central track to a source track
            %  sourceTrack = transformToCentral(obj,centralTrack) transforms
            %  a central-level track to a source track using the
            %  property CentralToLocalTransformFcn.cd
            
            if ~obj.pIsTransformToLocalValid
                validateTransformToLocal(obj, centralTrack);
            end
            localTrack = obj.CentralToLocalTransformFcn(centralTrack);
        end
        
        function sync(obj,miniObject)
            % sync: Synchronizes configuration with a mini configuration
            % which can be a struct or a fuserSourceConfiguration. While
            % syncing the miniObject this method make sure that
            % 1. SourceIndex is same.
            % 2. IsInternalSource is same.
            % 3. IsInitializingCentralTracks is same.
            %
            % Validation is performed via set method of each property. 
            validateattributes(miniObject,{'struct','fuserSourceConfiguration'},{'scalar'},'fuserSourceConfiguration');
            if isstruct(miniObject)
                fieldsSpecified = fieldnames(miniObject);
                if coder.target('MATLAB')
                    isFieldAMember = ismember(fieldsSpecified,{'SourceIndex','IsInternalSource','IsInitializingCentralTracks'});
                else
                    isFieldAMember = false(numel(fieldsSpecified),1);
                    for i = 1:numel(fieldsSpecified)
                        isFieldAMember(i) = any(strcmpi(fieldsSpecified{i},{'SourceIndex','IsInternalSource','IsInitializingCentralTracks'}));
                    end
                end
                if ~all(isFieldAMember)
                    firstInvalidProp = (fieldsSpecified(find(~isFieldAMember,true,'first')));
                    coder.internal.assert(false,'fusion:fuserSourceConfiguration:invalidProp',firstInvalidProp{1});
                end
                for i = 1:numel(fieldsSpecified)
                    obj.(fieldsSpecified{i}) = miniObject.(fieldsSpecified{i});
                end
            else
                % Only sync tunable properties
                obj.SourceIndex = miniObject.SourceIndex;
                obj.IsInternalSource = miniObject.IsInternalSource;
                obj.IsInitializingCentralTracks = miniObject.IsInitializingCentralTracks;
            end
        end
        
        function clonedObj = clone(obj)
            %CLONE  Create a copy of the fusing  configuration
            %  clonedObj = CLONE(obj) returns a fuserSourceConfiguration
            %  object with the same properties as the object.
            clonedObj = fuserSourceConfiguration(...
                'SourceIndex', obj.SourceIndex, ...
                'IsInternalSource', obj.IsInternalSource, ...
                'IsInitializingCentralTracks',obj.IsInitializingCentralTracks,...
                'LocalToCentralTransformFcn', obj.LocalToCentralTransformFcn,...
                'CentralToLocalTransformFcn', obj.CentralToLocalTransformFcn);
        end

        function newStruct = toStruct(obj)           
            % TOSTRUCT Convert fuserSourceConfiguration object to 
            % structure.
            % newStruct = TOSTRUCT(obj) returns a structure equivalent to
            % obj.

            newStruct = struct(...
                'SourceIndex', obj.SourceIndex, ...
                'IsInternalSource', obj.IsInternalSource, ...
                'IsInitializingCentralTracks',obj.IsInitializingCentralTracks,...
                'LocalToCentralTransformFcn', func2str(obj.LocalToCentralTransformFcn),...
                'CentralToLocalTransformFcn', func2str(obj.CentralToLocalTransformFcn));
        end
    end
    
    methods
        % Setters, getters
        function set.SourceIndex(obj,val)
            validateattributes(val,{'numeric'},{'scalar','integer','positive'},...
                mfilename,'SourceIndex');
            obj.SourceIndex = val;   
        end
        
        function set.IsInternalSource(obj,val)
            validateattributes(val,{'logical','numeric'},{'binary','scalar'},...
                mfilename,'IsInternalSource');
            obj.IsInternalSource = val;
        end
        
        function set.IsInitializingCentralTracks(obj,val)
            validateattributes(val,{'logical','numeric'},{'binary','scalar'},...
                mfilename,'IsInitializingCentralTracks');
            obj.IsInitializingCentralTracks = val;
        end
        
        function set.LocalToCentralTransformFcn(obj,func)
            validateattributes(func,{'function_handle','char','string'},{'nonempty'},...
                mfilename,'LocalToCentralTransformFcn');
            if isa(func,'function_handle')
                obj.pLocalToCentralTransformFcn = func;
            else
                validateattributes(func,{'string','char'},{'scalartext'},mfilename,'LocalToCentralTransformFcn')
                obj.pLocalToCentralTransformFcn = str2func(func);
            end
        end
        
        function value = get.LocalToCentralTransformFcn(obj)
            value = obj.pLocalToCentralTransformFcn;
        end
        
        function set.CentralToLocalTransformFcn(obj,func)
            validateattributes(func,{'function_handle','char','string'},{'nonempty'},...
                mfilename,'CentralToLocalTransformFcn');
            if isa(func,'function_handle')
                obj.pCentralToLocalTransformFcn = func;
            else
                validateattributes(func,{'string','char'},{'scalartext'},mfilename,'CentralToLocalTransformFcn')
                obj.pCentralToLocalTransformFcn = str2func(func);
            end
        end
        
        function value = get.CentralToLocalTransformFcn(obj)
            value = obj.pCentralToLocalTransformFcn;
        end
    end
    
    methods (Access = protected)
        function validateTransformToCentral(obj,track)
            % This transform can fail in three ways: 
            %  1. The function provided by the user does not support
            %     objectTrack. In that case that error provided will be
            %     clear enough: 'MATLAB:UndefinedFunction'.
            %  2. The function works on objectTrack but returns a value
            %     that is not an objectTrack.
            %  3. The function returns an objectTrack but is not the
            %     inverse of trasnformation to local. 
            % For the last two validations, we use validateTransforms
            
            
            ct = obj.pLocalToCentralTransformFcn(track);
            
            % Validate #2:
            coder.internal.assert(isa(ct,'objectTrack') || (isa(ct,'struct') && isfield(ct,'State')),...
                'fusion:fuserSourceConfiguration:ExpectedObjectTrack',...
                'LocalToCentralTransformFcn','objectTrack or a struct with similar fields');
            
            % Validate #3:
            lt = obj.pCentralToLocalTransformFcn(ct);
            validateSimilarTracks(obj,lt,track);
            
            % If both pass, then transforms are valid
            obj.pIsTransformToCentralValid = true;
        end
        
        function validateTransformToLocal(obj,track)
            % This transform can fail in three ways: 
            %  1. The function provided by the user does not support
            %     objectTrack. In that case that error provided will be
            %     clear enough: 'MATLAB:UndefinedFunction'.
            %  2. The function works on objectTrack but returns a value
            %     that is not an objectTrack.
            %  3. The function returns an objectTrack but is not the
            %     inverse of trasnformation to local. 
            % For the last two validations, we use validateTransforms
            
            lt = obj.pCentralToLocalTransformFcn(track);
            
            % Validate #2:
            coder.internal.assert(isa(lt,'objectTrack') || (isa(lt,'struct') && isfield(lt, 'State')),...
                'fusion:fuserSourceConfiguration:ExpectedObjectTrack',...
                'LocalToCentralTransformFcn','objectTrack or a struct with similar fields');
            
            % Validate #3:
            ct = obj.pLocalToCentralTransformFcn(lt);
            validateSimilarTracks(obj,ct,track);
            
            % If both pass, then transforms are valid
            obj.pIsTransformToLocalValid = true;
        end
        
        function validateSimilarTracks(~,actTrack,expTrack)
            % For tracks to be similar, check that their state and state
            % covariance sizes are the same
            coder.internal.assert(all(size(actTrack.State)==size(expTrack.State)),...
                'fusion:fuserSourceConfiguration:ExpectedEqualSizes','State')
        end
    end
    % Methods related to construction
    methods (Access = protected)
        function setProperties(obj,varargin)
            if coder.target('MATLAB')
                [isinter, isinit, l2cfcn, c2lfcn] = parseMATLAB(obj,varargin{:});
            else
                [isinter, isinit, l2cfcn, c2lfcn] = parseCodegen(obj,varargin{:});
            end
            obj.IsInternalSource = isinter;
            obj.IsInitializingCentralTracks = isinit;
            obj.LocalToCentralTransformFcn = l2cfcn;
            obj.CentralToLocalTransformFcn = c2lfcn;
        end
        function [isinte, isinit, l2cfcn, c2lfcn] = parseMATLAB(obj,varargin)
            
            % Define parser
            defArgs = getDefaultArgs(obj);
            parser = inputParser;
            parser.addParameter('SourceIndex',                 defArgs.SourceIndex);
            parser.addParameter('IsInternalSource',            defArgs.IsInternalSource);
            parser.addParameter('IsInitializingCentralTracks', defArgs.IsInitializingCentralTracks);
            parser.addParameter('LocalToCentralTransformFcn',  defArgs.LocalToCentralTransformFcn);
            parser.addParameter('CentralToLocalTransformFcn',  defArgs.CentralToLocalTransformFcn);
            
            % Parse
            parser.parse(varargin{:});
            
            % Provide outputs
            isinte = parser.Results.IsInternalSource;
            isinit = parser.Results.IsInitializingCentralTracks;
            l2cfcn = parser.Results.LocalToCentralTransformFcn;
            c2lfcn = parser.Results.CentralToLocalTransformFcn;
        end
        function [isinte, isinit, l2cfcn, c2lfcn] = parseCodegen(obj,varargin)
            
            % Define parser
            defArgs = getDefaultArgs(obj);
            parms = struct( ...
                'SourceIndex',                 uint32(0), ...
                'IsInternalSource',            uint32(0), ...
                'IsInitializingCentralTracks', uint32(0), ...
                'LocalToCentralTransformFcn',  uint32(0), ...
                'CentralToLocalTransformFcn',  uint32(0) ... 
            );
        
            popt = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', false);
            
            % Parse
            optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});
            
            % Provide outputs
            isinte = eml_get_parameter_value(optarg.IsInternalSource,...
                defArgs.IsInternalSource, varargin{:});
            isinit = eml_get_parameter_value(optarg.IsInitializingCentralTracks,...
                defArgs.IsInitializingCentralTracks, varargin{:});
            l2cfcn = eml_get_parameter_value(optarg.LocalToCentralTransformFcn,...
                defArgs.LocalToCentralTransformFcn, varargin{:});
            c2lfcn = eml_get_parameter_value(optarg.CentralToLocalTransformFcn,...
                defArgs.CentralToLocalTransformFcn, varargin{:});
        end
    end

    methods (Access = {?fusionConfiguration, ?matlab.unittest.TestCase})
        function args = getDefaultArgs(~)
            args = struct(...
                'SourceIndex', 1, ...
                'IsInternalSource', true,...
                'IsInitializingCentralTracks', true,...
                'LocalToCentralTransformFcn',@(track) track,...
                'CentralToLocalTransformFcn',@(track) track);
        end
    end
end