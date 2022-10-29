classdef OSPABase < matlab.System
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2019 The MathWorks, Inc.
    
    % A base class for OSPA metric computation. The base class provides
    % routines for common computations across OSPA metrics.
    
    %#codegen
    
    % OSPA properties
    properties
        % CutoffDistance Threshold for distance between track and truth (c)
        % Specify the cut-off distance between tracks and truths as a real,
        % scalar and positive value. If the computed distance is higher than
        % the threshold, the distance is saturated to the CutoffDistance.
        %   Default: 30
        %
        CutoffDistance = 30;
        
        % Order Order of the metric (p)
        %   Specify the order of metric computation as a real scalar value.
        %
        %   Default: 2
        Order = 2;
    end
    
    % Distance properties
    properties (Nontunable)
        %MotionModel  Specifies desired motion model
        %   Select the motion model used by the tracks input by choosing one of
        %   'constvel', 'constacc' 'constturn' or 'singer'.  These expect
        %   the 'State' field to have a column vector as follows:
        %
        %      'constvel'  - position     in elements [1 3 5]
        %                    velocity     in elements [2 4 6]
        %
        %      'constacc'  - position     in elements [1 4 7]
        %                    velocity     in elements [2 5 8]
        %                    acceleration in elements [3 6 9]
        %
        %      'constturn' - position     in elements [1 3 6]
        %                    velocity     in elements [2 4 7]
        %                    yaw rate     in element  [5]
        %
        %      'singer'    - position     in elements [1 4 7]
        %                    velocity     in elements [2 5 8]
        %                    acceleration in elements [3 6 9]
        %
        %   Each option above also expects the 'StateCovariance' field to have
        %   position and velocity information in the rows and columns
        %   corresponding to the 'State' property positional input selector.
        %
        %Default:  'constvel'
        MotionModel = 'constvel';
        
        % Distance Specifies the physical quantity for distance
        %   Specify the distance to use between a track and a truth for
        %   computation of OSPA metric. You can choose one of the
        %   following:
        %      'posnees'   - the normalized estimation error squared (NEES)
        %                    in track position
        %      'velnees'   - the NEES in track velocity
        %      'posabserr' - the absolute error in track position
        %      'velabserr' - the absolute error in track velocity
        %      'custom'    - A custom distance between track and truth.
        %   If you specify Distance as 'custom', you must specify the
        %   DistanceFcn property.
        %
        %   Default: 'posnees'
        Distance = 'posnees'
        
        % DistanceFcn Specify a custom function for computation of
        % distance between track and truth. The function must support the
        % following syntax
        %   dij = DistanceFcn(Track, Truth)
        % where:
        %   Track is an element of tracks array passed to the step method
        %   Truth is an element of truths array passed to the step method
        %
        %   Default : none
        DistanceFcn = ''
    end
    
    % Labelled tracks computation properties
    properties (Nontunable)
        %TrackIdentifierFcn  Handle to a function returning unique track identifiers
        %   Specifies the track identifiers for the TRACK input of the step
        %   function.  The track identifiers should be unique string or numeric
        %   values.
        %
        %   The function must have the following syntax
        %      TRACKID = TRACKIDENTIFIER(TRACK)
        %
        %   TRACKID is a numeric array of the same size as TRACK.
        %   TRACK is the array passed into the step method.
        %
        %   The default identification function assumes TRACK is an array of
        %   struct or class with a "TrackID" fieldname or property.
        TrackIdentifierFcn = ''
        
        %TruthIdentifierFcn  Handle to a function returning unique truth identifiers
        %   Specifies the truth object identifiers for the TRUTH input of the
        %   step function.  The truth identifiers should be unique string or
        %   numeric values.
        %
        %   The function must have the following syntax
        %      TRUTHID = TRUTHIDENTIFIER(TRUTH)
        %
        %   TRUTHID is a numeric array of the same size as TRUTH.
        %   TRUTH is the array passed into the step method.
        %
        %   The default identification function assumes TRUTH is an array of
        %   struct or class with a 'PlatformID' fieldname or property.
        TruthIdentifierFcn = ''
    end
    
            
    % Private properties
    properties (Access = {?fusion.internal.metrics.OSPABase,?matlab.unittest.TestCase})
        pLastAssignment
    end
    
    properties (Nontunable, Access = {?fusion.internal.metrics.OSPABase,?matlab.unittest.TestCase})
        pTruthIdentifierFcn
        pTrackIdentifierFcn
    end
    
    properties (Abstract, Nontunable)
        % HasAssignmentInput A flag to enable the assignment input to the
        % step method. This is an abstract property as each inheriting
        % class may need a different help text for it.
        HasAssignmentInput (1, 1) logical
    end
    
    properties (Nontunable, Access = {?fusion.internal.metrics.OSPABase,?matlab.unittest.TestCase})
        pDataType
        pDistanceFcn
        pIsBuiltInDistance
        pBuiltIns
    end
    
    properties (Hidden, Constant)
        MotionModelSet = matlab.system.StringSet({'constvel','constacc','constturn','singer'});
        DistanceSet = matlab.system.StringSet({'posnees','velnees','posabserr','velabserr','custom'});
    end
    
    methods
        function obj = OSPABase(varargin)
            setProperties(obj,numel(varargin),varargin{:});
            obj.pBuiltIns = fusion.internal.metrics.OSPABuiltins;
            
            if coder.target('MATLAB')
                setupDefaultIdentifiers(obj);
            end            
        end
    end
    % Setters and getters
    methods
        function set.CutoffDistance(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','nonsparse','scalar','positive'}, mfilename,'CutoffDistance');
            obj.CutoffDistance = val;
        end
                
        function set.Order(obj, val)
            validateattributes(val, {'numeric'}, {'real','finite','nonsparse','scalar','positive'}, mfilename,'Order');
            obj.Order = val;
        end

        function set.DistanceFcn(obj, val)
            validateattributes(val, {'function_handle','char','string'},{},mfilename,'DistanceFcn');
            obj.DistanceFcn = val;
        end
            
        function set.TrackIdentifierFcn(obj, val)
            validateattributes(val, {'function_handle','char','string'},{},mfilename,'TrackIdentifierFcn');
            obj.TrackIdentifierFcn = val;
        end
        
        function set.TruthIdentifierFcn(obj, val)
            validateattributes(val, {'function_handle','char','string'},{},mfilename,'TruthIdentifierFcn');
            obj.TruthIdentifierFcn = val;
        end
    end
    
    methods (Access = protected)
        function [locOspa, cardOspa, optimalAssignment] = stepImpl(obj, tracks, truths, varargin)
            % Retrieve OSPA parameters from properties
            % m and n value for OSPA computation
            validateOSPAInputs(obj, tracks, truths, varargin{:});
            
            % order and cut-off distance
            p = obj.Order;
            c = obj.CutoffDistance;
            
            % Compute distance matrix
            dMatrix = getDistanceMatrix(obj, tracks, truths);
            
            [numTracks, numTruths] = size(dMatrix);
            n = max(numTracks, numTruths);
            m = min(numTracks, numTruths);
            
            % Compute optimal assignment.
            cN = c^p/2; % cost of non-assignment.
            % For OSPA, we want to assign each m object. To avoid running
            % into numerical edge-cases, where cij + cji > 2*cNonAssign.
            % Just double cN to make sure.
            optimalAssignment = assignjv(dMatrix, 2*cN); 
            
            % Localization OSPA
            ind = sub2ind([numTracks numTruths],optimalAssignment(:,1),optimalAssignment(:,2));
            dMatrixCol = dMatrix(:);
            locOspa = (1/n*sum(dMatrixCol(ind)))^(1/p);
            locOspa(n == 0 && m == 0) = 0;

            % Cardinality OSPA
            cardOspa = (c^p*(n - m)/n)^(1/p);
            % zero OSPA for 0 tracks and 0 truths.
            cardOspa(n == 0 && m == 0) = 0;
        end
        
        function setupImpl(obj,tracks, truths)
            % Setup data type of the object
            if ~coder.internal.is_defined(obj.pDataType)
                if ~isempty(tracks) && ~isempty(truths)
                    d = obj.pDistanceFcn(tracks(1),truths(1));
                    obj.pDataType = class(d);
                else
                    obj.pDataType = 'double';
                end
            end
            
            % Assignments can include nan or 0. Use a numeric class here
            lastAssignment = zeros(0,2,obj.pDataType);
            coder.varsize('lastAssignment',[inf 2],[1 0]);
            obj.pLastAssignment = lastAssignment;
        end
        
        function validateInputsImpl(obj, tracks, truths, varargin)
           validateOSPAInputs(obj, tracks, truths, varargin{:});
        end        
            
        function validatePropertiesImpl(obj)
            %validate properties and setup default values.            
            validateDistanceFcn(obj);
            setupDefaultIdentifiers(obj);
            validateTrackIdentifierFcn(obj)
            validateTruthIdentifierFcn(obj);
        end              
        
        function flag = isInactivePropertyImpl(obj, prop)
            %DistanceFcn inactive unless Distance is 'custom'
            flag = strcmpi(prop,'DistanceFcn') && ~strcmpi(obj.Distance,'custom');
            
            % MotionModel inactive when Distance is 'custom'
            flag = flag || strcmpi(prop,'MotionModel') && strcmpi(obj.Distance, 'custom');
        end
    
        function num = getNumInputsImpl(obj)
            num = 2 + obj.HasAssignmentInput;
        end
    end
    
    methods (Access = protected)
        function loadObjectImpl(obj, s, wasLocked)
            % Set private and protected properties
            if wasLocked
                obj.pDistanceFcn = s.pDistanceFcn;
                obj.pIsBuiltInDistance = s.pIsBuiltInDistance;
                obj.pDataType = s.pDataType;
                obj.pTrackIdentifierFcn = s.pTrackIdentifierFcn;
                obj.pTruthIdentifierFcn = s.pTruthIdentifierFcn;
                obj.pLastAssignment = s.pLastAssignment;
            end
            obj.pBuiltIns = s.pBuiltIns;
            % Don't use 'DistanceFcn' if Distance is not 'custom'. It
            % defaults to a empty double on saving.
            if ~strcmpi(s.Distance,'custom')
                s = rmfield(s,'DistanceFcn');
            end
            loadObjectImpl@matlab.System(obj,s,wasLocked)
        end
        
        function s = saveObjectImpl(obj)
            s = saveObjectImpl@matlab.System(obj);
            s.pBuiltIns = obj.pBuiltIns;
            if isLocked(obj)
                s.pDistanceFcn = obj.pDistanceFcn;
                s.pIsBuiltInDistance = obj.pIsBuiltInDistance;
                s.pDataType = obj.pDataType;
                s.pTrackIdentifierFcn = obj.pTrackIdentifierFcn;
                s.pTruthIdentifierFcn = obj.pTruthIdentifierFcn;
                s.pLastAssignment = obj.pLastAssignment;
            end
        end
        
        function resetImpl(obj)
            % Restore the state
            obj.pLastAssignment(:) = 0;
        end
        
        function releaseImpl(obj)
            % Restore state
            obj.pLastAssignment(:) = 0;
        end
    end
    
    methods (Access = {?fusion.internal.metrics.OSPABase,?matlab.unittest.TestCase})
        function indexedAssignment = convertIdentifierToIndices(~, identAssignment, trackIds, truthIds)
            % Convert an identifier assignment to indexedAssignment.
            % assignment is a N-by-2 matrix with TrackIDs on first column
            % and TruthIDs on second column. Each column can have 0's which
            % represent invalid assignments.
            
            % truthIDs are identities of current input truths.
            % trackIDs are identities of current input tracks.
            valid = identAssignment(:,1) > 0 & identAssignment(:,2) > 0;
            trackIdsAssignment = identAssignment(valid,1);
            truthIdsAssignment = identAssignment(valid,2);
            
            % Compute indexed assignment
            [isAssignedTrackInInput, trackIndex] = ismember(trackIdsAssignment, trackIds);
            [isAssignedTruthInInput, truthIndex] = ismember(truthIdsAssignment, truthIds);
            
            % Assert that each assignment track must be in the list of
            % input tracks
            if ~all(isAssignedTrackInInput)
                tracksInAssignmentNotInInput = trackIdsAssignment(~isAssignedTrackInInput);
                coder.internal.error('fusion:internal:OSPABase:TrackNotInList',tracksInAssignmentNotInInput(1));
            end
            if ~all(isAssignedTruthInInput)
                truthsInAssignmentNotInInput = truthIdsAssignment(~isAssignedTruthInInput);
                coder.internal.error('fusion:internal:OSPABase:TruthNotInList',truthsInAssignmentNotInInput(1));
            end
            
            % Assemble assignment
            indexedAssignment = [trackIndex truthIndex];
        end
        
        function identifierAssignment = convertIndexToIdentifiers(~, indexedAssignment, trackIds, truthIds)
            % Given current indexed assignment, convert it to assignment of
            % identifiers
            % indexedAssignment is a N-by-2 matrix with indices of tracks on
            % first column and indices of truths on second column.
            
            identifierAssignment = [trackIds(indexedAssignment(:,1)) truthIds(indexedAssignment(:,2))];
        end
        
        function validateDistanceFcn(obj)
             % If Distance is not custom
            if strcmpi(obj.Distance,'custom')
                % Assert that DistanceFcn is specified
                coder.internal.assert(coder.internal.is_defined(obj.DistanceFcn),'fusion:internal:OSPABase:DistanceFcnUndefined');
                % Set the distance fcn
                if isa(obj.DistanceFcn,'function_handle')
                    obj.pDistanceFcn = obj.DistanceFcn;
                else
                    obj.pDistanceFcn = str2func(obj.DistanceFcn);
                end
                obj.pIsBuiltInDistance = false;
            else
                % Set it using builtins
                obj.pDistanceFcn = obj.pBuiltIns.getDistanceFcn(obj.MotionModel,obj.Distance);
                obj.pIsBuiltInDistance = true;
            end
        end        
            
        function validateTruthIdentifierFcn(obj)
             % Set Truth Identifier
            if isa(obj.TruthIdentifierFcn,'function_handle')
                obj.pTruthIdentifierFcn = obj.TruthIdentifierFcn;
            else
                obj.pTruthIdentifierFcn = str2func(obj.TruthIdentifierFcn);
            end
        end
           
        function validateTrackIdentifierFcn(obj)
            % Set Track identifier
            if isa(obj.TrackIdentifierFcn,'function_handle')
                obj.pTrackIdentifierFcn = obj.TrackIdentifierFcn;
            else
                obj.pTrackIdentifierFcn = str2func(obj.TrackIdentifierFcn);
            end 
        end
        
        function setupDefaultIdentifiers(obj)
            if ~coder.internal.is_defined(obj.TruthIdentifierFcn)
                obj.TruthIdentifierFcn = obj.pBuiltIns.getTruthIdentifier;
            end
            if ~coder.internal.is_defined(obj.TrackIdentifierFcn)
                obj.TrackIdentifierFcn = obj.pBuiltIns.getTrackIdentifier; 
            end
        end
        
        function validateOSPAInputs(obj, tracks, truths, varargin)
            validateTracks(obj, tracks);
            validateTruths(obj, truths);
            validateAssignments(obj, varargin{:});
        end
        
        function validateTracks(obj, tracks)
            % Validate if using built-in distance functions.
            if obj.pIsBuiltInDistance
                if ~isempty(tracks)
                    coder.internal.assert(isfieldorprop(tracks(1),'State'),'fusion:internal:OSPABase:InvalidBuiltInTrackStructure');
                    % If distance is set to 'posnees' or 'velnees', assert
                    % StateCovariance as field
                    cond = ~isfieldorprop(tracks(1), 'StateCovariance') && (strcmpi(obj.Distance,'posnees') || strcmpi(obj.Distance,'velnees'));
                    coder.internal.assert(~cond,'fusion:internal:OSPABase:InvalidBuiltInTrackStructureCovarianceRequired');
                    n = numel(tracks(1).State);
                    x = strcmp(obj.MotionModel,{'constvel','constacc','constturn','singer'});
                    y = [6 9 7 9];
                    coder.internal.assert(isequal(n,y(x)) && iscolumn(tracks(1).State),'fusion:internal:OSPABase:InvalidState',obj.MotionModel);
                end
            end
        end
        
        function validateTruths(obj, truths)
            % Validate if using built-in distance functions.
            if obj.pIsBuiltInDistance
                if ~isempty(truths)
                    cond = isfieldorprop(truths(1),'Position') && isfieldorprop(truths(1),'Velocity');
                    coder.internal.assert(cond,'fusion:internal:OSPABase:InvalidBuiltInTruthStructure');
                    coder.internal.assert(isrow(truths(1).Position) && isrow(truths(1).Velocity),'fusion:internal:OSPABase:InvalidTruthState');
                end
            end
        end
        
        function validateAssignments(~, assignment)
            if nargin > 1
                validateattributes(assignment,{'numeric'},{'real','nonsparse','ncols',2},mfilename,'assignments',3);
                % TrackIDs must be unique
                coder.internal.assert(numel(assignment(:,1)) == numel(unique(assignment(:,1))),'fusion:internal:OSPABase:uniqueTrackAssignment');
                
                % TruthIDs must be unique
                coder.internal.assert(numel(assignment(:,2)) == numel(unique(assignment(:,2))),'fusion:internal:OSPABase:uniqueTruthAssignment');
            end
        end
        
        function dMatrix = getDistanceMatrix (obj, tracks, truths, truncate)
            if nargin == 3
                truncate = true;
            end
            dMatrix = zeros(numel(tracks), numel(truths), obj.pDataType);
            for i = 1:numel(tracks)
                for j = 1:numel(truths)
                    dMatrix(i,j) = obj.pDistanceFcn(tracks(i), truths(j));
                end
            end
            if truncate
                dMatrix(dMatrix > obj.CutoffDistance) = obj.CutoffDistance;
            end
            % Raise to the order
            dMatrix = dMatrix.^(obj.Order);
        end
        
       
        
        function [trackIds, truthIds] = getInputIdentities(obj, tracks, truths)
            trackIds = getTrackIdentities(obj, tracks);
            truthIds = getTruthIdentities(obj, truths);
        end
        
        function tids = getTrackIdentities(obj, tracks)
            % Obtain track identities
            ids = obj.pTrackIdentifierFcn(tracks);
            tids = ids(:);
        end
        
        function tids = getTruthIdentities(obj, truths)
            % Obtain truth identities
            ids = obj.pTruthIdentifierFcn(truths);
            tids = ids(:);
        end
    end
    
    % Is allowed in system block method.
    methods (Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end

function flag = isfieldorprop(dataStruct, propName)
% Return true if a dataStruct has a field or property named propName
if coder.target('MATLAB')
    flag = isprop(dataStruct,propName) || isfield(dataStruct,propName);
else
    % Arrays of objects are not supported in codegen. This must be a
    % struct.
    flag = isfield(dataStruct,propName);
end
end