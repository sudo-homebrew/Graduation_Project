classdef (Sealed) trackScoreLogic < matlabshared.tracking.internal.fusion.TrackLogic
%trackScoreLogic Confirm and delete tracks based on track score
%   logic = trackScoreLogic creates a track score logic with default
%   confirmation and deletion thresholds.
%   A track will be confirmed if the current track score is greater than or
%   equal to the ConfirmationThreshold. Track score is also known as the
%   log likelihood of a track.
%   A track will be deleted if the current track score has decreased
%   relative to the maximum track score by the DeletionThreshold.
%
%   ... = trackScoreLogic('ConfirmationThreshold',a,'DeletionThreshold',b)
%   allows you to specify the confirmation or deletion threshold. Both a
%   and b must be scalars.
%   Unspecified properties will have default values.
%
%   Use init to initialize the track score with the first 'hit'. 
%   Use hit to update the track score with any subsequent 'hit'.
%   Use miss to update the track score with any subsequent 'miss'.
%
% trackScoreLogic properties:
%   ConfirmationThreshold   - parameters controlling track confirmation
%   DeletionThreshold       - parameters controlling track deletion
%   Score                   - the current track logic score (read only)
%   MaxScore                - the maximum track logic score (read only)
%
% trackScoreLogic methods:
%   init                - initialize the track logic with first 'hit'
%   hit                 - update the track logic with a subsequent 'hit'
%   miss                - update the track with a 'miss'
%   sync                - synchronize the trackScoreLogic object
%   mergeScores         - update the track score by track merging
%   checkConfirmation   - check if the track should be confirmed
%   checkDeletion       - check if the track should be deleted
%   output              - get the current track score
%   reset               - resets the object state
%   clone               - creates a copy of the object
%
% Class support:
% --------------
%   If either ConfirmationThreshold or DeletionThreshold is given on
%   construction as single precision, the other will be converted to single
%   precision and the class will use single precision.
%
% Example: 
% --------
% % Create a score-based track logic
% scoreLogic = trackScoreLogic('ConfirmationThreshold', 20, ...
%   'DeletionThreshold', -5)
% wasInitialized = false;
% pd = 0.9;   % Probability of detection
% pfa = 1e-6; % Probability of false alarm
% volume = 1; % The volume of a sensor detection bin
% beta = 0.1; % New target rate in a unit volume
%
% % Update the score-based logic 3 times. See how the score increases with
% % a 'hit' and decreases with a 'miss'. By the end of the 3rd update, the
% % confirmation flag will be true.
% for i = 1:3
%     likelihood = 0.05 + 0.05*rand(1); % likelihood of the measurement
%     detectedFlag = logical(mod(i,2)); % Only even updates are true
%     if detectedFlag && ~wasInitialized
%         init(scoreLogic, volume, beta);
%         wasInitialized = true;
%     elseif detectedFlag && wasInitialized
%         hit(scoreLogic, volume, likelihood);
%     else
%         miss(scoreLogic);
%     end
%     confFlag = checkConfirmation(scoreLogic);
%     disp(['Score and MaxScore: ', num2str(output(scoreLogic)), ...
%       '.  Confirmation Flag is: ',num2str(confFlag)])
% end
%
% % Update the logic with 3 'miss' updates. The deletion flag will turn
% % true by the end of the 3rd miss.
% for i = 1:3
%     miss(scoreLogic);
%     deleteFlag = checkDeletion(scoreLogic);
%     disp(['Score and MaxScore: ', num2str(output(scoreLogic)), ...
%       '.  Deletion Flag is: ',num2str(deleteFlag)])
% end
%
% See also: trackerGNN, trackerTOMHT, trackHistoryLogic

% References:
% [1] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
% Tracking Systems", Artech House, 1999.

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen
    
    properties(Dependent)
        % ConfirmationThreshold - The minimum score for track confirmation
        %   Define the threshold for track confirmation as a real,
        %   positive, scalar. If the logic score is above this threshold,
        %   the track should be confirmed.
        %
        % Default: 20
        ConfirmationThreshold 
        
        % DeletionThreshold - The decrease in score for track deletion
        %   Define the threshold for track deletion as a real, negative,
        %   scalar. If the (current score - the maximum score) is more
        %   negative than this threshold, the track should be deleted.
        %
        % Default: -5
        DeletionThreshold
    end
    
    properties (GetAccess = public, SetAccess = private, Dependent)
        Score                   % The current track score
        MaxScore                % The maximum track score
    end

    properties (Access = private)
        %pConfirmationThreshold The threshold for confirmation
        pConfirmationThreshold 
        
        %pDeletionThreshold     The threshold for deletion
        pDeletionThreshold 
        
        %pScore                 The current track score
        pScore
        
        %pMaxScore              The current maximum score
        pMaxScore
        
        %pClassToUse            The class used for saving the properties
        pClassToUse
        
        %pIsFirstUpdate         True if object was not updated yet
        pIsFirstUpdate = true
    end
    
    % Default confirmation and deletion values
    properties (Access = {?trackScoreLogic, ?matlab.unittest.TestCase},...
            Constant = true)
        constDefaultConfirmation = 20;
        constDefaultDeletion = -5;
    end
    properties (Access = {?matlabshared.tracking.internal.fusion.TrackLogic, ?matlab.unittest.TestCase},...
            Constant = true)
        constPd = 0.9; % Default probability of detection
        constPfa = 1e-6; % Default probability of false alarm
    end
    
    % Implementing the TrackLogic interface
    methods
        function obj = trackScoreLogic(varargin)
            % Constructor to the trackScoreLogic object
            if coder.target('MATLAB')
                [confirmation,deletion] = parseInputsMATLAB(obj, varargin{:});
            else
                [confirmation,deletion] = parseInputsCodegen(obj, varargin{:});
            end
            
            % Set the class to use
            if isa(confirmation,'single') || isa(deletion,'single')
                obj.pClassToUse = 'single';
            else
                obj.pClassToUse = 'double';
            end
            
            setProperties(obj, confirmation, deletion);
        end
        
        function init(obj,volume,beta,varargin)
            %INIT Initialize a track score logic
            % INIT(obj, volume, beta) initializes the score on the first
            % 'hit' using the default values: pd=0.9, pfa=1e-6. volume is
            % the volume of the sensor detection bin. For example, a 2-D
            % radar will have a sensor bin volume of:
            %   (azimuth res. in radians) * (range) * (range res.).
            % beta is the rate of new targets in a unit volume.
            % 
            % INIT(obj, volume, beta, pd, pfa) allows you to specify
            % non-default values of pd and pfa.
            narginchk(3,5)
            coder.internal.assert(obj.pIsFirstUpdate, 'fusion:trackLogic:alreadyInitialized')
            [pd,pfa] = parsePdPfa(obj,varargin{:});
            scoreIncrement = fusion.internal.trackScoreInit(volume,beta,pd,pfa);
            keepScore(obj,scoreIncrement)
        end
        
        function miss(obj,varargin)
            %Miss Update the track score logic with a miss
            % MISS(obj) updates the track score in a case of a 'miss' using
            % the default values: pd=0.9 and pfa=1e-6.
            %
            % MISS(obj, pd, pfa) allows you to specify non-default values
            % of pd and pfa.
            narginchk(1,3)
            coder.internal.assert(~obj.pIsFirstUpdate, 'fusion:trackLogic:notInitialized','miss')
            [pd,pfa] = parsePdPfa(obj,varargin{:});
            scoreIncrement = fusion.internal.trackScoreUpdate(false,pd,pfa);
            keepScore(obj,scoreIncrement)
        end
        
        function hit(obj,volume,likelihood,varargin)
            %HIT Update the track score logic with a hit
            % HIT(obj, volume, lhood) updates the track score in a
            % case of a 'hit' using the default values: pd=0.9 and
            % pfa=1e-6. volume is the volume of the sensor detection bin.
            % For example, a 2-D radar will have a sensor bin volume of:
            %   (azimuth resolution) * (range) * (range resolution).
            % lhood is the likelihood of the detection being assigned
            % to the track. See the likelihood method of the tracking
            % filter.
            %
            % HIT(obj, volume, lhood, pd, pfa) allows you to specify
            % non-default values of pd and pfa.
            narginchk(3,5)
            coder.internal.assert(~obj.pIsFirstUpdate, 'fusion:trackLogic:notInitialized','hit')
            [pd,pfa] = parsePdPfa(obj,varargin{:});
            pdc = cast(pd,obj.pClassToUse);
            pfac = cast(pfa,obj.pClassToUse);
            scoreIncrement = fusion.internal.trackScoreUpdate(true,volume,likelihood,pdc,pfac);
            keepScore(obj, scoreIncrement)
        end
        
        function set.ConfirmationThreshold(obj, value)
            % set.ConfirmationThreshold(obj, value) sets the confirmation
            % score threshold
            validateattributes(value,{'single','double'}, ...
                {'real','finite','positive','scalar'}, mfilename, ...
                'ConfirmationThreshold')
            obj.pConfirmationThreshold = cast(value, obj.pClassToUse);
        end
        
        function value = get.ConfirmationThreshold(obj)
            % value = get.ConfirmationThreshold(obj) returns the score
            % threshold value used for track confirmation.
            value = obj.pConfirmationThreshold;
        end
        
        function set.DeletionThreshold(obj, value)
            % set.DeletionThreshold(obj, value) sets the confirmation
            % score threshold
            validateattributes(value,{'single','double'}, ...
                {'real','finite','scalar','<',0}, mfilename, ...
                'DeletionThreshold')
            obj.pDeletionThreshold = cast(value, obj.pClassToUse);
        end
        
        function value = get.DeletionThreshold(obj)
            % value = get.DeletionThreshold(obj) returns the score
            % threshold value used for track deletion.
            value = obj.pDeletionThreshold;
        end
        
        function value = output(obj)
            %OUTPUT  Return the current and maximum score of the object
            % [currentScore, maxScore] = OUTPUT(obj) returns the current
            % score, currentScore, and the maximum score, maxScore. 
            value = [obj.pScore, obj.pMaxScore];
        end
        
        function value = get.Score(obj)
            % score = get.Score(obj) returns the track score.
            value = obj.pScore;
        end
        
        function value = get.MaxScore(obj)
            % maxScore = get.MaxScore(obj) returns the track score.
            value = obj.pMaxScore;
        end
        
        function sync(this,that)
            %SYNC Synchronize the probabilities of two objects
            % SYNC(this,that) synchronizes the probability of this object
            % to be the same as the probability of that object
            validateattributes(that,{mfilename},{'scalar'},mfilename,'that');
            this.pScore = that.Score;
            this.pMaxScore = that.MaxScore;
            this.pIsFirstUpdate = ~isInitialized(that);
        end
        
        function mergeScores(this,that)
            % mergeScores(thisLogic,thatLogic) updates the score of
            % thisLogic by merging the score of thatLogic. Both thisLogic
            % and thatLogic must be trackScoreLogic objects.
            %
            % Score merging increases the score of thisLogic by:
            %   log(1+exp(-(thisLogic-thatLogic)))
            
            narginchk(2,2)
            validateattributes(this,{'trackScoreLogic'},{'scalar'},'mergeScores','thisLogic',1);
            validateattributes(that,{'trackScoreLogic'},{'scalar'},'mergeScores','thatLogic',2);
            mainScore = this.Score;
            mergedScore = that.Score;
            this.pScore = fusion.internal.trackScoreMerge(mainScore, mergedScore);
            
            % Update max score if needed
            if this.pScore > this.pMaxScore
                this.pMaxScore = this.pScore;
            end
        end
        
        function tf = checkConfirmation(obj)
            %checkConfirmation Check if the track should be confirmed
            % tf = checkConfirmation(obj) returns true if the track should
            % be confirmed based on the track score.
            
            tf = obj.pScore >= obj.pConfirmationThreshold;
        end
        
        function tf = checkDeletion(obj, varargin)
            %checkDeletion Check if the track should be deleted
            % tf = checkDeletion(obj) returns true if the track should be
            % deleted based on the track score.
            
            tf = (obj.pScore - obj.pMaxScore) < obj.pDeletionThreshold;
        end
        
        function reset(obj)
            %RESET Reset the object state
            % RESET(obj) resets the object state
            obj.pScore      = zeros(1,'like',obj.pScore);
            obj.pMaxScore   = zeros(1,'like',obj.pScore);
            obj.pIsFirstUpdate = true;
        end
        
        function copied = clone(obj)
            %CLONE Create a copy of the current track logic object
            % newObj = clone(obj) returns a copy of obj.
            
            % Construct a new object
            copied = trackScoreLogic(...
                'ConfirmationThreshold',obj.pConfirmationThreshold, ...
                'DeletionThreshold',obj.pDeletionThreshold); 
            
            % Copy the private properties
            copied.pScore                   = obj.pScore;
            copied.pMaxScore                = obj.pMaxScore;
            copied.pClassToUse              = obj.pClassToUse;
            copied.pIsFirstUpdate           = obj.pIsFirstUpdate;
        end
    end
    methods (Static = true)
        %------------------------------------------------------------------
        % loadobj loads the track score logic object
        %------------------------------------------------------------------
        function obj = loadobj(sobj)
            % Assign non-dependent public properties
            obj = trackScoreLogic(...
                'ConfirmationThreshold', sobj.ConfirmationThreshold, ...
                'DeletionThreshold', sobj.DeletionThreshold);
            
            % Load protected/private properties
            loadPrivateProtectedProperties(obj,sobj);
        end
    end
    
    methods (Access = {?matlabshared.tracking.internal.fusion.TrackLogic, ...
            ?matlabshared.tracking.internal.fusion.ObjectTrack, ...
            ?matlabshared.tracking.internal.fusion.TrackManager, ...
            ?matlab.unittest.TestCase})
        function syncState(obj,state)
            %syncState Synchronize the track logic state to given state
            %  syncState(obj,state) synchronizes the track logic state to
            %  the given state and sets the pIsFirstUpdate flag to false
            %  state must be a 2-element real, and finite vector,
            %  [currentScore maxScore], where currentScore <= maxScore.
            
            validateattributes(state, {'single','double'}, ...
                {'finite','real','nondecreasing','vector','numel',2}, ...
                mfilename, 'state')
            obj.pScore = state(1);
            obj.pMaxScore = state(2);
            obj.pIsFirstUpdate = false;
        end
        
        function tf = isInitialized(obj)
            tf = ~obj.pIsFirstUpdate;
        end
    end
    
    % Private parser methods
    methods (Access = private)
        function [confirmation,deletion] = parseInputsMATLAB(obj, varargin)
            p = inputParser;
            p.addParameter('ConfirmationThreshold', obj.constDefaultConfirmation);
            p.addParameter('DeletionThreshold', obj.constDefaultDeletion);
            p.parse(varargin{:});
            confirmation = p.Results.ConfirmationThreshold;
            deletion = p.Results.DeletionThreshold;
        end
        
        function [confirmation,deletion] = parseInputsCodegen(obj, varargin)
            % Define parser
            parms = struct( ...
                'ConfirmationThreshold',    uint32(0), ...                
                'DeletionThreshold',    uint32(0) ...
            );
            
            popt = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', false);
            
            % Parse
            optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});            
            
            % Provide outputs
            confirmation = eml_get_parameter_value(optarg.ConfirmationThreshold, obj.constDefaultConfirmation, varargin{:});            
            deletion     = eml_get_parameter_value(optarg.DeletionThreshold, obj.constDefaultDeletion, varargin{:});
        end

        function setProperties(obj, conf, del)
            % setProperties(obj, confirmation, deletion) sets the
            % ConfirmationThreshold and DeletionThreshold properties, both
            % must be real scalars.
            validateattributes(conf,{'single','double'}, ...
                {'real','finite','positive','scalar'}, mfilename, ...
                'ConfirmationThreshold')
            validateattributes(del,{'single','double'}, ...
                {'real','finite','scalar','<',0}, mfilename, ...
                'DeletionThreshold')
            
            obj.pConfirmationThreshold = cast(conf,obj.pClassToUse);
            obj.pDeletionThreshold = cast(del,obj.pClassToUse);
            
            obj.pScore = cast(0, obj.pClassToUse);
            obj.pMaxScore = cast(0, obj.pClassToUse);
        end
        
        function loadPrivateProtectedProperties(obj,sobj)
            obj.pScore          = sobj.pScore;
            obj.pMaxScore       = sobj.pMaxScore;
            obj.pClassToUse     = sobj.pClassToUse;
            obj.pIsFirstUpdate  = sobj.pIsFirstUpdate;
        end
        
        function sobj = saveobj(obj)
            sobj.ConfirmationThreshold  = obj.ConfirmationThreshold;
            sobj.DeletionThreshold      = obj.DeletionThreshold;
            sobj.pScore                 = obj.pScore;
            sobj.pMaxScore              = obj.pMaxScore;
            sobj.pClassToUse            = obj.pClassToUse;
            sobj.pIsFirstUpdate         = obj.pIsFirstUpdate;
        end
        
        function keepScore(obj, scoreIncrement)
            obj.pScore = obj.pScore + scoreIncrement;
            
            if obj.pScore > obj.pMaxScore
                obj.pMaxScore = obj.pScore;
            end
            obj.pIsFirstUpdate = false;
        end
    end
    
    methods(Static,Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'pClassToUse'};
        end
    end
end
