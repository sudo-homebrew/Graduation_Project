classdef (Sealed, Hidden) trackIntegratedLogic < ...
        matlabshared.tracking.internal.fusion.TrackLogic
%trackIntegratedLogic Confirm and delete tracks based on existence probability
%   logic = trackIntegratedLogic creates a track integrated logic with
%   default confirmation and deletion probability thresholds.
%   A track is confirmed if the current probability of track existence is
%   greater than or equal to the ConfirmationThreshold.
%   A track is deleted if the current probability of track existence is
%   less than or equal to DeletionThreshold.
%
%   The probability of track existence is also used for the detections
%   assignment in the trackerJPDA object.
%
%   ... = trackIntegratedLogic('ConfirmationThreshold',a,'DeletionThreshold',b)
%   allows you to specify the confirmation or deletion threshold. Both a
%   and b must be scalars between 0 and 1.
%   Unspecified properties will have default values.
%
%   Use init to initialize the probability of track existence with the first 'hit'.
%   Use hit to update the probability of track existence with any subsequent 'hit'.
%   Use miss to update the probability of track existence with any subsequent 'miss'.
%   Use propagate to predict the probability of track existence.
%
% trackIntegratedLogic properties:
%   ConfirmationThreshold - threshold for track confirmation
%   DeletionThreshold     - threshold for track deletion
%   ExistenceProbability  - probability of track existence
%
% trackIntegratedLogic methods:
%   init              - initialize the track logic with first 'hit'
%   hit               - update the track logic with a subsequent 'hit'
%   miss              - update the track with a 'miss'
%   propagate         - propagate the probability of track existence
%   sync              - synchronize the trackIntegratedLogic object
%   checkConfirmation - check if the track should be confirmed
%   checkDeletion     - check if the track should be deleted
%   output            - get the current probability of track existence
%   reset             - reset the object state
%   clone             - create a copy of the object
%
% Class support:
% --------------
%   If either ConfirmationThreshold or DeletionThreshold is given on
%   construction as single precision, the other will be converted to single
%   precision and the class will use single precision.
%
% Example:
% --------
% % Create an integrated probability-based track logic
% probLogic = trackIntegratedLogic('ConfirmationThreshold', 0.95, ...
%   'DeletionThreshold', 0.2)
% wasInitialized = false;
% pd = 0.9;    % Probability of detection
% lambda = 1e-5;  % Clutter density
% beta = 1e-2; % New target rate in a unit volume
%
% % Update the logic 3 times. See how the probability increases with a
% % 'hit' and decreases with a 'miss'. By the end of the 3rd update, the
% % confirmation flag will be true.
% for i = 1:3
%     detectedFlag = logical(mod(i,2)); % Only even updates are true
%     if detectedFlag && ~wasInitialized
%         init(probLogic, beta, pd, lambda);
%         wasInitialized = true;
%     elseif detectedFlag && wasInitialized
%         hit(probLogic, 0.999);
%     else
%         miss(probLogic);
%     end
%     confFlag = checkConfirmation(probLogic);
%     disp(['Probability of track existence: ', num2str(output(probLogic)), ...
%       '.  Confirmation Flag is: ',num2str(confFlag)])
% end
%
% % Update the logic with 4 'miss' updates. The deletion flag will turn
% % true by the end of the 4th miss.
% for i = 1:4
%     miss(probLogic);
%     deleteFlag = checkDeletion(probLogic);
%     disp(['Probability of track existence: ', num2str(output(probLogic)), ...
%       '.  Deletion Flag is: ',num2str(deleteFlag)])
% end
%
% See also: trackHistoryLogic, trackScoreLogic

%References:
% [1] Musicki, Darko, and Robin Evans. "Joint integrated probabilistic data
%     association: JIPDA." 2002.

% Copyright 2018-2019 The MathWorks, Inc.

%#codegen
    
    properties(Dependent)
        % ConfirmationThreshold - Threshold for track confirmation
        %   Define the minimum probability of track existence required to
        %   confirm the track. Must be a real scalar in the range (0,1]. If
        %   the probability of track existence is above this threshold, the
        %   track should be confirmed.
        %
        % Default: 0.9
        ConfirmationThreshold
        
        % DeletionThreshold - Threshold for track deletion
        %   Define the maximum probability of track existence required to
        %   delete the track. Must be a real scalar in the range (0,Cf],
        %   where Cf is the confirmation threshold. If the probability of
        %   track existence is lower than the deletion threshold, the track
        %   should be deleted.
        %
        % Default: 0.2
        DeletionThreshold
    end
    
    properties        
        %DeathRate Time Rate of target deaths.
        %   Specify the rate at which true targets disappear. DeathRate is
        %   related to the probability of track existence (PTE) via:
        %      PTE(t+dt) = (1-DeathRate)^dt * PTE(t)
        %   dt is the time interval since the previous update time t.
        %
        %   Default = 0.01;
        DeathRate = 0.01;
    end
    
    properties (GetAccess = public, SetAccess = private, Dependent)
        ExistenceProbability    % The current probability of track existence
    end
    
    properties (Access = private)
        %pConfirmationThreshold The threshold for confirmation
        pConfirmationThreshold
        
        %pDeletionThreshold		The threshold for deletion
        pDeletionThreshold
        
        %pClassToUse			The class used for saving the properties
        pClassToUse
        
        %pExistenceProbability  The current probability of track existence
        pExistenceProbability
        
        %pIsFirstUpdate			True if object was not updated yet
        pIsFirstUpdate = true
        
        %pIsConfirmed           True if ExistenceProbability is above the Confirmation Threshold
        pIsConfirmed = false;
    end
    
    % Default confirmation and deletion values
    properties (Access = {?trackIntegratedLogic, ?matlab.unittest.TestCase},...
            Constant = true)
        constDefaultConfirmation = 0.9;
        constDefaultDeletion = 0.2;
        constDefaultDeathRate = 0.01;
    end
    
    properties (Access = {?matlabshared.tracking.internal.fusion.TrackLogic, ?matlab.unittest.TestCase},...
            Constant = true)
        constPd = 0.9; % Default probability of detection
        constPfa = 1e-6; % Default false alarm density
    end
    
    % Implementing the TrackLogic interface
    methods
        function obj = trackIntegratedLogic(varargin)
            % Constructor to the trackIntegratedLogic object
            if coder.target('MATLAB')
                [confirmation,deletion, death] = parseInputsMATLAB(obj, varargin{:});
            else
                [confirmation,deletion, death] = parseInputsCodegen(obj, varargin{:});
            end
            
            % Set the class to use
            if isa(confirmation,'single') || isa(deletion,'single')
                obj.pClassToUse = 'single';
            else
                obj.pClassToUse = 'double';
            end
            
            setProperties(obj, confirmation, deletion, death);
        end
        
        function init(obj,beta,varargin)
            %INIT Initialize a track integrated logic
            % INIT(obj, beta) initializes the probability of track
            % existence on the first 'hit' using the default values:
            % pd=0.9, lambda=1e-6. beta is the new target density and lambda
            % is the clutter density
            %
            % INIT(obj, beta, pd, lambda) allows you to specify
            % non-default values of pd and lambda.
            
            narginchk(2,4)
            coder.internal.assert(obj.pIsFirstUpdate, 'fusion:trackLogic:alreadyInitialized')
            [pd,lambda] = parsePdPfa(obj,varargin{:});
            betac = cast(beta,obj.pClassToUse);
            pdc = cast(pd,obj.pClassToUse);
            lambdac = cast(lambda,obj.pClassToUse);
            betaPd =betac*pdc;
            obj.pExistenceProbability = (betaPd)/(lambdac+betaPd);
            obj.pIsFirstUpdate = false;
        end
        
        function miss(obj,varargin)
            %MISS Update the track integrated logic with a miss
            % MISS(obj) updates the probability of track existence in a
            % case of a 'miss' using the default values: pd=0.9 
            %
            % MISS(obj, pd) allows you to specify non-default values
            % of pd
            
            narginchk(1,2)
            coder.internal.assert(~obj.pIsFirstUpdate, 'fusion:trackLogic:notInitialized','miss')
            pd = parsePdPfa(obj, varargin{:});
            PTE = obj.pExistenceProbability;
            pdc = cast(pd,obj.pClassToUse);
            obj.pExistenceProbability = PTE*(1-pdc)/(PTE*(1-pdc)+1-PTE);
        end
        
        function hit(obj,posterior,varargin)
            %HIT Update the track integrated logic with a hit
            % HIT(obj,posterior) updates the probability of track existence
            % based on the probability of JPDA association events. If the
            % track is associated with n measurements, posterior is an n+1
            % array such that:
            %   posterior(i)   = p(existence,zi|measurements),  i=1,...,n
            %   posterior(n+1) = p(existence,no_association|measurements)
            % where zi is association to measurement i.
            
            % Reference: [1] equation (9)
            
            narginchk(2,2)
            coder.internal.assert(~obj.pIsFirstUpdate, 'fusion:trackLogic:notInitialized','hit')
            validateattributes(posterior,{'single','double'},{'real','nonsparse','vector','nonnegative'},'hit','posterior');
            posteriorc = cast(posterior,obj.pClassToUse);
            coder.internal.assert(sum(posteriorc)<(1+sqrt(eps(obj.pClassToUse))),'fusion:trackLogic:posteriorSum')
            obj.pExistenceProbability = sum(posteriorc);
        end
        
        function propagate(obj,dt,varargin)
            %PROPAGATE Propagate the probability of track existence
            % PROPAGATE(tracklogic, dt) propagates the probability of track
            % existence over time interval dt using the following equation:
            %   Pte = (1-DeathRate)^dt * Pte 
            % with default DeathRate of 0.01
            % PROPAGATE(tracklogic,dt,deathRate) lets you specify a non
            % default value death rate for the propagation.
            
            narginchk(2,3)
            coder.internal.assert(~obj.pIsFirstUpdate, 'fusion:trackLogic:notInitialized','propagate')
            validateattributes(dt,{'numeric'},...
                {'real','finite','positive','scalar','nonsparse'},mfilename,'dt');
            dtc = cast(dt,obj.pClassToUse);
            if ~isempty(varargin)
                validateattributes(varargin{1},{'numeric'},...
                    {'real','finite','>=',0,'<',1,'scalar','nonsparse'},mfilename,'deathRate');
                dR = cast(varargin{1},obj.pClassToUse);
                pS = (1-dR)^dtc;
            else
                pS = cast((1-obj.DeathRate)^dtc,obj.pClassToUse);
            end
            pte = obj.ExistenceProbability;
            obj.pExistenceProbability = pS*pte;
        end
        
        function set.ConfirmationThreshold(obj, value)
            % set.ConfirmationThreshold(obj, value) sets the confirmation
            % probability threshold
            validateattributes(value,{'single','double'}, ...
                {'real','finite','positive','scalar','<=',1}, mfilename, ...
                'ConfirmationThreshold')
            validateConfDel(obj)
            obj.pConfirmationThreshold = cast(value, obj.pClassToUse);
        end
        
        function value = get.ConfirmationThreshold(obj)
            % value = get.ConfirmationThreshold(obj) returns the
            % probability
            % threshold value used for track confirmation.
            value = obj.pConfirmationThreshold;
        end
        
        function set.DeletionThreshold(obj, value)
            % set.DeletionThreshold(obj, value) sets the deletion
            % probability threshold
            validateattributes(value,{'single','double'}, ...
                {'real','finite','scalar','<=',1}, mfilename, ...
                'DeletionThreshold')
            validateConfDel(obj)
            obj.pDeletionThreshold = cast(value, obj.pClassToUse);
        end
        
        function value = get.DeletionThreshold(obj)
            % value = get.DeletionThreshold(obj) returns the probability
            % threshold value used for track deletion.
            value = obj.pDeletionThreshold;
        end
        
        function validateConfDel(obj)
            coder.internal.assert(obj.DeletionThreshold <= obj.ConfirmationThreshold, ...
                'fusion:trackLogic:confdel','DeletionThreshold','ConfirmationThreshold');
        end
        
        function value = output(obj)
            %OUTPUT Return current probability of track existence
            % pte = output(obj) returns the current probability of track
            % existence, pte.
            value = obj.pExistenceProbability;
        end
        
        function set.ExistenceProbability(obj,value)
            obj.pExistenceProbability = cast(value,obj.pClassToUse);
        end
        
        function value = get.ExistenceProbability(obj)
            value = obj.pExistenceProbability;
        end
        
        function sync(this,that)
            %SYNC Synchronize the probabilities of two objects
            % SYNC(this,that) synchronizes the probability of this object
            % to be the same as the probability of that object
            validateattributes(that,{'fusion.internal.trackIntegratedLogic'},{'scalar'},mfilename,'that');
            this.pExistenceProbability = that.ExistenceProbability;
            this.pIsFirstUpdate = ~isInitialized(that);
        end
        
        function tf = checkConfirmation(obj)
            %checkConfirmation Check if the track should be confirmed
            % tf = checkConfirmation(obj) returns true if the track should
            % be confirmed based on the probability of track existence.
            if obj.pIsFirstUpdate
                tf = false;
            elseif obj.pIsConfirmed
                tf = true;
            else
                tf = obj.pExistenceProbability >= obj.pConfirmationThreshold;
                obj.pIsConfirmed = tf;
            end
        end
        
        function tf = checkDeletion(obj, varargin)
            %checkDeletion Check if the track should be deleted
            % tf = checkDeletion(obj) returns true if the track should be
            % deleted based on the probability of track existence.
            tf = obj.pExistenceProbability < obj.pDeletionThreshold;
        end
        
        function reset(obj)
            %RESET Reset the object state
            % RESET(obj) resets the object state
            obj.pExistenceProbability = zeros(1,'like',obj.pExistenceProbability);
            obj.pIsFirstUpdate = true;
        end
        
        function copied = clone(obj)
            %CLONE Create a copy of the current track logic object
            % newObj = clone(obj) returns a copy of obj.
            
            % Construct a new object
            copied = fusion.internal.trackIntegratedLogic(...
                'ConfirmationThreshold',obj.pConfirmationThreshold, ...
                'DeletionThreshold',obj.pDeletionThreshold,...
                'DeathRate',obj.DeathRate);
            
            % Copy the private properties
            copied.pExistenceProbability  = obj.pExistenceProbability;
            copied.pClassToUse            = obj.pClassToUse;
            copied.pIsFirstUpdate         = obj.pIsFirstUpdate;
            copied.pIsConfirmed           = obj.pIsConfirmed;
        end
    end
    methods (Static = true)
        %------------------------------------------------------------------
        % loadobj loads the track Integrated logic object
        %------------------------------------------------------------------
        function obj = loadobj(sobj)
            % Assign non-dependent public properties
            obj = fusion.internal.trackIntegratedLogic(...
                'ConfirmationThreshold', sobj.ConfirmationThreshold, ...
                'DeletionThreshold', sobj.DeletionThreshold,...
                'DeathRate',sobj.DeathRate);
            
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
            %  state must be a scalar value between 0 and 1
            
            validateattributes(state, {'single','double'}, ...
                {'positive','real','scalar','<',1}, mfilename, 'state')
            obj.ExistenceProbability = cast(state,obj.pClassToUse);
            obj.pIsFirstUpdate = false;
        end
        
        function tf = isInitialized(obj)
            tf = ~obj.pIsFirstUpdate;
        end
    end
    
    % Private parser methods
    methods (Access = private)
        function [confirmation,deletion,death] = parseInputsMATLAB(obj, varargin)
            p = inputParser;
            p.addParameter('ConfirmationThreshold', obj.constDefaultConfirmation);
            p.addParameter('DeletionThreshold', obj.constDefaultDeletion);
            p.addParameter('DeathRate', obj.constDefaultDeathRate);
            p.parse(varargin{:});
            confirmation = p.Results.ConfirmationThreshold;
            deletion = p.Results.DeletionThreshold;
            death = p.Results.DeathRate;
        end
        
        function [confirmation,deletion, death] = parseInputsCodegen(obj, varargin)
            % Define parser
            parms = struct( ...
                'ConfirmationThreshold', uint32(0), ...
                'DeletionThreshold',     uint32(0), ...
                'DeathRate',             uint32(0));
            
            popt = struct( ...
                'CaseSensitivity', false, ...
                'StructExpand',    true, ...
                'PartialMatching', false);
            
            % Parse
            optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});
            
            % Provide outputs
            confirmation = eml_get_parameter_value(optarg.ConfirmationThreshold, obj.constDefaultConfirmation, varargin{:});
            deletion     = eml_get_parameter_value(optarg.DeletionThreshold, obj.constDefaultDeletion, varargin{:});
            death        = eml_get_parameter_value(optarg.DeathRate, obj.constDefaultDeathRate, varargin{:});
        end
        
        function setProperties(obj, conf, del, death)
            % setProperties(obj, confirmation, deletion) sets the
            % ConfirmationThreshold and DeletionThreshold properties, both
            % must be real scalars.
            validateattributes(conf,{'single','double'}, ...
                {'real','finite','positive','scalar','<=',1}, mfilename, ...
                'ConfirmationThreshold')
            validateattributes(del,{'single','double'}, ...
                {'real','finite','scalar','positive','<=',conf}, mfilename, ...
                'DeletionThreshold')
            validateattributes(death,{'single','double'},...
                {'real','finite','scalar','>=',0,'<',1},mfilename,...
                'DeathRate');
            obj.pConfirmationThreshold = cast(conf,obj.pClassToUse);
            obj.pDeletionThreshold = cast(del,obj.pClassToUse);
            obj.DeathRate = cast(death,obj.pClassToUse);
            obj.pExistenceProbability = cast(0,obj.pClassToUse);
        end
        
        function loadPrivateProtectedProperties(obj,sobj)
            obj.pExistenceProbability = sobj.pExistenceProbability;
            obj.pClassToUse	          = sobj.pClassToUse;
            obj.pIsFirstUpdate        = sobj.pIsFirstUpdate;
            obj.pIsConfirmed          = sobj.pIsConfirmed;
        end
        
        function sobj = saveobj(obj)
            sobj.ConfirmationThreshold = obj.ConfirmationThreshold;
            sobj.DeletionThreshold     = obj.DeletionThreshold;
            sobj.DeathRate             = obj.DeathRate;
            sobj.pClassToUse           = obj.pClassToUse;
            sobj.pIsFirstUpdate	       = obj.pIsFirstUpdate;
            sobj.pExistenceProbability = obj.pExistenceProbability;
            sobj.pIsConfirmed          = obj.pIsConfirmed;
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
