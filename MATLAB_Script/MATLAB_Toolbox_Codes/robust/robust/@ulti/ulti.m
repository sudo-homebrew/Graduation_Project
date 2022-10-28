classdef ulti < genlti
   % Uncertain LTI Model objects.
   %
   %   Uncertain LTI models arise when connecting ordinary LTI models (see NUMLTI) 
   %   with uncertain elements (see UNCERTAINBLOCK). ULTI models keep track of how 
   %   the uncertainty affect the nominal LTI dynamics. 
   % 
   %   There are two types of uncertain LTI models:
   %     * Uncertain state-space models (USS), which arise when there are no FRD 
   %       models among the ordinary LTI models
   %     * Uncertain FRD models (UFRD), which arise when there is at least one FRD
   %       model in the mix.
   %   All uncertain LTI models derive from the @ulti superclass. This class is   
   %   not user-facing and cannot be instantiated.
   %
   %   See also USS, UFRD, NUMLTI, LTI, ControlDesignBlock.
   
   %   Copyright 1986-2011 The MathWorks, Inc.
   properties (SetAccess=private, Dependent)
      % Nominal value.
      NominalValue
   end
   properties (Dependent)
      % Model uncertainty.
      Uncertainty
   end
   
   methods
            
      function Value = get.NominalValue(sys)
         % GET method for NominalValue property
         Value = getValue_(sys);
      end

      function Value = get.Uncertainty(sys)
         % GET method for Uncertainty property (alias of "Blocks")
         Value = sys.Blocks;
      end
      
      function sys = set.Uncertainty(sys,Value)
         % SET method for Uncertainty property
         try
            sys.Blocks = Value;
         catch ME
           error(ME.identifier,strrep(ME.message,'Blocks','Uncertainty'))
         end
      end
            
   end
      
   %% DATA ABSTRACTION INTERFACE
   methods(Access = protected)
      
      function boo = isParametric_(~)
         boo = false;
      end
      
      function boo = isUncertain_(~)
         boo = true;
      end
      
      % ANALYSIS
      function [SM,INFO,IRD] = robstab_(sys,opt)
         % Robust stability analysis
         Data = sys.Data_; % either lftdataFRD or lftdataSS object
         for ct=1:numel(Data)
            [X,Y,Z] = robstab(Data(ct),opt);
            if ct==1 || X.UpperBound<MinMargin
               SM = X;  INFO = Y;  IRD = Z;
               MinMargin = X.UpperBound;
               INFO.Model = ct;
            end
         end
      end
      
      function [SM,INFO,IRD] = robgain_(sys,gamma,opt)
         % Robust gain analysis
         Data = sys.Data_; % either lftdataFRD or lftdataSS object
         for ct=1:numel(Data)
            [X,Y,Z] = robgain(Data(ct),gamma,opt);
            if ct==1 || X.UpperBound<MinMargin
               SM = X;  INFO = Y;  IRD = Z;
               MinMargin = X.UpperBound;
               INFO.Model = ct;
            end
         end
      end

      function [SM,INFO,IRD] = robuststab_(sys,Opt,sNeed)
         % Robust stability analysis, sys is USS or UFRD
         Data = sys.Data_; % either lftdataFRD or lftdataSS object
         % Pre-allocate output arrays
         AS = size(Data);
         SM = struct('LowerBound',cell(AS),'UpperBound',[],...
            'DestabilizingFrequency',[]);
         INFO = struct('Sensitivity',cell(AS),'Frequency',[],...
            'BadUncertainValues',[],'MussvBnds',[],'MussvInfo',[]);
         IRD = struct('StableFlag',cell(AS),'hasID',[],'SensText',[],...
            'muB',[],'BlockName',[],'DestabilizingValue',[]);
         % Perform analysis
         for ct=1:numel(Data)
            [SM(ct),INFO(ct),IRD(ct)] = robuststab(Data(ct),Opt,sNeed);
         end
      end
      
      function [PM,INFO,IRD] = robustperf_(sys,Opt,sNeed)
         Data = sys.Data_; % either lftdataFRD or lftdataSS object
         AS = size(Data);
         PM = struct('LowerBound',cell(AS),'UpperBound',[],...
            'CriticalFrequency',[]);
         INFO = struct('Sensitivity',cell(AS),'Frequency',[],...
            'BadUncertainValues',[],'MussvBnds',[],'MussvInfo',[]);
         IRD = struct('StableFlag',cell(AS),'hasID',[],'SensText',[],...
            'muB',[],'BlockName',[],'DestabilizingValue',[]);
         for ct=1:numel(Data)
            [PM(ct),INFO(ct),IRD(ct)] = robustperf(Data(ct),Opt,sNeed);
         end
      end
      
      % TRANSFORMATIONS
      function sys = replaceB2B_(sys,BlockNames,BlockValues)
         % Replaces blocks by other blocks.
         sys = replaceB2B_@ltipack.LFTModelArray(sys,BlockNames,BlockValues);
         % Renormalize
         sys = unormalize_(sys);
      end
      
   end
   
   %% PROTECTED METHODS
   methods (Access = protected)
      
      function checkBlockCompatibility(sys,B)
         % All blocks in a ULTI model must be uncertain
         checkBlockCompatibility@genlti(sys,B)
         if ~all(logicalfun(@isUncertain,B))
            ctrlMsgUtils.error('Robust:umodel:NotUncertainBlock',class(sys))
         end
      end
      
      function sys = reload(sys,s)
         % Restore data when loading object
         if isfield(s,'dynamicsys')
            % Pre-MCOS
            sys = reload@DynamicSystem(sys,s.dynamicsys);
         else
            sys = reload@DynamicSystem(sys,s);
         end
      end
      
   end
   
   %% HIDDEN METHODS
   methods (Hidden)
      
      function sys = setBlocks(sys,Value)
         % Modifying uncertain block data
         sys = setBlocks@ltipack.LFTModelArray(sys,Value);
         % Renormalize
         sys = unormalize_(sys);
      end
      
   end

   
end

