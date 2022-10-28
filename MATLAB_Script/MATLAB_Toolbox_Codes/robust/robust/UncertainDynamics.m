classdef UncertainDynamics < UncertainBlock & DynamicBlock
   % Control Design blocks for modeling uncertain dynamics
   
%   Copyright 2010-2012 The MathWorks, Inc.
   
   properties (GetAccess = public, SetAccess = private, Dependent)
      % Nominal value.
      NominalValue
   end
      
   %% PUBLIC METHODS
   methods

      function Value = get.NominalValue(blk)
         % GET method for NominalValue property (returns SS model).
         Value = copyMetaData(blk,ss_(blk));
      end

   end
      
   %% PROTECTED INTERFACES
   methods (Access = protected)
      
      % INDEXING SUPPORT
      function M = subparen(blk,indices)
         % Indexing forces conversion to GENSS
         M = subparen(uss(blk),indices);
      end
      
   end
   
   %% DATA ABSTRACTION INTERFACE
   methods (Access=protected)
            
      %% MODEL CHARACTERISTICS
      % Note: ULTIDYN/UDYN blocks are always marked "static" so that it can be
      % used in continuous or discrete time, and "stable" based on their nominal 
      % value      
      function ns = order_(~)
         % Get number of states in nominal value.
         ns = 0;
      end
      
      function [a,b,c,d,Ts] = ssdata_(blk,varargin)
         % Quick access to explicit state-space data
         if ~isequal(1,1,varargin{:})
            ctrlMsgUtils.error('Control:ltiobject:access2')
         end
         M = getNominalMatrix_(blk);
         [ny,nu] = size(M);
         a = [];  b = zeros(0,nu);  c = zeros(ny,0);  d = M;  Ts = blk.Ts_;
      end
      
      %% INDEXING
      function M = createLHS(~)
         % Creates LHS in assignment. Returns 0x0 USS
         M = uss();
      end      
   
      %% ANALYSIS
      function p = pole_(varargin)
         p = zeros(0,1);
      end
            
      function fresp = evalfr_(blk,s)
         fresp = evalfr(ltipack_ssdata(blk),s);
      end

      function varargout = timeresp_(blk,varargin)
         [varargout{1:nargout}] = timeresp_(ss_(blk),varargin{:});
      end
      
      function varargout = lsim_(blk,varargin)
         [varargout{1:nargout}] = lsim_(ss_(blk),varargin{:});
      end

      %% TRANSFORMATIONS
      function sys = getValue_(blk)
         % Returns nominal value
         sys = blk.NominalValue;
      end

      function blk = chgTimeUnit_(blk,newUnits)
         % Rescale time vector: tnew = sf * told
         if blk.Ts_>0
            blk.Ts_ = tunitconv(blk.TimeUnit,newUnits) * blk.Ts_;
         end
         blk.TimeUnit = newUnits; % direct set
      end
      
      function blk = transpose_(~) %#ok<STOUT>
         ctrlMsgUtils.error('Control:transformation:Transpose1') 
      end
      
      function blk = ctranspose_(~) %#ok<STOUT>
         ctrlMsgUtils.error('Control:transformation:Transpose1') 
      end
      
      function sys = uminus_(blk)
         sys = uminus_(uss(blk));
      end
            
      function M = repmat_(blk,s)
         M = repmat_(uss(blk),s);
      end
      
      function  blk = setValue_(blk,~) %#ok<INUSD>
         error(message('Robust:umodel:setBlockValue1'))
      end
      
   end
   
   
   %% PROTECTED METHODS
   methods (Access = protected)
      
      function M = getNominalMatrix_(blk,R,S)
         % Gets nominal value in matrix form
         M = getOffset(blk);
         if nargin>1
            M = M-S;
            if ~isempty(R)
               M = lft(R,M);
            end
         end
      end
            
   end
   
end
