classdef (InferiorClasses = {? matlab.graphics.axis.Axes, ? matlab.ui.control.UIAxes}) udyn < UncertainDynamics
%UDYN  Creates uncertain dynamic block.
%
%  UH = UDYN(NAME,IOSIZE) creates an uncertain dynamic system object with 
%  input/output dimension specified by iosize. This object represents 
%  unknown multivariable, time-varying nonlinear dynamics.
%
%  The analysis tools (e.g., ROBSTAB) do not currently handle this type 
%  of uncertain element and @udyn is mostly a placeholder at the moment.
%
%  Example: Create a 2-by-3 udyn element and check its size and properties.
%     N = udyn('N',[2 3])
%     get(N)
%
%  See also ULTIDYN, UREAL, UCOMPLEX, USS, UFRD.
   
%   Author(s): Andy Packard, Gary Balas, P. Gahinet
%   Copyright 1986-2012 The MathWorks, Inc.
           
   % TYPE MANAGEMENT IN BINARY OPERATIONS
   methods (Static, Hidden)
      
      function boo = isClosed(~)
         boo = false;
      end
      
      function T = toClosed(~)
         T = 'uss';
      end
      % Note: getAttributes never called (first converted to USS)      
   end
    
   %% PUBLIC METHODS
   methods
      
      function blk = udyn(name,iosize,varargin)
         ni = nargin;
         if ni==0
            name = 'UNNAMED';  iosize = [1 1];
         elseif ni==1
            iosize = [1 1];
         elseif rem(ni,2)~=0
            ctrlMsgUtils.error('Robust:umodel:udyn1')
         elseif ~(isnumeric(iosize) && isreal(iosize) && ...
               all(iosize>=0 & rem(iosize,1)==0 & isfinite(iosize)))
            ctrlMsgUtils.error('Robust:umodel:udyn2')
         else
            if isscalar(iosize)
               iosize = iosize([1 1]);
            elseif ~isequal(size(iosize),[1 2])
               ctrlMsgUtils.error('Robust:umodel:udyn2')
            end
            iosize = full(double(iosize));
         end
         blk.IOSize_ = iosize;
         blk.Ts_ = 0;
         try
            blk.Name = name;
            % Process additional inputs.
            if ~isempty(varargin)
               blk = set(blk,varargin{:});
            end
         catch ME
            throw(ME)
         end
      end
      
   end
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access=protected)

      % INPUTOUTPUTMODEL
      function displaySize(~,sizes)
         % Display for "size(sys)"
         disp(ctrlMsgUtils.message('Robust:umodel:SizeUDYN',sizes(1),sizes(2)))
      end
      
   end
      
      
   %% HIDDEN INTERFACES
   methods (Hidden)
            
      % CONTROLDESIGNBLOCK
      function Offset = getOffset(blk)
         % Default value is the nominal value
         Offset = zeros(blk.IOSize_);
      end
      
      function str = getDescription(blk,ncopies)
         % Short description for block summary in LFT model display
         nyu = iosize(blk);
         ioSize = sprintf('%dx%d',nyu(1),nyu(2));
         MsgID = 'Robust:umodel:udyn3';
         str = ctrlMsgUtils.message(MsgID,blk.Name,ioSize,ncopies);
      end
   
   end
   
      
   %% UTILITIES
   methods (Hidden)
      
      function [R,S,T] = normalizeBlock(blk)
         % Computes R,S,T such that
         %    * LFT(R,blk-S) is normalized
         %    * blk = LFT(T,LFT(R,blk-S))
         % Returns R=[] if blk-S is already normalized (then blk = LFT(T,blk-S))
         S = getOffset(blk);
         R = [];
         [ny,nu] = size(S);
         T = [S eye(ny);eye(nu) zeros(nu,ny)];
      end
      
      function Aval = norm2act(blk,Nval)
         if ~isa(Nval,'DynamicSystem')
            ctrlMsgUtils.error('Robust:umodel:norm2act5')
         end
         sN = size(Nval);
         if ~isequal(sN(1:2),blk.IOSize_)
            ctrlMsgUtils.error('Robust:umodel:norm2act4',blk.IOSize_(1),blk.IOSize_(2))
         end
         Aval = Nval;
      end
      
      function [Nval,Ndist] = act2norm(blk,Aval)
         % Converts actual block values to normalized block values. The 
         % transformation is 
         %   * N = A/Bound for Type='Gain'
         %   * N = lft(R,A-S) for Type='PositiveReal'
         if ~isa(Aval,'DynamicSystem')
            ctrlMsgUtils.error('Robust:umodel:act2norm5')
         end
         sA = size(Aval);
         if ~isequal(sA(1:2),blk.IOSize_)
            ctrlMsgUtils.error('Robust:umodel:act2norm4',blk.IOSize_(1),blk.IOSize_(2))
         end
         Nval = Aval;
         if nargout>1
            Ndist = zeros([sA(3:end) 1 1]);
            for ct=1:numel(Ndist)
               Ndist(ct) = norm(Nval(:,:,ct),inf);
            end
         end
      end
      
      function SampleArray = sample(varargin) %#ok<STOUT>
         % Not supported.
         ctrlMsgUtils.error('Robust:umodel:udynsample')
      end

      function blkN = getNormalizedForm(blk)
         % Returns normalized version of block (see LFTDATA)         
         blkN = udyn(sprintf('%sNormalized',getName(blk)),iosize(blk));
         blkN.Ts_ = blk.Ts_;
      end
      
   end
       
   %% STATIC METHODS
   methods(Static, Hidden)
      
      blk = loadobj(s)
      
   end
   
end
