classdef umat < genmat
   % Uncertain matrices.
   %
   %   Uncertain matrices (UMAT) are rational expressions involving uncertain 
   %   parameters of type UREAL, UCOMPLEX, or UCOMPLEXM. They can be used for 
   %   worst-case gain analysis and are useful for building uncertain state-space 
   %   models (see USS).
   %
   %   There are two ways to create UMAT objects.
   %     1.  Create uncertain elements and combine them:
   %            p = ureal('p',1);
   %            M = [0 p; 1 p^2]
   %         M is a 2-by-2 uncertain matrix (UMAT object).
   %
   %     2.  Convert double arrays to UMAT type:
   %            M = umat(1)
   %         Here M is a 1-by-1 matrix (UMAT object) with no uncertainty.
   %
   %   Type "methods(umat)" for a list of functions operating on UMATs.
   %
   %   See also ureal, ucomplex, ucomplexm, uss, usample, usubs.
   
   %   Author(s): Andy Packard, Gary Balas, P. Gahinet
%   Copyright 1986-2011 The MathWorks, Inc.
   
   properties (SetAccess=private,Dependent)
      % Nominal value.
      NominalValue
   end
   properties (Dependent)
      % Model uncertainty.
      Uncertainty
   end
   
   % TYPE MANAGEMENT IN BINARY OPERATIONS
   methods (Static, Hidden)
      
      function T = superiorTypes()
         T = {'umat','genmat'};
      end
            
      function A = getAttributes(A)
         % Override default attributes
         A.System = false;
         A.FRD = false;
      end
      
      function T = toSystem(~)
         T = 'uss';
      end
      
      function T = toFRD(~)
         T = 'ufrd';
      end
      
   end
   
   methods
      
      function M = umat(D)
         if nargin==0
            % UMAT()
            M.Data_ = ltipack.lftdataM([],ltipack.LFTBlockWrapper.emptyBlockList());
            M.IOSize_ = [0 0];
         elseif strcmp(class(D),'umat') %#ok<STISA>
            % Handle conversion UMAT(M) where M is @umat
            M = D;
         elseif isnumeric(D)
            % UMAT(double)
            B = ltipack.LFTBlockWrapper.emptyBlockList();
            D = double(full(D));
            s = [size(D) ones(1,2)];
            Data = ltipack.lftdataM.array(s(3:end));
            for ct=1:numel(Data)
               Data(ct).IC = D(:,:,ct);
               Data(ct).Blocks = B;
            end
            M.Data_ = Data;
            M.IOSize_ = s(1:2);
         else
            ctrlMsgUtils.error('Control:general:InvalidSyntaxForCommand','umat','umat')
         end
      end
            
      function Value = get.NominalValue(M)
         % GET method for NominalValue property
         Value = double(M);
      end

      function Value = get.Uncertainty(M)
         % GET method for Uncertainty property (alias of "Blocks")
         Value = M.Blocks;
      end
      
      function M = set.NominalValue(M,~) %#ok<*MANU>
         % Read-only for USS
         ctrlMsgUtils.error('Robust:umodel:umat2')
      end
      
      function M = set.Uncertainty(M,Value)
         % SET method for Uncertainty property
         try
            M.Blocks = Value;
         catch ME
           error(ME.identifier,strrep(ME.message,'Blocks','Uncertainty'))
         end
      end
      
   end
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access = protected)
      
      % INPUTOUTPUTMODEL
      function displaySize(M,sizes)
         % Displays SIZE information in SIZE(SYS)
         ny = sizes(1);
         nu = sizes(2);
         nb = nblocks(M);
         if length(sizes)==2,
            disp(ctrlMsgUtils.message('Robust:umodel:SizeUMAT1',ny,nu,nb))
         else
            ArrayDims = sprintf('%dx',sizes(3:end));
            disp(ctrlMsgUtils.message('Robust:umodel:SizeUMAT2',ArrayDims(1:end-1)))
            if isempty(nb)
               nb = 0;
            else
               nb = nb(:);
            end
            if all(nb==nb(1))
               disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENMAT3',ny,nu,nb(1)))
            else
               disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENMAT4',ny,nu,min(nb),max(nb)))
            end
         end
      end
      
   end
   
   
   %% DATA ABSTRACTION INTERFACE
   methods (Access=protected)
   
      % MODEL CHARACTERISTICS
      function boo = isParametric_(~)
         boo = false;
      end
      function boo = isUncertain_(~)
         boo = true;
      end
      
      % TRANSFORMATIONS
      function [H,DELTA,BlkStruct,BList] = lftdata_(M,varargin)
         % LFTDATA support for LFT models. Decomposes M as
         %     M = LFT(DELTA,H)
         % where DELTA includes all blocks listed in BLOCKNAMES and H includes 
         % the remaining blocks. Non-uncertain blocks are absorbed into the IC 
         % model and all uncertain blocks in DELTA are normalized and sorted 
         % by name. If M is a model array, the blocks listed in BLOCKNAMES must 
         % appear in each model with the same multiplicity (so that BLKSTRUCT
         % and BLIST are the same for all models).
         Data = M.Data_;
         [Data(1),B,BlkStruct] = ulftdata(Data(1),varargin{:});
         for ct=2:numel(Data)
            [Data(ct),Bx] = ulftdata(Data(ct),varargin{:});
            if ~isequal(Bx,B)
               ctrlMsgUtils.error('Robust:umodel:NotSupportedVaryingUncertainty','lftdata')
            end
         end
         % Construct H and DELTA
         H = umat.make(Data);
         [nw,nz] = iosize(B);
         DELTA = umat.make(ltipack.lftdataM(...
            [zeros(nw,nz) eye(nw);eye(nz) zeros(nz,nw)],B));
         BList = getBlockList(B);
      end
   
      function M = replaceB2B_(M,BlockNames,BlockValues)
         % Replaces blocks by other blocks.
         M = replaceB2B_@ltipack.LFTModelArray(M,BlockNames,BlockValues);
         % Renormalize
         M = unormalize_(M);
      end
      
   end
   
   
   %% PROTECTED METHODS
   methods (Access = protected)
      
      function s = getPropStruct(M)
         % Hide "Blocks" property (aliased to "Uncertainty" for @umat)
         s = rmfield(getPropStruct@InputOutputModel(M),'Blocks');
      end
      
      function checkBlockCompatibility(~,B)
         % All blocks in a UMAT must be static and uncertain
         if ~all(logicalfun(@(x) isUncertain(x) && isa(x,'StaticModel'),B))
            ctrlMsgUtils.error('Robust:umodel:umat3')
         end
      end
      
   end
   
   
   %% HIDDEN METHODS
   methods (Hidden)

      function M = setBlocks(M,Value)
         % Modifying uncertain block data
         M = setBlocks@ltipack.LFTModelArray(M,Value);
         % Renormalize
         M = unormalize_(M);
      end
      
   end
   
   %% STATIC METHODS
   methods(Static, Hidden)
      
      blk = loadobj(s)
      
      function M = make(D,IOSize)
         % Constructs UMAT model from nonempty ltipack.lftdataM array
         % Note: Assumes uncertainty in D is normalized.
         M = umat;
         M.Data_ = D;
         if nargin>1
            M.IOSize_ = IOSize;  % support for empty model arrays
         else
            M.IOSize_ = iosize(D(1));
         end
      end
      
      function M = convert(X)
         % Safe conversion to UMAT
         if isnumeric(X)
            M = umat(X);
         else
            M = copyMetaData(X,umat_(X));
         end
      end
      
   end
   
end
