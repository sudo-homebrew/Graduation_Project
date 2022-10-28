classdef (SupportExtensionMethods=true, InferiorClasses = {? matlab.graphics.axis.Axes, ? matlab.ui.control.UIAxes}) uss < genss & ulti
   % Uncertain state space models.
   %
   %   Uncertain state-space models (USS) arise when combining ordinary LTI
   %   models (see NUMLTI) with uncertain elements (see UNCERTAINBLOCK). 
   %   USS models keep track of how the uncertain elements interact with the 
   %   fixed dynamics. They can be used for robust stability and worst-case 
   %   performance analysis.
   %
   %   There are three ways to construct a USS object:
   %
   %     1.  Use SS to create uncertain state-space models from uncertain 
   %         matrices A,B,C,D (see UMAT):
   %            p = ureal('p',1);
   %            A = [0 3*p; -p p^2];
   %            B = [0; p];
   %            C = ones(2);  D=zeros(2,1);
   %            usys = ss(A,B,C,D)
   %         The resulting model USYS is USS and depends on the uncertain 
   %         parameter p.
   %
   %     2.  Combine numeric LTI models with uncertain elements:
   %            sys = tf(1, [1 1]);
   %            k = ureal('k',1);
   %            D = ultidyn('Delta',[1 1]);
   %            usys = k*sys*(1+0.1*D)
   %
   %     3.  Convert a double array or numeric LTI model to USS:
   %            M = tf(1,[1 1 1]);
   %            usys = uss(M)
   %         Here, USYS is a USS object with two states and no uncertainty.
   %
   %   Type "methods(uss)" for a list of functions operating on USS models.
   %
   %   See also ureal, ucomplex, ucomplexm, ultidyn, ss, usample.   
   
   %   Author(s): Andy Packard, Gary Balas, P. Gahinet
%   Copyright 1986-2011 The MathWorks, Inc.
   
   % TYPE MANAGEMENT IN BINARY OPERATIONS
   methods (Static, Hidden)
      
      function T = superiorTypes()
         T = {'uss','genss'};
      end
      
      function A = getAttributes(A)
         % Override default attributes
         A.FRD = false;
      end
            
      function T = toFRD(~)
         T = 'ufrd';
      end
      
      function F = toABCD(~)
         F = @umat.make;
      end
      
   end
   
   methods
      
      function sys = uss(D)
         if nargin>0
            % Note: USS() handled by superclass
            if strcmp(class(D),'uss') %#ok<STISA>
               % Handle conversion USS(SYS) where SYS is @uss
               sys = D;
            elseif isnumeric(D)
               % USS(numeric array)
               B = ltipack.LFTBlockWrapper.emptyBlockList();
               D = double(D);
               s = [size(D) ones(1,2)];
               Data = ltipack.lftdataSS.array(s(3:end));
               for ct=1:numel(Data)
                  Data(ct).IC = ltipack.ssdata([],zeros(0,s(2)),zeros(s(1),0),D(:,:,ct),[],0);
                  Data(ct).Blocks = B;
               end
               sys.Data_ = Data;
               sys.IOSize_ = s(1:2);
            else
               ctrlMsgUtils.error('Control:general:InvalidSyntaxForCommand','uss','uss')
            end
         end
      end
      
   end
   
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access=protected)
   
      function displaySize(sys,sizes)
         % Displays SIZE information in SIZE(SYS)
         ny = sizes(1);
         nu = sizes(2);
         nx = order(sys);
         nb = nblocks(sys);
         if length(sizes)==2
            disp(ctrlMsgUtils.message('Robust:umodel:SizeUSS1',ny,nu,nx,nb))
         else
            ArrayDims = sprintf('%dx',sizes(3:end));
            disp(ctrlMsgUtils.message('Robust:umodel:SizeUSS2',ArrayDims(1:end-1)))
            if isempty(nx)
               nx = 0;  nb = 0;
            else
               nx = nx(:);  nb = nb(:);
            end
            if all(nx==nx(1))
               if all(nb==nb(1))
                  disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENSS3',ny,nu,nx(1),nb(1)))
               else
                  disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENSS4',ny,nu,nx(1),min(nb),max(nb)))
               end
            else
               if all(nb==nb(1))
                  disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENSS5',ny,nu,min(nx),max(nx),nb(1)))
               else
                  disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENSS6',ny,nu,min(nx),max(nx),min(nb),max(nb)))
               end
            end
         end
      end
      
   end
   
   %% DATA ABSTRACTION INTERFACE
   methods (Access=protected)
      
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
         [Data(1),B,BlkStruct] = ulftdata(Data(1),varargin{:}); %#ok<*PROPLC>
         for ct=2:numel(Data)
            [Data(ct),Bx] = ulftdata(Data(ct),varargin{:});
            if ~isequal(Bx,B)
               ctrlMsgUtils.error('Robust:umodel:NotSupportedVaryingUncertainty','lftdata')
            end
         end
         % Construct H and DELTA
         H = uss.make(Data);
         H.TimeUnit = M.TimeUnit;
         if any(logicalfun(@(x) isa(x,'DynamicSystem'),B))
            DELTA = uss.make(Data(1).blk2lft(B));
            DELTA.TimeUnit = M.TimeUnit;
         else
            [nw,nz] = iosize(B);
            IC = [zeros(nw,nz) eye(nw);eye(nz) zeros(nz,nw)];
            DELTA = umat.make(ltipack.lftdataM(IC,B));
         end
         BList = getBlockList(B);
      end
      
   end
   
   %% PROTECTED METHODS
   methods (Access = protected)
      
      function s = getPropStruct(sys)
         % Hide "Blocks" property (aliased to "Uncertainty" for @uss)
         s = rmfield(getPropStruct@InputOutputModel(sys),'Blocks');
         n = numel(fieldnames(s));
         s = orderfields(s,[n-1 n 1:n-2]);
      end
      
   end
   
   %% HIDDEN METHODS
   methods (Hidden)
      
      function M = uss2umat(sys)
         % Turns static USS with possible ULTIDYN blocks into UMAT to facilitate
         % reloading of old-style UFRD objects (FOR UFRD.LOADOBJ USE ONLY)
         Dss = sys.Data_;
         D = ltipack.lftdataM.array(size(Dss)); %#ok<*PROP>
         for ct=1:numel(D)
            D(ct).IC = Dss(ct).IC.d;
            D(ct).Blocks = Dss(ct).Blocks;
         end
         M = umat.make(D,sys.IOSize_);
      end
      
   end
   
   %% STATIC METHODS
   methods(Static, Hidden)
      
      sys = loadobj(s)
      
      function sys = make(D,IOSize)
         % Constructs USS model from nonempty ltipack.lftdataSS array.
         % Note: Assumes uncertainty in D is normalized.
         sys = uss;
         sys.Data_ = D;
         if nargin>1
            sys.IOSize_ = IOSize;  % support for empty model arrays
         else
            sys.IOSize_ = iosize(D(1));
         end
      end
      
      function sys = convert(X)
         % Safe conversion to USS
         if isnumeric(X)
            sys = uss(X);
         else
            sys = copyMetaData(X,uss_(X));
         end
      end
      
   end

end


