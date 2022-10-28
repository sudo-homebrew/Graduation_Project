classdef (InferiorClasses = {? matlab.graphics.axis.Axes, ? matlab.ui.control.UIAxes}) ufrd < ulti & genfrd
   % Uncertain FRD models.
   % 
   %   Uncertain Frequency Response Data models (UFRD) arise when combining 
   %   ordinary FRD models (see FRD) with uncertain elements (see 
   %   UNCERTAINBLOCK). UFRD models keep track of how the uncertain elements 
   %   affect the frequency response. They can be used for robust stability 
   %   and worst-case performance analysis.
   %
   %   There are three ways to construct a UFRD object:
   %
   %     1.  Combine FRD models with uncertain elements:
   %            sys = frd(rand(100,1),logspace(-2,2,100));
   %            k = ureal('k',1);
   %            D = ultidyn('Delta',[1 1]);
   %            usys = k*sys*(1+0.1*D)
   %
   %     2.  Convert a USS model to UFRD:
   %            a = ureal('a',1,'range',[0.5 1.8]);
   %            usys = tf(a,[1 a]);
   %            freqs = logspace(-2,2,100);
   %            usys = ufrd(usys,freqs)
   %         See InputOutputModel/ufrd for details on the conversion syntax.
   %
   %     3.  Use FRD to create a UFRD model from uncertain response data
   %         (see UMAT):
   %            RespData = rand(1,1,100) * ureal('delta',1,'percent',50);
   %            usys = frd(RespData,logspace(-2,2,100),0.1)
   %
   %   Type "methods(ufrd)" for a list of functions operating on UFRD models.
   %
   %   See also ureal, ucomplex, ucomplexm, ultidyn, frd, usample.   
   
   %   Author(s): Andy Packard, Gary Balas, P. Gahinet
   %   Copyright 1986-2011 The MathWorks, Inc.
   
   properties (SetAccess=private, Dependent)
      % Uncertain frequency response
      ResponseData
   end
   % OBSOLETE PROPERTIES
   properties (Access = public, Dependent, Hidden)
      % Renamed to FrequencyUnit
      Units
   end
   
   % TYPE MANAGEMENT IN BINARY OPERATIONS
   methods (Static, Hidden)
      
      function T = superiorTypes()
         T = {'ufrd','genfrd'};
      end
      
      function A = getAttributes(A)
         % Override default attributes
      end
      
   end
   
   methods
      
      function sys = ufrd(varargin)
         ni = nargin;
         if ni>0
            if strcmp(class(varargin{1}),'ufrd') %#ok<STISA>
               % UFRD(SYS,FREQ,UNIT) where SYS is @ufrd
               if ni==1
                  % no-op
                  sys = varargin{1};
               else
                  try
                     [sys,freq,funit] = FRDModel.parseFRDInputs('ufrd',varargin);
                     freq = funitconv(funit,'rad/TimeUnit',sys.TimeUnit)*freq;
                     % Note: May error if FREQ differs from frequency grid for some models
                     sys = copyMetaData(sys,ufrd_(sys,freq));
                     % Restore frequency units
                     sys = chgFreqUnit(sys,funit);
                  catch ME
                     throw(ME)
                  end
               end
            else
               ctrlMsgUtils.error('Control:general:InvalidSyntaxForCommand','ufrd','ufrd')
            end
         end
      end
            
      function Value = get.ResponseData(sys)
         % GET method for ResponseData property
         Value = localGetResponse(sys.Data_,sys.IOSize_);
      end
            
      function Value = get.Units(sys)
         Value = sys.FrequencyUnit;
      end      
      function sys = set.Units(sys,Value)
         sys.FrequencyUnit = Value;
      end
   end
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access=protected)

      function displaySize(sys,sizes)
         % Displays SIZE information in SIZE(SYS)
         ny = sizes(1);
         nu = sizes(2);
         nf = nfreqs(sys);
         nb = nblocks(sys);
         if length(sizes)==2,
            disp(ctrlMsgUtils.message('Robust:umodel:SizeUFRD1',ny,nu,nf,nb))
         else
            ArrayDims = sprintf('%dx',sizes(3:end));
            disp(ctrlMsgUtils.message('Robust:umodel:SizeUFRD2',ArrayDims(1:end-1)))
            if isempty(nb)
               nb = 0;
            else
               nb = nb(:);
            end
            if all(nb==nb(1))
               disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENFRD3',ny,nu,nf,nb(1)))
            else
               disp(ctrlMsgUtils.message('Control:lftmodel:SizeGENFRD4',ny,nu,nf,min(nb),max(nb)))
            end
         end
      end
      
   end
   
   %% DATA ABSTRACTION INTERFACE
   methods (Access=protected)
      
      function [H,DELTA,BlkStruct,BList] = lftdata_(M,varargin)
         % LFTDATA support for LFT models. Decomposes M as M = LFT(DELTA,H).
         Data = M.Data_;
         [Data(1),B,BlkStruct] = ulftdata(Data(1),varargin{:});
         for ct=2:numel(Data)
            [Data(ct),Bx] = ulftdata(Data(ct),varargin{:});
            if ~isequal(Bx,B)
               ctrlMsgUtils.error('Robust:umodel:NotSupportedVaryingUncertainty','lftdata')
            end
         end
         % Construct H and DELTA
         H = ufrd.make(Data);
         H.TimeUnit = M.TimeUnit;
         H = chgFreqUnit(H,M.FrequencyUnit);
         [nw,nz] = iosize(B);
         IC = [zeros(nw,nz) eye(nw);eye(nz) zeros(nz,nw)];
         if any(logicalfun(@(x) isa(x,'DynamicSystem'),B))
            ICSS = ltipack.ssdata([],zeros(0,nw+nz),zeros(nw+nz,0),IC,[],Data(1).IC.Ts);
            DELTA = uss.make(ltipack.lftdataSS(ICSS,B));
            DELTA.TimeUnit = M.TimeUnit;
         else
            DELTA = umat.make(ltipack.lftdataM(IC,B));
         end
         BList = getBlockList(B);
      end
      
   end
   
   
   %% PROTECTED METHODS
   methods (Access = protected)

      function s = getPropStruct(sys)
         % Hide "Blocks" property (aliased to "Uncertainty" for @ufrd)
         s = rmfield(getPropStruct@genfrd(sys),'Blocks');
      end
      
   end

   
   %% STATIC METHODS
   methods(Static, Hidden)
      
      sys = loadobj(s)
      
      function sys = make(D,IOSize)
         % Constructs UFRD model from nonempty ltipack.lftdataFRD array
         % Note: Assumes uncertainty in D is normalized.
         sys = ufrd;
         sys.Data_ = D;
         if nargin>1
            sys.IOSize_ = IOSize;  % support for empty model arrays
         else
            sys.IOSize_ = iosize(D(1));
         end
      end
      
      function sys = convert(X,refsys)
         % Safe conversion to UFRD.
         %   X = ufrd.convert(X,REFSYS) casts the variable X to UFRD using the  
         %   frequency vector and frequency units of REFSYS. This method is used 
         %   in assignments, conversions to UFRD, and binary operations to
         %   correctly handle numeric arrays, static models, and undefined 
         %   sampling times.
         f = refsys.Frequency;
         Ts = refsys.Ts;
         if isnumeric(X)
            % Casting numeric array to UFRD
            s = size(X);
            X = repmat(reshape(double(X),[s(1:2) 1 s(3:end)]),[1 1 length(f)]);
            sys = ufrd(frd(X,f,Ts));
            sys.TimeUnit = refsys.TimeUnit;
            sys.FrequencyUnit = refsys.FrequencyUnit;
         elseif isa(X,'StaticModel')
            % Casting static model to UFRD. Make sure to propagate time units
            sys = ufrd(X,f,refsys.FrequencyUnit,refsys.TimeUnit);
         elseif isa(X,'ltipack.SingleRateSystem')
            % Casting static or dynamic model to UFRD
            if X.Ts==-1 && (Ts>0 || isstatic(X))
               % Override Ts=-1 before computing frequency response
               X.Ts = Ts;
            end
            sys = ufrd(X,f,refsys.FrequencyUnit);
         end
      end
      
   end

end


%---------------------------------------
function R = localGetResponse(Data,DefaultSize)
% Retrieves response data as UMAT
% Note: Somewhat inconsistent with USS behavior where matrices
% without uncertainty are shown as double arrays, but needed for 
% backward compatibility
ArraySize = size(Data);
if isempty(Data) || isempty(Data(1).IC.Frequency)
   % Empty array
   R = umat(zeros([DefaultSize,0,ArraySize]));
else
   % Allocate data array
   nf = numel(Data(1).IC.Frequency);
   SData = ltipack.lftdataM.array([nf ArraySize]);
   nblk = 0;
   for ct=1:numel(Data)
      D = getResponse(Data(ct));  % nf-by-1 lftdataM vector 
      nblk = max(nblk,numel(D(1).Blocks));
      SData(:,ct) = D;
   end
   R = umat.make(SData);
end
end
