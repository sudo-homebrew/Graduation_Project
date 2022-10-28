function [Pu,AllWData] = ucover(varargin)
%UCOVER  Fits an uncertain model to a set of LTI responses.
%
% USYS = UCOVER(PARRAY,PNOM,ORD1,ORD2,UTYPE) returns an uncertain 
% system USYS with nominal value PNOM and whose range of behaviors 
% includes all LTI responses in the LTI array PARRAY. PNOM and PARRAY 
% can be SS, TF, ZPK, or FRD models. USYS is of class UFRD if PNOM
% is an FRD model and of class USS otherwise.
%
% You can use any of the following uncertainty models:
%    Input Multiplicative:     USYS = PNOM*(I + W1*ULTIDYN*W2)
%    Output Multiplicative:    USYS = (I + W1*ULTIDYN*W2)*PNOM
%    Additive:                 USYS = PNOM + W1*ULTIDYN*W2 
% where ULTIDYN is a unit-norm uncertain dynamics block and W1 and W2  
% are diagonal, stable, minimum-phase shaping filters that adjust the 
% amount of uncertainty at each frequency. Set UTYPE to 'InputMult' 
% (default), 'OutputMult', or 'Additive' to select the desired model.
%
% ORD1 and ORD2 specify the order (number of states) of each diagonal  
% entry of W1 and W2. If PNOM has NU inputs and NY outputs, ORD1 and ORD2
% should be vectors of length:
%         UTYPE           ORD1          ORD2
%       InputMult        NU-by-1       NU-by-1
%       OutputMult       NY-by-1       NY-by-1
%        Additive        NY-by-1       NU-by-1
% In addition, UCOVER supports the following shortcuts for ORD1 and ORD2:
%    * ORD1=[] means W1=1
%    * A scalar value for ORD1 makes W1 scalar valued.
%
% USYS = UCOVER(PARRAY,PNOM,ORD) uses the simplified uncertainty model 
%    USYS = PNOM*(I + W*ULTIDYN)
% where W is a scalar-valued filter of order ORD. This corresponds to 
% ORD1=ORD and ORD2=[] (W1=W and W2=1).
%
% [USYS,INFO] = UCOVER(PARRAY,...) also returns a structure INFO with
% optimal filter values over a frequency grid. To reuse this information
% and quickly try different orders for W1 and W2, use the syntax:
%    [USYSnew,INFOnew] = UCOVER(PNOM,INFO,ORD1new,ORD2new)
%
% The computation of W1 and W2 proceeds in two steps: 
%    1. Optimal values of W1 and W2 are computed on a frequency grid
%    2. These values are fit by dynamic filters of the specified orders.
% When using multiplicative uncertainty for MIMO systems, the degree to 
% which USYS can cover PARRAY is limited because multiplicative uncertainty 
% acts only on the row (output) or column (input) space of PNOM. To gauge 
% how large the residuals (unexplained portion of PARRAY-PNOM) are, compare
% the gains of PNOM and INFO.Residual (an FRD array commensurate with PARRAY) 
% using 
%    sigma(PNOM,INFO.Residual)
%
% See also FRD/FITMAGFRD, USS, UFRD, ULTIDYN.

%   Copyright 2003-2013 The MathWorks, Inc.
nin = nargin;
if nin<2 || nin>5
   error('RCT:ucover:IncorrectNumberInputArg','UCOVER requires between 2 and 5 input arguments.')
end
% Fill in missing arguments
[varargin{:}] = convertStringsToChars(varargin{:});
Defaults = {1,[],'InputMultiplicative'};
if ischar(varargin{nin})
   % User-defined UTYPE (always last argument)
   varargin = [varargin(1:nin-1) Defaults(nin-2:2) varargin(nin)];
else
   varargin = [varargin , Defaults(nin-1:3)];
end

% XXX--Make a ucoverset function to create the options structure.  The help
% for ucoverset can describe the use of R1 and R2.
Types = {'Additive';'InputMultiplicative';'OutputMultiplicative'};
data2fit = [];
InteractiveFit = 0;  % Interactive fitting for future use
% [USYS,INFO] = ucover(Parray,Pnom)  IM with diagonal 1st order on each side
% [USYS,INFO] = ucover(Parray,Pnom,O1,O2,Type)
% [USYS,INFO] = ucover(Parray,Pnom,O1,O2,OptionStruct)
% [USYS,newINFO] = ucover(INFO,O1,O2)  % really calls ROBUST/UCOVER, which
%     then calls ucover(Pnom,INFO,O1,O2) to get the @lti dispatch

% Two main syntax:
% [USYS,INFO] = ucover(Parray,Pnom{,O1,O2,Type/OptionStruct})
% [USYS,INFO] = ucover(Pnom,INFO,O1{,O2})
InfoInFlag = (isstruct(varargin{2}));  % true if INFO struct supplied
if InfoInFlag
   % [USYS,INFO] = ucover(PNOM,INFO,O1{,O2})
   Pnom = varargin{1};
   AllWDatain = varargin{2};
   w1Order = varargin{3};
   w2Order = varargin{4};
   DeltaName = AllWDatain.DeltaName;
   type = AllWDatain.Type;
   Residual = AllWDatain.Residual;
   szPaOrig = [size(Residual) 1 1];
   nY = szPaOrig(1);
   nU = szPaOrig(2);
   [~,Freqs] = frdata(Residual);
   [OIFlagW1,OIFlagW2,OCFlag,OFitFlag,OScalarFlag] = LOCALRetrieveOriginalFlags(AllWDatain);
   if ~isa(Pnom,'FRDModel')
       Pnomg = frd(Pnom,Freqs);
   else
       Pnomg = frd(Pnom);
   end
   Pnomg = delay2z(Pnomg);
   if OScalarFlag
      data2fit = AllWDatain.W1opt*AllWDatain.W2opt;
      Parrayg = [];
      omega = [];
      nFreq = [];
      nModels = [];
   end
   [type,~,nW1,nW2] = LOCALgetType(Types,type,nY,nU);
   [~,~,ScalarFlag,CFlag,IFlagW1,IFlagW2,FitFlag] = ...
       LOCALMakeFlags(w1Order,w2Order,nW1,nW2);
   if ~isequal(OScalarFlag,ScalarFlag)
       error('RCT:ucover:InvalidReuse','Order Specification is inconsistent with prior optimization.');
   end
   if ~isequal(OCFlag,CFlag)
       error('RCT:ucover:InvalidReuse','Order Specification is inconsistent with prior optimization.');
   end
   if ~ScalarFlag
       if ~isequal(OIFlagW1,IFlagW1)
           error('RCT:ucover:InvalidReuse','ORD1 specification is inconsistent with prior optimization.');
       end
       if ~isequal(OIFlagW2,IFlagW2)
           error('RCT:ucover:InvalidReuse','ORD2 specification is inconsistent with prior optimization.');
       end
       if ~isequal(OFitFlag,FitFlag)
           error('RCT:ucover:InvalidReuse','Order specification is inconsistent with prior optimization.');
       end
   end
else
   % [USYS,INFO] = ucover(PARRAY,PNOM{,O1,O2,Type/OptionStruct})
   % NOTE: USYS is either USS or UFRD based on PNOM's type 
   Parray = varargin{1};
   Pnom = varargin{2};
   if ~(isa(Parray,'DynamicSystem') && isa(Pnom,'DynamicSystem'))
      error('RCT:ucover:NeedLTIObjects','PARRAY and PNOM should be LTI objects (ss, frd, zpk, tf).');
   end
   w1Order = varargin{3};
   w2Order = varargin{4};
   if isa(varargin{5},'struct')
      opts = varargin{5};
      type = opts.Type;
      R1 = opts.R1;
      R2 = opts.R2;      
   elseif isa(varargin{5},'char')
      type = varargin{5};
      R1 = [];
      R2 = [];
   else
      % For now, documented ARG5 must be UTYPE
      error('RCT:ucover:Arg5Problem','Fifth input argument must be UTYPE (char)');      
   end   
   if isempty(type)
      type = 'InputMult';
   end
end
TS = getTs(Pnom);

% Based on Pnom and Parray, first determine
%     nY, nU, nFreq, nModels, Pnomg, Parrayg, omega
szP = size(Pnom);
nY = szP(1); 
nU = szP(2);
if ~InfoInFlag
   szPaOrig = [size(Parray) 1];  % need to reshape Residual at end
   % Collapse Parray to a single array dimension
   Parray = reshape(Parray,[prod(szPaOrig(3:end)) 1]);
   szPa = [size(Parray) 1];
   if szP(1)==szPa(1) && szP(2)==szPa(2) && length(szP)==2
      nModels = szPa(3);
      if isa(Pnom,'FRDModel')
         omega = get(Pnom,'Frequency');
      elseif isa(Parray,'FRDModel')
         omega = get(Parray,'Frequency');
      else
         % Automatic frequency selection can have problems.  We have issues
         % when Pnom and/or Parray have pure integrators, as this tends to
         % confuse the automatic frequency grid selectors.  All
         % strategies are consolidated in LOCALpickw
         Pnom = ss(Pnom);
         Parray = ss(Parray);
         omega = LOCALpickw(Pnom,Parray);
      end
      nFreq = length(omega);
      if isa(Pnom,'FRDModel')
         Pnomg = frd(Pnom);
      else
         Pnomg = frd(Pnom,omega);
      end
      Pnomg = absorbDelay(Pnomg);
      if isa(Parray,'FRDModel')
         Parrayg = frd(Parray);
      else
         Parrayg = frd(Parray,omega);
      end
      Parrayg = absorbDelay(Parrayg);
      try
         % Check compatibility of Ts, TimeUnit, Frequency,...
         [Parrayg,Pnomg] = matchAttributes(Parrayg,Pnomg);
      catch ME
         throw(ME)
      end
   else
      error('RCT:ucover:DimensionMismatch','P and PARRAY should have same input/output dimensions.');
   end
end

[type,dsuf,nW1,nW2,Emsg] = LOCALgetType(Types,type,nY,nU);
if ~isempty(Emsg)
    error(Emsg{:});
end

[Flag1,Flag2,ScalarFlag,CFlag,IFlagW1,IFlagW2,FitFlag,Emsg] = ...
    LOCALMakeFlags(w1Order,w2Order,nW1,nW2);
if ~isempty(Emsg)
    error(Emsg{:});
end

if ScalarFlag==0 &&  InfoInFlag==0
   [R,R1,R2] = LOCALMakeR(IFlagW1,R1,nW2,IFlagW2,nW1,R2);
end
   
if InfoInFlag==0
   if isempty(inputname(1))
      DeltaName = [dsuf(2:end) 'Delta' ];
   else
      DeltaName = [inputname(1) dsuf 'Delta' ];
   end
end

if ScalarFlag==1
   if Flag1==3 && Flag2==3
      scalarorder = w1Order+w2Order;
   elseif Flag1==3
      scalarorder = w1Order;
   else
      scalarorder = w2Order;
   end      
   [Pu,~,AllWData] = LOCALScalar(Pnom,Pnomg,Parrayg,omega,scalarorder,...
      DeltaName,nY,nU,nFreq,nModels,type,InfoInFlag,data2fit);
else
   if CFlag==0
      if InfoInFlag==0
         ResidData = zeros(nY,nU,nFreq,nModels);
         RPnomg = Pnomg.ResponseData;  %nY-nU-NFreq
         % XXX: line below acts as though we need to collapse all array
         % dimensions into the 1st array dimension, but Parrayg only having
         % one array dimension is already enforced earlier...
         RParrayg = Parrayg.ResponseData(:,:,:,:);  % nY-nU-NFreq-PE
         w1rowdata = zeros([1 nW1 nFreq]);
         w2rowdata = zeros([1 nW2 nFreq]);
         switch type
            case {'InputMult'}
               for i=1:nFreq
                  if IFlagW1==0 && IFlagW2==0
                     [W1mat,W2mat,~,tResidDataMat] = ...
                        LOCALInputMult12(RPnomg(:,:,i),RParrayg(:,:,i,:),R1,R2);
                  elseif IFlagW1==0 && IFlagW2==1
                     [W1mat,~,tResidDataMat] = ...
                        LOCALInputMultW1(RPnomg(:,:,i),RParrayg(:,:,i,:),R);
                     W2mat = eye(nW2);
                  elseif IFlagW1==1 && IFlagW2==0
                     [W2mat,~,tResidDataMat] = ...
                        LOCALInputMultW2(RPnomg(:,:,i),RParrayg(:,:,i,:),R);
                     W1mat = eye(nW1);
                  else
                     error('RCT:ucover:NeedOneNonEmptyOrder','Either ORD1 or ORD2 must be nonempty');
                  end
                  w1rowdata(1,:,i) = diag(W1mat);
                  w2rowdata(1,:,i) = diag(W2mat);
                  ResidData(:,:,i,:) = tResidDataMat;
               end
            case {'OutputMult'}
               for i=1:nFreq
                  if IFlagW1==0 && IFlagW2==0
                     [W1mat,W2mat,~,tResidDataMat] = ...
                        LOCALOutputMult12(RPnomg(:,:,i),RParrayg(:,:,i,:),R1,R2);
                  elseif IFlagW1==0 && IFlagW2==1
                     [W1mat,~,tResidDataMat] = ...
                        LOCALOutputMultW1(RPnomg(:,:,i),RParrayg(:,:,i,:),R);
                     W2mat = eye(nW2);
                  elseif IFlagW1==1 && IFlagW2==0
                     [W2mat,~,tResidDataMat] = ...
                        LOCALOutputMultW2(RPnomg(:,:,i),RParrayg(:,:,i,:),R);
                     W1mat = eye(nW1);
                  else
                     error('RCT:ucover:NeedOneNonEmptyOrder','Either ORD1 or ORD2 must be nonempty');                     
                  end
                  w1rowdata(1,:,i) = diag(W1mat);
                  w2rowdata(1,:,i) = diag(W2mat);
                  ResidData(:,:,i,:) = tResidDataMat;
               end
            case 'Additive'
               for i=1:nFreq
                  H = zeros(nY,nU,nModels);
                  for j=1:nModels
                     H(:,:,j) = RParrayg(:,:,i,j) - RPnomg(:,:,i);
                  end
                  if IFlagW1==0 && IFlagW2==0
                     [W1mat,W2mat] = LOCALAdditiveWT(H,R1,R2);
                  elseif IFlagW1==0 && IFlagW2==1
                     W1mat = LOCALAdditiveW1(H,R);
                     W2mat = eye(nW2);
                  elseif IFlagW1==1 && IFlagW2==0
                     W2mat = LOCALAdditiveW2(H,R);
                     W1mat = eye(nW1);
                  else
                     error('RCT:ucover:NeedOneNonEmptyOrder','Either ORD1 or ORD2 must be nonempty');
                  end
                  w1rowdata(1,:,i) = diag(W1mat);
                  w2rowdata(1,:,i) = diag(W2mat);
               end
         end
         w1row = frd(w1rowdata,omega,TS); % returned in AllWData
         w2row = frd(w2rowdata,omega,TS); % returned in AllWData
         Residual = frd(ResidData,omega,TS);
      else  % InfoInFlag=0
         % XXX: DOES FRD have a reliable DIAG?
         %w1row = simplify(diag(ufrd(AllWDatain.W1opt))).';
         %w2row = simplify(diag(ufrd(AllWDatain.W2opt))).';
         w1row = diag(AllWDatain.W1opt).';
         w2row = diag(AllWDatain.W2opt).';
%          w1row = LOCALfrdMat2RowDiag(AllWDatain.W1opt);
%          w2row = LOCALfrdMat2RowDiag(AllWDatain.W2opt);
      end
      if FitFlag==1
         if IFlagW1==0
            W1 = [];
            d.type = '()';
            for j=1:nW1
               d.subs = {1 j};
               magdata = subsref(w1row,d);
               weight = LOCALmakefitweight(magdata,type,Pnomg);
               wt1 = fitmagfrd(magdata,w1Order(j),[],weight,1);
               W1 = append(W1,wt1);
            end
         else
            W1 = 1; % W1 = ss([],[],[],eye(nW1),TS);            
         end
         if IFlagW2==0
            W2 = [];
            d.type = '()';
            for j=1:nW2
               d.subs = {1 j};
               magdata = subsref(w2row,d);
               weight = LOCALmakefitweight(magdata,type,Pnomg);
               wt2 = fitmagfrd(magdata,w2Order(j),[],weight,1);
               W2 = append(W2,wt2);
            end
         else
            W2 = 1; % W2 = ss([],[],[],eye(nW2),TS);
         end
      elseif InteractiveFit==1
          % For future...
      else
         % XXX: DOES FRD have reliable DIAG
         %W1 = simplify(diag(ufrd(w1row))); % row-to-diag matrix
         %W2 = simplify(diag(ufrd(w2row))); % row-to-diag matrix
         W1 = diag(w1row); % row-to-diag matrix
         W2 = diag(w2row); % row-to-diag matrix
      end
   elseif IFlagW1==0 && IFlagW2==0
      % CFLAG==1, both W1 free, W2 free DOUBLEs
      if InfoInFlag == 0 
         %case 'FixedW1W2'
         RPnomg = get(Pnomg,'ResponseData');  %nY-nU-NFreq
         RParrayg = get(Parrayg(:,:,:),'ResponseData');  % nY-nU-NFreq-PE
         switch type
            % ALL W's are DOUBLES
            case 'InputMult'
               [W1mat,W2mat,~,ResidData] = ...
                  LOCALInputMult12(RPnomg,RParrayg,R1,R2);
            case 'OutputMult'
               [W1mat,W2mat,~,ResidData] = ...
                  LOCALOutputMult12(RPnomg,RParrayg,R1,R2);
            case 'Additive'
               H = RParrayg - repmat(RPnomg,[1 1 1 nModels]);
               ResidData = zeros(nY,nU,nFreq,nModels);
               [W1mat,W2mat] = LOCALAdditiveWT(H,R1,R2);
         end
      else
         W1mat = varargin{2}.W1;
         W2mat = varargin{2}.W2;
      end
      w1row = frd(repmat(diag(W1mat),[1 1 nFreq]),omega,TS);
      w2row = frd(repmat(diag(W2mat),[1 1 nFreq]),omega,TS);
      Residual = frd(ResidData,omega,TS);
      W1 = ss([],[],[],W1mat,TS);  % should be returned as SS, for consistency
      W2 = ss([],[],[],W2mat,TS);  % should be returned as SS, for consistency
   elseif IFlagW2==1
      %case 'FixedW1'
      RPnomg = get(Pnomg,'ResponseData');  %nY-nU-NFreq
      RParrayg = get(Parrayg(:,:,:),'ResponseData');  % nY-nU-NFreq-PE
      switch type
         % ALL W's are DOUBLES
         case 'InputMult'
            [W1mat,~,ResidData] = ...
               LOCALInputMultW1(RPnomg,RParrayg,R);
         case 'OutputMult'
            [W1mat,~,ResidData] = ...
               LOCALOutputMultW1(RPnomg,RParrayg,R);
         case 'Additive'
            H = RParrayg - repmat(RPnomg,[1 1 1 nModels]);
            ResidData = zeros(nY,nU,nFreq,nModels);
            [W1mat] = LOCALAdditiveW1(H,R);
      end
      W2mat = eye(nW2);
      w1row = frd(repmat(diag(W1mat),[1 1 nFreq]),omega,TS);
      w2row = frd(repmat(diag(W2mat),[1 1 nFreq]),omega,TS);
      Residual = frd(ResidData,omega,TS);
      W1 = ss([],[],[],W1mat,TS);  % should be returned as SS, for consistency
      W2 = 1; % W2 = ss([],[],[],W2mat,TS); 
   elseif IFlagW1==1
      %case 'FixedW2'
      RPnomg = get(Pnomg,'ResponseData');  %nY-nU-NFreq
      RParrayg = get(Parrayg(:,:,:),'ResponseData');  % nY-nU-NFreq-PE
      switch type
         % ALL W's are DOUBLES
         case 'InputMult'
            [W2mat,~,ResidData] = ...
               LOCALInputMultW2(RPnomg,RParrayg,R);
         case 'OutputMult'
            [W2mat,~,ResidData] = ...
               LOCALOutputMultW2(RPnomg,RParrayg,R);
         case 'Additive'
            H = RParrayg - repmat(RPnomg,[1 1 1 nModels]);
            ResidData = zeros(nY,nU,nFreq,nModels);
            [W2mat] = LOCALAdditiveW2(H,R);
      end
      W1mat = eye(nW1);
      Residual = frd(ResidData,omega,TS);
      w1row = frd(repmat(diag(W1mat),[1 1 nFreq]),omega,TS);
      w2row = frd(repmat(diag(W2mat),[1 1 nFreq]),omega,TS);
      W1 = 1; % W1 = ss([],[],[],W1mat,TS);
      W2 = ss([],[],[],W2mat,TS);  % should be returned as SS, for consistency
   end
   % Weights are now ready.  Create ULTIDYN object, and combine
   % with Pnom in the appropriate manner to form PU, the uncertain object.
   Delta = ultidyn(DeltaName,[nW1 nW2]); 
   switch type
       case 'InputMult'
          Pu = Pnom*(eye(nU) + W1*Delta*W2);
          Pu.InputName = get(Pnom,'InputName');
       case 'OutputMult'
          Pu = (eye(nY) + W1*Delta*W2)*Pnom;
          Pu.OutputName = get(Pnom,'OutputName');
       case 'Additive'
          Pu = Pnom + W1*Delta*W2;
          Pu.InputName = get(Pnom,'InputName');
          Pu.OutputName = get(Pnom,'OutputName');
   end
   AllWData.W1 = W1;
   AllWData.W2 = W2;
   %AllWData.ScalarW = [];
   AllWData.W1opt = diag(w1row);
   AllWData.W2opt = diag(w2row);
   %AllWData.FrdW = [];
   AllWData.Ord1 = w1Order;
   AllWData.Ord2 = w2Order;
   %AllWData.ScalarOrd = [];
   AllWData.Type = type;
   AllWData.DeltaName = DeltaName;
   AllWData.Residual = reshape(Residual,szPaOrig(3:end));
   %AllWData.PNominal = Pnom;
   if strcmp(type,'InputMult') || strcmp(type,'OutputMult')
       [ResidData,ResidFreq] = frdata(Residual);
       nFreq = size(ResidData,3);
       nModels = size(ResidData,4);
       ResidNorm = zeros(nFreq,nModels);
       for i=1:nFreq % loop over frequency
            for j=1:nModels
                ResidNorm(i,j) = norm(ResidData(:,:,i,j));
            end
       end
       ResidNorm = frd(max(ResidNorm,[],2), ResidFreq, TS);
       if strcmp(type,'InputMult') 
            PW2 = fnorm(Pnomg*AllWData.W2opt);
            W1n = fnorm(AllWData.W1opt);
            wcRelResid = norm(ResidNorm/PW2/W1n,inf);
       else
            W1P = fnorm(AllWData.W1opt*Pnomg);
            W2n = fnorm(AllWData.W2opt);
            wcRelResid = norm(ResidNorm/W1P/W2n,inf);
       end       
   else
       wcRelResid = 0;
   end
   if wcRelResid >1e-5
        warning('RCT:ucover:NonZeroResidualWarning','The uncertainty model is not able to cover PARRAY. Residual is contained in INFO.Residual');
   end
end

% MAIN SUBROUTINES
%----------------beginning of LOCALInputMult12-----------------------------
function [W1,W2,copt,ResidData] = LOCALInputMult12(P,Ptilde,RW1,RW2,xinit)
% Given:
%   P ny-by-nu-by-nw complex matrix
%   Ptilde ny-by-nu-by-nw-by-nm complex matrix
%   RW1 nu-by-1, real, positive numbers
%   RW2 nu-by-1, real, positive numbers
%
%   The entries of W1 and W2 are "minimized", via the cost
%       sum_i (RW1(i)*|W1(i)|^2 + sum_i (RW2(i)*|W2(i)|^2
%
% See page 38, Notebook: MUSYN Aug 08
% See Also: LMI: page 82B-84 MUSYN0506 folder.

if nargin==4
   xinit = [];
end
szPtilde = [size(Ptilde) 1 1];
nY = szPtilde(1);
nU = szPtilde(2); 
nFreq = szPtilde(3);
nModels = szPtilde(4);
ResidData = zeros(szPtilde); 

setlmis([]);
[W1s] = lmivar(1,repmat([1 0],[nU 1]));
[W2s] = lmivar(1,repmat([1 0],[nU 1]));

cnt = 0;
for i=1:nFreq
   [U1,U2,V1,~,Sigma1,rk] = LOCALsvdrank(P(:,:,i),nY,nU);
   if rk==nU
      % V1 = V, V2 = [];
      psinv = (V1/Sigma1)*U1';
      Peff = eye(nU);
      PU2 = U2*U2';
      for j=1:nModels
         % P=EYE case
         PtildeEff = psinv*Ptilde(:,:,i,j);
         ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j);
         Heff = PtildeEff - Peff; % 
         cnt = cnt + 1;   
         HR = real(Heff);
         HI = imag(Heff);
         lmiterm([-cnt 3 3 W1s],1,1);
         lmiterm([-cnt 1 1 W1s],1,1);
         lmiterm([-cnt 2 2 W2s],1,1);  % nu
         lmiterm([-cnt 4 4 W2s],1,1);  % nu
         lmiterm([-cnt 1 2 0],HR);
         lmiterm([-cnt 3 4 0],HR);
         lmiterm([-cnt 1 4 0],-HI);
         lmiterm([-cnt 2 3 0],HI');
      end
   elseif rk==nY
      Peff = P(:,:,i);
      PR = real(Peff);
      PI = imag(Peff);
      %PU1 = U1*U1'; % will be EYE
      %PU2 = U2*U2'; % will be ZEROS
      for j=1:nModels
%          PtildeP1 = PU1*Ptilde(:,:,i,j);
%          ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j); % Identically 0, since U2 is NY-by-0
         PtildeP1 = Ptilde(:,:,i,j);
         Heff = PtildeP1 - Peff; %
         HR = real(Heff);
         HI = imag(Heff);
         cnt = cnt + 1;   
         lmiterm([-cnt 3 3 W1s],PR,PR');
         lmiterm([-cnt 1 1 W1s],PR,PR');
         lmiterm([-cnt 1 1 W1s],PI,PI');
         lmiterm([-cnt 3 3 W1s],PI,PI');
         lmiterm([-cnt 1 3 W1s],PR,PI');
         lmiterm([-cnt 1 3 W1s],-PI,PR');
         lmiterm([-cnt 2 2 W2s],1,1);  % nu
         lmiterm([-cnt 4 4 W2s],1,1);  % nu
         lmiterm([-cnt 1 2 0],HR);
         lmiterm([-cnt 3 4 0],HR);
         lmiterm([-cnt 1 4 0],-HI);
         lmiterm([-cnt 2 3 0],HI');
      end
   elseif rk>0
      % USV' = NY-by-NU. so V' [rk-by-NU;(NU-rk)-by-NU]
      % and V' = [V1';V2'], so V1 is NU-by-rk
      Peff = Sigma1*V1';  % rk-by-NU
      PU2 = U2*U2';
      % I assume that Peff is Peff, problem pointed out in pascal's April 15 bug
      PR = real(Peff);
      PI = imag(Peff);
      for j=1:nModels
         ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j);
         Heff = U1'*Ptilde(:,:,i,j) - Peff; % 
         HR = real(Heff);
         HI = imag(Heff);
         cnt = cnt + 1;   
         % LMI is the same as rk==nY case, with appropriate
         % definitions for Peff, PtildeEff, etc.
         lmiterm([-cnt 3 3 W1s],PR,PR');
         lmiterm([-cnt 1 1 W1s],PR,PR');
         lmiterm([-cnt 1 1 W1s],PI,PI');
         lmiterm([-cnt 3 3 W1s],PI,PI');
         lmiterm([-cnt 1 3 W1s],PR,PI');
         lmiterm([-cnt 1 3 W1s],-PI,PR');
         lmiterm([-cnt 2 2 W2s],1,1);  % nu
         lmiterm([-cnt 4 4 W2s],1,1);  % nu
         lmiterm([-cnt 1 2 0],HR);
         lmiterm([-cnt 3 4 0],HR);
         lmiterm([-cnt 1 4 0],-HI);
         lmiterm([-cnt 2 3 0],HI');
      end
   else
      error('RCT:ucover:Identically0PNominal','P is identically zero');
   end
end
lmiterm([-(nModels*nFreq+1) 1 1 W1s],1,1);  % nu
lmiterm([-(nModels*nFreq+2) 1 1 W2s],1,1);  % nu
% lmiterm([(N+3) 1 1 W1s],eye(nr),eye(nr));
% lmiterm([(N+4) 1 1 W2s],eye(nc),eye(nc));
% lmiterm([-(N+3) 1 1 0],(1.005)*maxnrm*eye(nr));
% lmiterm([-(N+4) 1 1 0],(1.005)*maxnrm*eye(nc));
lmis = getlmis;
c = [RW1(:);RW2(:)];
options = [0 0 0 0 1];
[copt,xopt] = mincx(lmis,c,options,xinit);
if isempty(copt)
    options(3) = -1;
    [copt,xopt] = mincx(lmis,c,options,xinit);
end
W1 = diag(sqrt(xopt(1:nU)));
W2 = diag(sqrt(xopt(nU+1:nU+nU)));
%------------------End of LOCALInputMult12-------------------------------


%----------------------Beginning of LOCALOutputMult12----------------------
function [W1,W2,copt,ResidData] = LOCALOutputMult12(P,Ptilde,RW1,RW2,xinit)
% Conjugate/Transpose, call InputMult, transform back
if nargin==4
   xinit = [];
end
% Need to commit to rules on Array Dimensions, note differences in PERMUTE
Pmod = conj(permute(P,[2 1 3:ndims(P)]));
Ptildemod = conj(permute(Ptilde,[2 1 3:ndims(Ptilde)]));
[W2,W1,copt,tResidData] = LOCALInputMult12(Pmod,Ptildemod,RW2,RW1,xinit);
ResidData = conj(permute(tResidData,[2 1 3 4]));
%---------------------------End of LOCALOutputMult12-----------------------


%----------------beginning of LOCALInputMultW1-----------------------------
function [W1,copt,ResidData] = LOCALInputMultW1(P,Ptilde,RW1,xinit)
% Given:
%   P ny-by-nu-by-nw complex matrix
%   Ptilde ny-by-nu-by-nm-by-nw complex matrix
%   RW1 nu-by-1, real, positive numbers
%
%   The entries of W1 are "minimized", via the cost
%       sum_i (RW1(i)*|W1(i)|^2 
%
% See page 38, Notebook: MUSYN Aug 08
% See Also: LMI: page 82B-84 MUSYN0506 folder.

if nargin==3
   xinit = [];
end
szPtilde = [size(Ptilde) 1 1];
nY = szPtilde(1);
nU = szPtilde(2); 
nFreq = szPtilde(3);
nModels = szPtilde(4);
ResidData = zeros(szPtilde); 

setlmis([]);
[W1s] = lmivar(1,repmat([1 0],[nU 1]));

cnt = 0;
for i=1:nFreq
   [U1,U2,V1,~,Sigma1,rk] = LOCALsvdrank(P(:,:,i),nY,nU);
   if rk==nU
      % V1 = V, V2 = [];
      psinv = (V1/Sigma1)*U1';
      Peff = eye(nU);
      PU2 = U2*U2';
      for j=1:nModels
         % P=EYE case
         PtildeEff = psinv*Ptilde(:,:,i,j);
         ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j);
         Heff = PtildeEff - Peff; % 
         cnt = cnt + 1;   
         HR = real(Heff);
         HI = imag(Heff);
         Hmat = [HR -HI;HI HR];
         Cmat = Hmat*Hmat';
         lmiterm([-cnt 1 1 W1s],1,1);
         lmiterm([-cnt 2 2 W1s],1,1);
         lmiterm([cnt 1 1 0],Cmat(1:nU,1:nU));
         lmiterm([cnt 1 2 0],Cmat(1:nU,nU+1:2*nU));
         lmiterm([cnt 2 2 0],Cmat(nU+1:2*nU,nU+1:2*nU));
      end
   elseif rk==nY
      Peff = P(:,:,i);
      PR = real(Peff);
      PI = imag(Peff);
      %PU1 = U1*U1'; % will be EYE
      %PU2 = U2*U2'; % will be ZEROS
      for j=1:nModels
%          PtildeP1 = PU1*Ptilde(:,:,i,j);
%          ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j); % Identically 0, since U2 is NY-by-0
         PtildeP1 = Ptilde(:,:,i,j);
         Heff = PtildeP1 - Peff; %
         HR = real(Heff);
         HI = imag(Heff);
         cnt = cnt + 1;   
         Hmat = [HR -HI;HI HR];
         Cmat = Hmat*Hmat';
         lmiterm([-cnt 1 1 W1s],PR,PR');
         lmiterm([-cnt 1 1 W1s],PI,PI');
         lmiterm([-cnt 1 2 W1s],PR,PI');
         lmiterm([-cnt 1 2 W1s],-PI,PR');
         lmiterm([-cnt 2 2 W1s],PR,PR');
         lmiterm([-cnt 2 2 W1s],PI,PI');
         % AKP, 1/24/09, changing nU (below) to nY
         lmiterm([cnt 1 1 0],Cmat(1:nY,1:nY));
         lmiterm([cnt 1 2 0],Cmat(1:nY,nY+1:2*nY));
         lmiterm([cnt 2 2 0],Cmat(nY+1:2*nY,nY+1:2*nY));
      end
   elseif rk>0
      % USV' = NY-by-NU. so V' [rk-by-NU;(NU-rk)-by-NU]
      % and V' = [V1';V2'], so V1 is NU-by-rk
      Peff = Sigma1*V1';  % rk-by-NU
      PR = real(Peff);
      PI = imag(Peff);
      PU2 = U2*U2';
      for j=1:nModels
         ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j);
         Heff = U1'*Ptilde(:,:,i,j) - Peff; % 
         HR = real(Heff);
         HI = imag(Heff);
         cnt = cnt + 1;   
         Hmat = [HR -HI;HI HR];
         Cmat = Hmat*Hmat'; % 2rk-2rk
         lmiterm([-cnt 1 1 W1s],PR,PR');
         lmiterm([-cnt 1 1 W1s],PI,PI');
         lmiterm([-cnt 1 2 W1s],PR,PI');
         lmiterm([-cnt 1 2 W1s],-PI,PR');
         lmiterm([-cnt 2 2 W1s],PR,PR');
         lmiterm([-cnt 2 2 W1s],PI,PI');
         lmiterm([cnt 1 1 0],Cmat(1:rk,1:rk));
         lmiterm([cnt 1 2 0],Cmat(1:rk,rk+1:2*rk));
         lmiterm([cnt 2 2 0],Cmat(rk+1:2*rk,rk+1:2*rk));
      end
   else
      error('RCT:ucover:Identically0PNominal','P is identically zero');
   end
end
lmiterm([-(nModels*nFreq+1) 1 1 W1s],1,1);  % nu
lmis = getlmis;
c = RW1(:);
options = [0 0 0 0 1];
[copt,xopt] = mincx(lmis,c,options,xinit);
if isempty(copt)
    options(3) = -1;
    [copt,xopt] = mincx(lmis,c,options,xinit);
end
W1 = diag(sqrt(xopt(1:nU)));
%------------------End of LOCALInputMultW1-------------------------------


%----------------beginning of LOCALInputMultW2-----------------------------
function [W2,copt,ResidData] = LOCALInputMultW2(P,Ptilde,RW2,xinit)
% Given:
%   P ny-by-nu-by-nw complex matrix
%   Ptilde ny-by-nu-by-nm-by-nw complex matrix
%   RW2 nu-by-1, real, positive numbers
%
%   The entries of W2 are "minimized", via the cost
%       sum_i (RW2(i)*|W2(i)|^2 
%
% See page 38, Notebook: MUSYN Aug 08
% See Also: LMI: page 82B-84 MUSYN0506 folder.

if nargin==3
   xinit = [];
end
szPtilde = [size(Ptilde) 1 1];
nY = szPtilde(1);
nU = szPtilde(2); 
nFreq = szPtilde(3);
nModels = szPtilde(4);
ResidData = zeros(szPtilde); 

setlmis([]);
[W2s] = lmivar(1,repmat([1 0],[nU 1]));

cnt = 0;
for i=1:nFreq
   [U1,U2,V1,~,Sigma1,rk] = LOCALsvdrank(P(:,:,i),nY,nU);
   if rk==nU
      % V1 = V, V2 = [];
      psinv = (V1/Sigma1)*U1';
      Peff = eye(nU);
      PU2 = U2*U2';
      for j=1:nModels
         % P=EYE case
         PtildeEff = psinv*Ptilde(:,:,i,j);
         ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j);
         Heff = PtildeEff - Peff; % 
         cnt = cnt + 1;   
         HR = real(Heff);
         HI = imag(Heff);
         Hmat = [HR -HI;HI HR];
         Dmat = Hmat'*Hmat;
         lmiterm([-cnt 1 1 W2s],1,1);
         lmiterm([-cnt 2 2 W2s],1,1);
         lmiterm([cnt 1 1 0],Dmat(1:nU,1:nU));
         lmiterm([cnt 1 2 0],Dmat(1:nU,nU+1:2*nU));
         lmiterm([cnt 2 2 0],Dmat(nU+1:2*nU,nU+1:2*nU));
      end
   elseif rk==nY
      Peff = P(:,:,i);
      PR = real(Peff);
      PI = imag(Peff);
      %PU1 = U1*U1'; % will be EYE
      %PU2 = U2*U2'; % will be ZEROS
      for j=1:nModels
%          PtildeP1 = PU1*Ptilde(:,:,i,j);
%          ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j); % Identically 0, since U2 is NY-by-0
         PtildeP1 = Ptilde(:,:,i,j);
         Heff = PtildeP1 - Peff; %
         HR = real(Heff);
         HI = imag(Heff);
         cnt = cnt + 1;   
         Hmat = [HR -HI;HI HR];
         Pmat = [PR PI;PI -PR];
         Pmat = Pmat*Pmat';
         Dmat = Hmat'*(Pmat\Hmat);
         lmiterm([-cnt 1 1 W2s],1,1);
         lmiterm([-cnt 2 2 W2s],1,1);
         lmiterm([cnt 1 1 0],Dmat(1:nU,1:nU));
         lmiterm([cnt 1 2 0],Dmat(1:nU,nU+1:2*nU));
         lmiterm([cnt 2 2 0],Dmat(nU+1:2*nU,nU+1:2*nU));
      end
   elseif rk>0
      % USV' = NY-by-NU. so V' [rk-by-NU;(NU-rk)-by-NU]
      % and V' = [V1';V2'], so V1 is NU-by-rk
      Peff = Sigma1*V1';  % rk-by-NU
      PR = real(Peff);
      PI = imag(Peff);
      PU2 = U2*U2';
      for j=1:nModels
         ResidData(:,:,i,j) = PU2*Ptilde(:,:,i,j);
         Heff = U1'*Ptilde(:,:,i,j) - Peff; % 
         HR = real(Heff);
         HI = imag(Heff);
         cnt = cnt + 1;   
         Hmat = [HR -HI;HI HR];
         Pmat = [PR PI;PI -PR];
         Pmat = Pmat*Pmat';
         Dmat = Hmat'*(Pmat\Hmat);         % NU-by-NU
         lmiterm([-cnt 1 1 W2s],1,1);
         lmiterm([-cnt 2 2 W2s],1,1);
         lmiterm([cnt 1 1 0],Dmat(1:nU,1:nU));
         lmiterm([cnt 1 2 0],Dmat(1:nU,nU+1:2*nU));
         lmiterm([cnt 2 2 0],Dmat(nU+1:2*nU,nU+1:2*nU));
      end
   else
      error('RCT:ucover:Identically0PNominal','P is identically zero');
   end
end
lmiterm([-(nModels*nFreq+1) 1 1 W2s],1,1);  % nu
lmis = getlmis;
c = RW2(:);
options = [0 0 0 0 1];
[copt,xopt] = mincx(lmis,c,options,xinit);
if isempty(copt)
    options(3) = -1;
    [copt,xopt] = mincx(lmis,c,options,xinit);
end
W2 = diag(sqrt(xopt(1:nU)));
%------------------End of LOCALInputMultW2-------------------------------


%----------------------Beginning of LOCALOutputMultW1----------------------
function [W1,copt,ResidData] = LOCALOutputMultW1(P,Ptilde,RW1,xinit)
% Conjugate/Transpose, call InputMult, transform back
if nargin==3
   xinit = [];
end
% Need to commit to rules on Array Dimensions, note differences in PERMUTE
Pmod = conj(permute(P,[2 1 3:ndims(P)]));
Ptildemod = conj(permute(Ptilde,[2 1 3:ndims(Ptilde)]));
[W1,copt,tResidData] = LOCALInputMultW2(Pmod,Ptildemod,RW1,xinit);
ResidData = conj(permute(tResidData,[2 1 3 4]));
%---------------------------End of LOCALOutputMultW1-----------------------


%----------------------Beginning of LOCALOutputMultW2----------------------
function [W2,copt,ResidData] = LOCALOutputMultW2(P,Ptilde,RW2,xinit)
% Conjugate/Transpose, call InputMult, transform back
if nargin==3
   xinit = [];
end
% Need to commit to rules on Array Dimensions, note differences in PERMUTE
Pmod = conj(permute(P,[2 1 3:ndims(P)]));
Ptildemod = conj(permute(Ptilde,[2 1 3:ndims(Ptilde)]));
[W2,copt,tResidData] = LOCALInputMultW1(Pmod,Ptildemod,RW2,xinit);
ResidData = conj(permute(tResidData,[2 1 3 4]));
%---------------------------End of LOCALOutputMultW2-----------------------


% ----------------Beginning of LOCALAdditiveWT-----------------------------
function [W1,W2,copt] = LOCALAdditiveWT(H,RW1,RW2,xinit)
% Given:
%   H ny-by-nu-by-nm-by-nw complex matrix
%   RW1 ny-by-1, real, positive numbers
%   RW2 nu-by-1, real, positive numbers
% Find W1 (ny-by-ny) and W2 (nu-by-nu), diagonal,
if nargin==3
   xinit = [];
end
szH = [size(H) 1 1];
nY = szH(1);
nU = szH(2); 
nFreq = szH(3);
nModels = szH(4);
setlmis([]);
[W1s] = lmivar(1,repmat([1 0],[nY 1]));
[W2s] = lmivar(1,repmat([1 0],[nU 1]));
HR = real(H);
HI = imag(H);
for i=1:nFreq  % number of frequencies
   for j=1:nModels  % number of models
      cnt = j + (i-1)*nModels;   
      % nrmH(i) = norm(H(:,:,i));
      lmiterm([-cnt 1 1 W1s],1,1);  %ny
      lmiterm([-cnt 2 2 W1s],1,1);
      lmiterm([-cnt 3 3 W2s],1,1); % nu
      lmiterm([-cnt 4 4 W2s],1,1);
      lmiterm([-cnt 1 3 0],HR(:,:,i,j));
      lmiterm([-cnt 1 4 0],-HI(:,:,i,j));
      lmiterm([-cnt 2 3 0],HI(:,:,i,j));
      lmiterm([-cnt 2 4 0],HR(:,:,i,j));
   end
end
lmiterm([-(nModels*nFreq+1) 1 1 W1s],1,1);  %ny
lmiterm([-(nModels*nFreq+2) 1 1 W2s],1,1);  %nu
lmis = getlmis;
c = [RW1(:);RW2(:)];
options = [0 0 0 0 1];
[copt,xopt] = mincx(lmis,c,options,xinit);
if isempty(copt)
    options(3) = -1;
    [copt,xopt] = mincx(lmis,c,options,xinit);
end
W1 = diag(sqrt(xopt(1:nY)));
W2 = diag(sqrt(xopt(nY+1:nY+nU)));
% ----------------------End of LOCALAdditiveWT-----------------------------

% ---------------------Beginning of LOCALAdditiveW1------------------------
function [W1,copt] = LOCALAdditiveW1(H,RW1,xinit)
if nargin==2
   xinit = [];
end
szH = [size(H) 1 1];
nY = szH(1);
nFreq = szH(3);
nModels = szH(4);
setlmis([]);
W1s = lmivar(1,repmat([1 0],[nY 1]));
HR = real(H);
HI = imag(H);
for i=1:nFreq  % number of frequencies
   for j=1:nModels  % number of models
      cnt = j + (i-1)*nModels;   
      term34 = [HR(:,:,i,j) -HI(:,:,i,j);HI(:,:,i,j) HR(:,:,i,j)];
      ttt = term34*term34';
      lmiterm([-cnt 1 1 W1s],1,1);
      lmiterm([-cnt 2 2 W1s],1,1);
      lmiterm([cnt 1 1 0],ttt(1:nY,1:nY));
      lmiterm([cnt 1 2 0],ttt(1:nY,nY+1:2*nY));
      lmiterm([cnt 2 2 0],ttt(nY+1:2*nY,nY+1:2*nY));
   end
end
lmiterm([-(nModels*nFreq+1) 1 1 W1s],1,1);
lmis = getlmis;
c = RW1(:);
options = [0 0 0 0 1];
[copt,xopt] = mincx(lmis,c,options,xinit);
W1 = diag(sqrt(xopt(1:nY)));
% ---------------------------End of LOCALAdditiveW1------------------------


% ---------------------Beginning of LOCALAdditiveW2------------------------
function [W2,copt] = LOCALAdditiveW2(H,RW2,xinit)
if nargin==2
   xinit = [];
end
szH = [size(H) 1 1];
nU = szH(2); 
nFreq = szH(3);
nModels = szH(4);
setlmis([]);
[W2s] = lmivar(1,repmat([1 0],[nU 1]));
HR = real(H);
HI = imag(H);
for i=1:nFreq  % number of frequencies
   for j=1:nModels  % number of models
      cnt = j + (i-1)*nModels;   
      term34 = [HR(:,:,i,j) -HI(:,:,i,j);HI(:,:,i,j) HR(:,:,i,j)];
      ttt = term34'*term34;
      lmiterm([-cnt 1 1 W2s],1,1);
      lmiterm([-cnt 2 2 W2s],1,1);
      lmiterm([cnt 1 1 0],ttt(1:nU,1:nU));
      lmiterm([cnt 1 2 0],ttt(1:nU,nU+1:2*nU));
      lmiterm([cnt 2 2 0],ttt(nU+1:2*nU,nU+1:2*nU));
   end
end
lmiterm([-(nModels*nFreq+1) 1 1 W2s],1,1);
lmis = getlmis;
c = RW2(:);
options = [0 0 0 0 1];
[copt,xopt] = mincx(lmis,c,options,xinit);
W2 = diag(sqrt(xopt(1:nU)));
% ---------------------------End of LOCALAdditiveW2------------------------


% -----------------------------------------------------------
function [Pu,wt,AllWData] = LOCALScalar(Pnom,Pnomg,Parrayg,omega,...
   scalarorder,DeltaName,nY,nU,nFreq,nModels,type,InfoInFlag,data2fit)

ResidData = zeros(nY,nU,nFreq,nModels);
TS = get(Pnom,'TS');
switch type
  case 'Additive'
     if ~InfoInFlag
        fgap = fnorm(Parrayg(:,:,:) - Pnomg);
        maxgain = max(fgap.ResponseData,[],4);
        data2fit = frd(maxgain,omega,TS);
     end
     wcRelResid = 0;
     if scalarorder~=-1 && ~isinf(scalarorder)
        weight = LOCALmakefitweight(data2fit,type,Pnomg);
        wt = LOCALfitmagfrdIteration(data2fit,scalarorder,weight);
     else
        wt = data2fit;
     end
     Delta = ultidyn(DeltaName,[nY nU]);
     if nY<=nU
        Pu = Pnom + wt*Delta;
        W1 = wt*eye(nY);
        W2 = eye(nU);
        Ord1 = scalarorder;
        Ord2 = 0;
        W1g = data2fit;
        W2g = frd(ones(1,1,nFreq),omega,TS);
     else
        Pu = Pnom + Delta*wt;
        W2 = wt*eye(nU);
        W1 = eye(nY);
        Ord2 = scalarorder;
        Ord1 = 0;
        W2g = data2fit;
        W1g = frd(ones(1,1,nFreq),omega,TS);
     end
     Pu.InputName = get(Pnom,'InputName');
     Pu.OutputName = get(Pnom,'OutputName');
  case 'InputMult'
     if nY>nU
%         warning('Since nY>nU, the Input Multiplicative model might not cover PARRAY.  Residual is contained in INFO.Residual');
        % Since there are more outputs than inputs, InputMultiplicative
        % uncertainty model can not cover any plants whose Output Range
        % space is not contained in Nominal Output Range space.
     end
     if ~InfoInFlag
        RPnomg = get(Pnomg,'ResponseData');  
        rdata = Parrayg(:,:,:);
        RParrayg = get(rdata,'ResponseData');
        maxgain = zeros(1,1,nFreq);
        gainarray = zeros(nModels,1);
        wcRelResid =zeros(nFreq,1);
        for i=1:nFreq % loop over frequency
           [U1,U2,V1,~,Sigma1] = LOCALsvdrank(RPnomg(:,:,i),nY,nU);
           ResidNorm = zeros(nModels,1);
           for j=1:nModels
              ResidData(:,:,i,j) = U2*U2'*RParrayg(:,:,i,j);
              gainarray(j) = norm(Sigma1\(U1'*RParrayg(:,:,i,j))-V1');
              ResidNorm(j) =norm(ResidData(:,:,i,j));
           end
           maxgain(1,1,i) = max(gainarray);
           wcRelResid(i)  = max(ResidNorm)/maxgain(1,1,i);
        end
        wcRelResid = max(wcRelResid);
        data2fit = frd(maxgain,omega,get(Parrayg,'Ts'));
     else
        wcRelResid = 0;
        %data2fit = Pnomg; %Passed in to fit
     end
     if scalarorder~=-1 && ~isinf(scalarorder)
        weight = LOCALmakefitweight(data2fit,type,Pnomg);
        wt = LOCALfitmagfrdIteration(data2fit,scalarorder,weight);
     else
        wt = data2fit;
     end
     Delta = ultidyn(DeltaName,[nU nU]);
     W1 = wt*eye(nU);
     W2 = eye(nU);
     Ord1 = scalarorder;
     Ord2 = 0;     
     W1g = data2fit;
     W2g = frd(ones(1,1,nFreq),omega,TS);
     Pu = Pnom*(eye(nU) + wt*Delta);
     Pu.InputName = get(Pnom,'InputName');
  case 'OutputMult'
     if ~InfoInFlag
        % InputMult code, with CCJT
        RPnomg = get(Pnomg,'ResponseData');  
        rdata = Parrayg(:,:,:);
        RParrayg = get(rdata,'ResponseData');
        maxgain = zeros(1,1,nFreq);
        gainarray = zeros(nModels,1);
        wcRelResid =zeros(nFreq,1);
        for i=1:nFreq % loop over frequency
           [U1,U2,V1,~,Sigma1] = LOCALsvdrank(RPnomg(:,:,i)',nU,nY);
           ResidNorm = zeros(nModels,1);
           for j=1:nModels
              ResidData(:,:,i,j) = (U2*U2'*RParrayg(:,:,i,j)')';
              gainarray(j) = norm(Sigma1\(U1'*RParrayg(:,:,i,j)')-V1');
              ResidNorm(j) =norm(ResidData(:,:,i,j));
           end
           maxgain(1,1,i) = max(gainarray);
           wcRelResid(i)  = max(ResidNorm)/maxgain(1,1,i);
        end
        wcRelResid = max(wcRelResid);
        data2fit = frd(maxgain,omega,get(Parrayg,'Ts'));
     else
        wcRelResid = 0;
     end
     if scalarorder~=-1 && ~isinf(scalarorder)
        weight = LOCALmakefitweight(data2fit,type,Pnomg);
        wt = LOCALfitmagfrdIteration(data2fit,scalarorder,weight);
     else
        wt = data2fit;
     end
     Delta = ultidyn(DeltaName,[nY nY]);
     W1 = wt*eye(nY);
     W2 = eye(nY);
     Ord1 = scalarorder;
     Ord2 = 0;
     W1g = data2fit;
     W2g = frd(ones(1,1,nFreq),omega,TS);
     Pu = (eye(nY) + wt*Delta)*Pnom;
     Pu.OutputName = get(Pnom,'OutputName');
  otherwise
end
AllWData.Ord1 = Ord1;
AllWData.Ord2 = Ord2;
%AllWData.ScalarOrd = scalarorder;
AllWData.W1opt = W1g;   % 1-by-1 frd
AllWData.W2opt = W2g;   % 1-by-1 frd
%AllWData.ScalarFrdW = W1g;   % 1-by-1 frd

%AllWData.ScalarW = wt;
%AllWData.W1 = W1;
%AllWData.W2 = W2;
S.type = '()';
S.subs = {1,1};
AllWData.W1 = subsref(W1,S);
AllWData.W2 = subsref(W2,S);

AllWData.Type = type;
AllWData.DeltaName = DeltaName;
AllWData.Residual = frd(ResidData,omega,get(Pnom,'Ts'));
if wcRelResid >1e-5
    warning('RCT:ucover:NonZeroResidualWarning','The uncertainty model was not able to cover PARRAY. Residual is contained in INFO.Residual');
end
%AllWData.PNominal = Pnom;


%-------------------------------------------------------------
function [IFlagW1,IFlagW2,CFlag,FitFlag,ScalarFlag] = ...
   LOCALRetrieveOriginalFlags(AllWDatain)
ScalarFlag = isscalar(AllWDatain.Ord1) && isscalar(AllWDatain.Ord2);
if ScalarFlag
   IFlagW1 = [];  % can't tell which
   IFlagW2 = [];  % can't tell which
   CFlag = AllWDatain.Ord1==0 && AllWDatain.Ord2==0;
   FitFlag = ~CFlag;
else
   %if norm(W2g(:,:,1)-eye(size(W2g,1)))==0
   IFlagW1 = isempty(AllWDatain.Ord1);
   IFlagW2 = isempty(AllWDatain.Ord2);
   W1g = get(AllWDatain.W1opt,'ResponseData');
   W2g = get(AllWDatain.W2opt,'ResponseData');
   dW1g = diff(W1g,1,3);
   dW2g = diff(W2g,1,3);
   CFlag = (norm(dW1g(:))==0) && (norm(dW2g(:))==0);
   FitFlag = ~CFlag;
end

%-------------------------------------------------------------
function [R,R1,R2] = LOCALMakeR(IFlagW1,R1,nW2,IFlagW2,nW1,R2)
if IFlagW1==1
   R = R1;
   if isempty(R)
      R = ones(nW2,1);
   elseif isscalar(R)
      R = repmat(R,[nW2 1]);
   elseif ~all(size(R)==[nW2 1])
      error('RCT:ucover:InvalidRW2','Invalid RWeight Dimension for W2');
   end
elseif IFlagW2==1
   R = R1;
   if isempty(R) 
      R = ones(nW1,1);
   elseif isscalar(R)
      R = repmat(R,[nW1 1]);
   elseif ~all(size(R)==[nW1 1])
      error('RCT:ucover:InvalidRW1','Invalid RWeight Dimension for W1');
   end
else
   % DOUBLE, FRD(?)
   R = [];
   if isempty(R1)
      R1 = ones(nW1,1);
   elseif isscalar(R1)
      R1 = repmat(R1,[nW1 1]);
   elseif ~all(size(R1)==[nW1 1])
      error('RCT:ucover:InvalidRW1','Invalid RWeight Dimension for W1');
   end
   if isempty(R2)
      R2 = ones(nW2,1);
   elseif isscalar(R2)
      R2 = repmat(R2,[nW2 1]);
   elseif ~all(size(R2)==[nW2 1])
      error('RCT:ucover:InvalidRW2','Invalid RWeight Dimension for W2');
   end
end

%-------------------------------------------------------------
function [U1,U2,V1,V2,Sigma1,rk,sigmax] = LOCALsvdrank(M,nY,nU)
nMax = max(nY,nU);
[U,Sigma,V] = svd(M);
if nY>1 && nU>1
   dSigma = diag(Sigma);
else
   dSigma = Sigma;
end
sigmax = Sigma(1);
rktol = nMax*eps(sigmax);
rk = sum(dSigma>rktol);
U1 = U(:,1:rk);
U2 = U(:,rk+1:end);
V1 = V(:,1:rk);
V2 = V(:,rk+1:end);
Sigma1 = Sigma(1:rk,1:rk);

function weight = LOCALmakefitweight(data2fit,type,Pnomg)
Rd = frdata(data2fit);
[PRd,Fr,TS] = frdata(Pnomg);
npts = length(Fr);
wRd = repmat(0.01,[1 1 npts]);
switch type
    case {'InputMult' 'OutputMult'}
        ratio = Rd(:);
    case 'Additive'
        svdP = zeros(npts,1);
        for i=1:npts
            svdP(i) = norm(PRd(:,:,i));
        end
        ratio = Rd(:)./svdP;
end
wRd(:,:,ratio>0.01 & ratio<100) = 1;
weight = frd(wRd,Fr,TS);

%----------------------
function wAll = LOCALpickw(Pnom,Parray)
nBins = 12;
nA = size(Parray,3) + 1;
wA = cell(nA,1);
wMin = inf; wMax = -inf;
D = [getPrivateData(Parray) ; getPrivateData(Pnom)];
for i=1:nA
   wA{i} = sigmaGrid(D(i),[]);
   wMin = min(wMin,min(wA{i}));
   wMax = max(wMax,max(wA{i}));
end
if wMax>(1e8)*wMin
   %warning('RCT:ucover:WideFrequencyWarning','Auto-Frequency Range is too wide, increasing minimum');
   wMin = (1e-8)*wMax;
end
wMin = 0.999*wMin;
wMax = 1.001*wMax;

wAll = [];
width = (log10(wMax) - log10(wMin))/nBins;
for k=1:nBins
   wStart = 10^(log10(wMin)+(k-1)*width);
   wEnd = 10^(log10(wMin)+k*width);
   wBin = [];
   lBin = 0;
   for i=1:nA
      wkeep = wA{i}(wA{i}>=wStart & wA{i}<wEnd);
      wBin = union(wBin,wkeep);
      lBin = max(lBin,length(wkeep));
   end
   N2 = lBin;
   if N2>0
      if length(wBin)==1
         V2 = wBin;
      else
         V2 = 10.^(LOCALsamplecdf(log10(wBin),N2));
      end
   else
      V2 = zeros(1,0);
   end
   wAll = [wAll V2']; %#ok<AGROW>
end
            
            
function [Flag1,Flag2,ScalarFlag,CFlag,IFlagW1,IFlagW2,FitFlag,Emsg] = ...
    LOCALMakeFlags(w1Order,w2Order,nW1,nW2)
Emsg = {};
% Check weight orders and set flags:
% O1 = [], scalar (inf, 0, or pos int), or vector (0s or pos ints)
if isempty(w1Order) || (isscalar(w1Order) && all(isinf(w1Order))) || ...
         (isa(w1Order,'double') && ismatrix(w1Order) && all(w1Order>=0) ...
         && min(size(w1Order))==1 && all(ceil(w1Order)==floor(w1Order)) ...
         && ~any(isinf(w1Order)))       
   if isempty(w2Order) || (isscalar(w2Order) && all(isinf(w2Order))) || ...
         (isa(w2Order,'double') && ismatrix(w2Order) && all(w2Order>=0) ...
         && min(size(w2Order))==1 && all(ceil(w2Order)==floor(w2Order)) ...
         && ~any(isinf(w2Order)))

      % Flag = 1 (empty=I), 2 (inf = no fit), 3 (scalar), 4 (vector)      
      if isempty(w1Order)
         Flag1 = 1;
      elseif isinf(w1Order)
         Flag1 = 2;
      elseif isscalar(w1Order)
         Flag1 = 3;
      else
         Flag1 = 4;
      end
      if isempty(w2Order)
         Flag2 = 1;
      elseif isinf(w2Order)
         Flag2 = 2;
      elseif isscalar(w2Order)
         Flag2 = 3;
      else
         Flag2 = 4;
      end
            
      if Flag1==1 && Flag2==1
         Emsg = {'RCT:ucover:NeedOneNonEmptyOrder','Either ORD1 or ORD2 must be nonempty'};
      elseif Flag1==2 && (Flag2==3 || Flag2==4) 
         Emsg = {'RCT:ucover:OrderInfProblem','If ORD1 is inf then ORD2 must be inf or empty'};
      elseif Flag2==2 && (Flag1==3 || Flag1==4) 
         Emsg = {'RCT:ucover:OrderInfProblem','If ORD2 is inf then ORD1 must be inf or empty'};
      elseif Flag1==3 && Flag2==4 && nW1==1 
         Flag1 = 4;   
      elseif Flag2==3 && Flag1==4 && nW2==1 
         Flag2 = 4;
      elseif (Flag1==3 && Flag2==4) || (Flag1==4 && Flag2==3)
         Emsg = {'RCT:ucover:OrderScalarProblem','If either ORD1 or ORD2 is a scalar then the other order must be scalar or empty'};
      end      
   else
      Emsg = {'RCT:ucover:InvalidOrd2','ORD2 must be either empty, inf, or a vector of non-negative integers'};
   end
else
   Emsg = {'RCT:ucover:InvalidOrd1','ORD1 must be either empty, inf, or a vector of non-negative integers'};
end   

ScalarFlag = (Flag1==3 || Flag2==3);
if (Flag1==1 && all(w2Order==0)) ||  (Flag2==1 && all(w1Order==0) ) ...
      || ( all(w1Order==0) && all(w2Order==0) )
   % [EYE,Constant], [Constant,EYE], [Constant,Constant]
   CFlag = 1;
else
   CFlag = 0;
end
IFlagW1 = (Flag1==1);
IFlagW2 = (Flag2==1);
FitFlag = 0;
if (Flag1==3) && (w1Order>0)
   % Scalar, non-constant W1
   FitFlag = 1;
elseif (Flag1==4) && (Flag2==1) && any(w1Order>0)
   % non-scalar W1, some orders > 0, and W2 = EYE
   FitFlag = 1;
elseif (Flag2==3) && (w2Order>0)
   % Scalar, non-constant W2
   FitFlag = 1;
elseif (Flag2==4) && (Flag1==1) && any(w2Order>0)
   % non-scalar W2, some orders > 0, and W1 = EYE
   FitFlag = 1;
elseif (Flag1==4) && (Flag2==4) && ( any(w1Order>0) || any(w2Order>0) ) 
   % non-scalar W1 and nonscalar W2, at least one orders > 0
   FitFlag = 1;
end
if Flag1==4 && nW1~=length(w1Order)
    Emsg = {'RCT:ucover:InvalidType','Dimension of ORD1 does not match PNOM and UTYPE.'};
end
if Flag2==4 && nW2~=length(w2Order)
    Emsg = {'RCT:ucover:InvalidType','Dimension of ORD2 does not match PNOM and UTYPE.'};
end
    
function [type,dsuf,nW1,nW2,Emsg] = LOCALgetType(Types,type,nY,nU)
Emsg = {};
cidx = strncmpi(type,Types,length(type));
if length(find(cidx))~=1
   Emsg = {'RCT:ucover:InvalidType','Invalid TYPE of uncertainty model'};
end
if cidx(1)==1 
   type = 'Additive';
   dsuf = '_Add';
   nW1 = nY;  nW2 = nU;
elseif cidx(2)==1 
   type = 'InputMult';
   dsuf = '_InputMult';
   nW1 = nU;  nW2 = nU;
elseif cidx(3)==1 
   type = 'OutputMult';
   dsuf = '_OutputMult';
   nW1 = nY;  nW2 = nY;
end
function xs = LOCALsamplecdf(p,N)
% P is a sorted list of values.  The histogram of P defines a density, and
% we draw N samples from this density.  The algorithm converts the values
% to a CDF, and then uniformly samples from here (with SPLINE interpolation).
% Result is sorted.
lp = length(p);
y = (1:lp)/lp;
vs = (0.5+(0:N-1))/N;  % gives repeatability, as opposed to sort(rand(N,1))
xs = sort(interp1(y(:),p(:),vs','spline'));

% function wrow = LOCALfrdMat2RowDiag(wmat)
% szM = iosize(wmat);
% minDim = min(szM);
% if minDim>=1
%    wrow = wmat(1,1);
%    for i=2:minDim
%       wrow(1,i) = wmat(i,i);
%    end
% end

function G = LOCALfitmagfrdIteration(data,finalorder,W)
BackOffFactor = 1.2;
[~,F] = frdata(data);
reldeg = [];
C.LowerBound = data;
C.UpperBound = [];
G = fitmagfrd(data,0,reldeg,W,C);
for i=1:finalorder
    C.UpperBound = BackOffFactor*abs(frd(G,F));
    G = fitmagfrd(data,i,reldeg,W,C);
end
G = G * getPeakGain(data/G,1e-3);
