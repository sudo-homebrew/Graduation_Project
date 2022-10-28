classdef ureal < UncertainBlock & StaticModel
   %UREAL  Creates uncertain real scalar.
   %
   %   A = UREAL(NAME,NOMINAL) creates an uncertain real parameter ranging  
   %   in the interval [NOMINAL-1,NOMINAL+1]. The string NAME specifies the 
   %   parameter name and the scalar NOMINAL specifies its nominal value.
   %   The resulting object A is of class "ureal".
   % 
   %   A = UREAL(NAME,NOMINAL,'PlusMinus',[-DL,DR]) creates an uncertain 
   %   parameter taking values in the interval [NOMINAL-DL,NOMINAL+DR]. 
   %   The deviations DL and DR must be positive scalars. When the interval  
   %   is centered at the nominal value (DL=DR), you can specify the 
   %   uncertainty level with a single value, for example,
   %      A = ureal('a',2,'PlusMinus',1.5)  % 2 +/- 1.5
   % 
   %   A = UREAL(NAME,NOMINAL,'Range',[LOW,HIGH]) specifies the uncertainty 
   %   level as an interval [LOW,HIGH]. The nominal value should lie in this
   %   interval.
   % 
   %   A = UREAL(NAME,NOMINAL,'Percentage',[-PL,PR]) specifies the uncertainty 
   %   level as percentage deviations from the nominal value:
   %      PL = 100 * |1-LOW/NOMINAL| ,    PR = 100 * |1-HIGH/NOMINAL|
   %   When PL=PR, you can specify the percentage uncertainty with a single 
   %   value, for example,
   %      A = ureal('a',2,'Percentage',10)  % 2 +/- 10%
   %
   %   Use the "Range", "PlusMinus", or "Percentage" properties of A to 
   %   query the uncertainty level as an interval, absolute deviation, or
   %   percentage deviation. In addition, use the "Mode" property to specify 
   %   which of these three uncertainty quantifications is independent of
   %   the nominal value. For example, if A.Mode='Range', then changing the 
   %   nominal value has no effect on A.Range but affects both A.PlusMinus 
   %   and A.Percentage. Note that "Mode" is initialized based on how you
   %   specify the uncertainty level, for example, 
   %      A = ureal('a',2)
   %   initializes A.Mode to 'PlusMinus' while
   %      A = ureal('a',1,'Percentage',[-10 20])
   %   initializes A.Mode to 'Percentage'.
   %
   %   Use the "AutoSimplify" property to control how expressions involving 
   %   uncertain blocks are simplified, type "help ureal.AutoSimplify" for
   %   details.
   %
   %   Examples:
   %      % Create a UREAL with name 'a' and nominal value 4:
   %      a = ureal('a',4);
   %      % The default uncertainty range is 4 plus/minus 1:
   %      a.PlusMinus
   %      % The corresponding percentage uncertainty is plus/minus 25%:
   %      a.Percentage
   %
   %      % Create a UREAL with name 'b', nominal value 5, and range [2 6].
   %      b = ureal('b',5,'Range',[2 6]);
   %      % Note that "Mode" is automatically set to 'Range':
   %      b.Mode
   %
   %      % Create a UREAL with a 25% uncertainty centered around the 
   %      % nominal value:
   %      c = ureal('c',4,'Percentage',25);
   %      % The corresponding interval is [3,5]:
   %      c.Range
   %      % To modify the nominal value to 4.5 while retaining the same [3,5]
   %      % range, force "Mode" to 'Range' and then set the nominal value:
   %      c.Mode = 'Range'
   %      c.NominalValue = 4.5
   %
   %   Note: When the range is not centered at the nominal value, there are
   %   additional restrictions on what values UREAL parameters can take,
   %   see getLimits for details.
   %
   %   See also getLimits, UCOMPLEX, ULTIDYN, UMAT, CONTROLDESIGNBLOCK.
   
%   Author(s): Andy Packard, Gary Balas, P. Gahinet
%   Copyright 1986-2017 The MathWorks, Inc.
      
   properties (Access = public, Dependent)   
      % Nominal value (scalar).
      NominalValue
      % Uncertainty quantification mode.
      Mode
      % Uncertainty range as an interval.
      Range
      % Uncertainty range as absolute deviations from nominal value. 
      PlusMinus
      % Uncertainty range as percentage deviations from nominal value. 
      Percentage
   end
   
   properties (Access = protected)
      % Storage properties (uncoupled)
      % Uncertainty quantification mode ([] or string)
      Mode_ = 'PlusMinus';
      % Uncertainty level as absolute deviations from nominal value
      PlusMinus_
   end
   
   % TYPE MANAGEMENT IN BINARY OPERATIONS
   methods (Static, Hidden)
      
      function boo = isClosed(~)
         boo = false;
      end
      
      function T = toClosed(~)
         T = 'umat';
      end
      % Note: getAttributes never called (first converted to UMAT)      
   end
    
   %% PUBLIC METHODS
   methods
      
      function blk = ureal(name,value,varargin)
         ni = nargin;
         if rem(ni,2)~=0
            error(message('Robust:umodel:ureal1'))
         elseif ni==0
            name = 'UNNAMED';  value = 0;
         end
         blk.PlusMinus_ = [-1 1];
         blk.IOSize_ = [1 1];
         try
            blk.Name = name;
            blk.NominalValue = value;
            % Process additional inputs.
            % If MODE is not explicitly set, initialize it consistently
            % with how the uncertainty level is specified
            ltipack.mustBeNameValuePairs(varargin)
            PropNames = lower(string(varargin(1:2:ni-2)));
            if ~any(startsWith(PropNames,"m"))
               idx = find(startsWith(PropNames,"p") | startsWith(PropNames,"r"),1,'last');
               if ~isempty(idx)
                  varargin = [varargin , {'Mode'} varargin(2*idx-1)];
               end
            end
            if ~isempty(varargin)
               blk = set(blk,varargin{:});
            end
         catch ME
            throw(ME)
         end
      end
      
      function Value = get.NominalValue(blk)
         % GET method for NominalValue property
         Value = blk.NominalValue_;
      end

      function Value = get.Mode(blk)
         % GET method for Mode property
         Value = blk.Mode_;
      end
      
      function Value = get.PlusMinus(blk)
         % GET method for PlusMinus property
         Value = blk.PlusMinus_;
      end
               
      function Value = get.Percentage(blk)
         % GET method for Percentage property
         Value = blk.PlusMinus_ * (100/abs(blk.NominalValue_));
      end
      
      function Value = get.Range(blk)
         % GET method for Range property
         Value = blk.NominalValue_ + blk.PlusMinus_;
      end
      
      function blk = set.Mode(blk,Value)
         % SET method for Name property
         UMode = ltipack.matchKey(Value,{'PlusMinus','Percentage','Range'});
         if isempty(UMode)
            error(message('Robust:umodel:ureal11'))
         elseif blk.NominalValue_==0 && strcmp(UMode,'Percentage')
            error(message('Robust:umodel:ublock2'))
         else
            blk.Mode_ = UMode;
         end
      end
      
      function blk = set.NominalValue(blk,Value)
         % SET method for NominalValue property
         if ~(isnumeric(Value) && isreal(Value) && isscalar(Value) && isfinite(Value))
            error(message('Robust:umodel:ureal2'))
         end
         Value = full(double(Value));
         switch blk.Mode
            case 'Range'
               % Hold range constant
               blk.PlusMinus_ = blk.PlusMinus_ + (blk.NominalValue_-Value);
               if (blk.PlusMinus_(1)>=0 || blk.PlusMinus_(2)<=0)
                  error(message('Robust:umodel:ureal3'))
               end
            case 'Percentage'
               % Hold percentage constant
               if Value==0
                  error(message('Robust:umodel:ublock3'))
               end
               blk.PlusMinus_ = blk.PlusMinus_ * abs(Value/blk.NominalValue_);
         end
         blk.NominalValue_ = Value;
      end
      
      function blk = set.PlusMinus(blk,Value)
         % SET method for PlusMinus property
         Value = localValidateRange(Value,'PlusMinus');
         blk.PlusMinus_ = reshape(Value,[1 2]);
      end
      
      function blk = set.Percentage(blk,Value)
         % SET method for Percentage property
         Value = localValidateRange(Value,'Percentage');
         if blk.NominalValue_==0
            error(message('Robust:umodel:ublock4'))
         end
         blk.PlusMinus_ = Value * abs(blk.NominalValue_/100);
      end
      
      function blk = set.Range(blk,Value)
         % SET method for Range property
         if ~(isnumeric(Value) && isreal(Value) && numel(Value)==2 && ...
               all(isfinite(Value)) && Value(1)<Value(2))
            error(message('Robust:umodel:ureal10'))
         end
         Value = full(double(Value));
         if prod(Value-blk.NominalValue_)>=0
            warning(message('Robust:umodel:ureal9'))
            blk.NominalValue_ = (Value(1)+Value(2))/2;
         end
         blk.PlusMinus_ = Value-blk.NominalValue_;
      end
      
   end
   
   %% ABSTRACT SUPERCLASS INTERFACES
   methods (Access=protected)

      function displaySize(~,~)
         % Display for "size(M)"
         disp(getString(message('Robust:umodel:SizeUREAL')))
      end

      function blk = uscale_(blk,fact)
         % Scales normalized uncertainty level
         % Note: For asymmetric ranges, transformation T has a discontinuity  
         % at T22*N=1 so need |T(2,2)*fact|<1 for the scaled range to be
         % connected.
         [~,~,T] = normalizeBlock(blk);
         if abs(T(2,2))*fact>1-10*eps
            error(message('Robust:umodel:ureal4',blk.Name,sprintf('%.6g',1/abs(T(2,2)))))
         end
         NR = [-fact fact];  % scaled normalized range
         blk.PlusMinus_ = sort((T(1,2)*T(2,1)) * (NR ./ (1-T(2,2)*NR)));
      end
      
   end
   
   %% DATA ABSTRACTION INTERFACE
   methods (Access=protected)
      
      %% INDEXING
      function M = createLHS(~)
         % Creates LHS in assignment.
         % Returns 0x0 UMAT 
         M = umat();
      end

      %% TRANSFORMATIONS
      function M = repmat_(blk,s)
         M = repmat_(umat(blk),s);
      end
      
      function M = uminus_(blk)
         M = uminus_(umat(blk));
      end
               
   end
   
   
   %% HIDDEN INTERFACES
   methods (Hidden)
      
      function isC = iscentered(blk)
         %ISCENTERED  Checks if nominal value is centered in the range.
         %
         %   ISCENTERED(A) returns TRUE if the nominal value of the UREAL
         %   A is the center of the uncertainty range and FALSE otherwise.
         %
         %   See also UREAL.
         pm = blk.PlusMinus_;
         isC = (abs(pm(1)+pm(2))<1e-8*pm(2));
      end
      
      % CONTROLDESIGNBLOCK
      function Offset = getOffset(blk)
         % Default value is the nominal value
         Offset = blk.NominalValue_;
      end
      
      function D = ltipack_ssdata(blk,varargin)
         % Converts to ltipack.ssdata object. When R,S are supplied,
         % computes blk-S if R=[] and lft(R,blk-S) otherwise.
         d = numeric_array(blk,varargin{:});
         D = ltipack.ssdata([],zeros(0,1),zeros(1,0),d,[],0);
      end
      
      function D = ltipack_frddata(blk,freq,varargin)
         % Converts to ltipack.frddata object
         d = numeric_array(blk,varargin{:});
         D = ltipack.frddata(repmat(d,[1 1 length(freq)]),freq,0);
      end
      
      function M = numeric_array(blk,R,S)
         % Converts to double scalar. When R,S are supplied,
         % computes blk-S if R=[] and lft(R,blk-S) otherwise.
         M = blk.NominalValue_;
         if nargin>1
            M = M-S;
            if ~(isempty(R) || M==0)
               M = R(1,1)+R(1,2)*R(2,1)*M/(1-M*R(2,2));
            end
         end
      end
      
      function str = getDescription(blk,ncopies)
         % Short description for block summary in LFT model display
         Nominal = sprintf('%.3g',blk.NominalValue_);
         switch blk.Mode_
            case 'PlusMinus'
               PM = blk.PlusMinus_;
               str = getString(message('Robust:umodel:ureal16',...
                  blk.Name,Nominal,...
                  sprintf('[%.3g,%.3g]',PM(1),PM(2)),ncopies));
            case 'Range'
               R = blk.Range;
               str = getString(message('Robust:umodel:ureal15',...
                  blk.Name,Nominal,...
                  sprintf('[%.3g,%.3g]',R(1),R(2)),ncopies));
            case 'Percentage'
               P = blk.Percentage;
               str = getString(message('Robust:umodel:ureal17',...
                  blk.Name,Nominal,...
                  sprintf('[%.3g,%.3g]',P(1),P(2)),ncopies));
         end
      end
      
      % OPTIMIZATION
      function ns = numState(~)
         % Size of A matrix from p2ss
         ns = 0;
      end

      function [a,b,c,d] = p2ss(~,p)
         % Constructs realization A(p),B(p),C(p),D(p) from parameter vector p
         a = [];  b = zeros(0,1);  c = zeros(1,0);  d = p;
      end
      
      function gj = gradUV(~,~,u,v,~)
         % Computes the gradient of the inner product
         %    phi(p) = Re(Trace(U'*[A(p) B(p);C(p) D(p)]*V))
         % with respect to the block parameters p(j) where j is a vector
         % of indices. The real or complex matrices U and V must have the
         % same number of columns.
         Gm = real(u*v');
         gj = Gm(:);
      end
      
   end
   
   
   %% PROTECTED METHODS
   methods (Access = protected)
      
      function M = subparen(blk,indices)
         % Indexing forces conversion to UMAT
         M = subparen(umat(blk),indices);
      end
      
   end
   
   
   %% UTILITIES
   methods (Hidden)
      
      function [R,S,T] = normalizeBlock(blk)
         % Computes R,S,T such that
         %    * LFT(R,blk-S) is normalized
         %    * blk = LFT(T,LFT(R,blk-S))
         % Returns R=[] if blk-S is already normalized (then blk = LFT(T,blk-S))
         S = blk.NominalValue_;
         D = abs(blk.PlusMinus_);
         if all(abs(D-1)<100*eps)
            R = [];   T = [S 1;1 0];
         else
            rD = 1./D;
            alpha = (rD(1)+rD(2))/2;  % (DL+DR)/(2*DL*DR)
            beta = (rD(1)-rD(2))/2;   % (DR-DL)/(2*DL*DR)
            aux = sqrt(alpha);
            R = [0 aux;aux -beta];
            aux = 1/aux;
            T = [S aux;aux beta/alpha];
         end
      end
      
      function Aval = norm2act(blk,Nval)
         % Converts normalized values to actual block values. The transformation
         % is A = T11 + T12*T21*N/(1-T22*N) where T11=S is the nominal value. 
         % For asymmetric ranges, this transformation has a discontinuity at 
         % T22*N=1 and is not monotonic over the whole real line.
         if ~(isnumeric(Nval) && isreal(Nval))
            error(message('Robust:umodel:norm2act1'))
         end
         [~,~,T] = normalizeBlock(blk);
         Nval = double(Nval);
         Aval = T(1,1) + (T(1,2)*T(2,1)) * (Nval ./ (1-T(2,2)*Nval));
         if T(2,2)~=0
            Aval(isinf(Nval)) = T(1,1) - T(1,2)*T(2,1)/T(2,2);
         end
      end
      
      function [Nval,Ndist] = act2norm(blk,Aval)
         % Converts actual block values to normalized block values. The 
         % transformation is N = R12*R21*(A-S)/(1-R22*(A-S)) where S is
         % the nominal value. For asymmetric ranges, this transformation 
         % has a discontinuity at R22*(A-S)=1 and is not monotonic over 
         % the whole real line.
         if ~(isnumeric(Aval) && isreal(Aval))
            error(message('Robust:umodel:act2norm1'))
         end
         [R,S] = normalizeBlock(blk);
         Nval = double(Aval) - S;
         if ~isempty(R)
            Nval = (R(1,2)*R(2,1)) * (Nval ./ (1-R(2,2)*Nval));
            if R(2,2)~=0
               Nval(isinf(Aval)) = -R(1,2)*R(2,1)/R(2,2);
            end
         end
         if nargout>1
            Ndist = abs(Nval);
         end
      end
      
      function CS = randSample_(blk,N)
         % Randomly samples uncertain real variable. Returns a cell array
         % of scalar double values.
         % Note: Uniformly sample the actual range of the parameter, 
         %       see g1949540 for rationale
         range = blk.NominalValue_ + blk.PlusMinus_;
         CS = num2cell(range(1) + (range(2)-range(1)) * rand(N,1));
      end
      
      function blk = getNormalizedForm(blk)
         % Returns normalized version of block
         blk = ureal(sprintf('%sNormalized',blk.Name),0);
      end
                  
   end
   
   
   %% STATIC METHODS
   methods(Static, Hidden)
      
      blk = loadobj(s)
      
   end
   
end

%--------------------------------------------
function Value = localValidateRange(Value,Prop)
% Validates values for PlusMinus and Percentage
if ~(isnumeric(Value) && isreal(Value) && any(numel(Value)==[1 2]) && all(isfinite(Value)))
   error(message('Robust:umodel:ureal5',Prop))
elseif isscalar(Value)
   % Treat -X and X as the same spec
   if Value==0
      error(message('Robust:umodel:ureal8',Prop))
   end
   Value = abs(Value) * [-1,1];
elseif Value(1)>=0 || Value(2)<=0
   error(message('Robust:umodel:ureal7',Prop))
end
Value = full(double(Value));
end
