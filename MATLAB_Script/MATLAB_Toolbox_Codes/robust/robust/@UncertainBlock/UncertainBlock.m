classdef UncertainBlock < ControlDesignBlock
   % Uncertain Control Design blocks.
   %
   %   Uncertain Control Design blocks help model uncertain elements in static 
   %   or dynamic input/output models. You can model uncertain real or complex
   %   parameters or quantify unmodeled system dynamics. The Robust Control
   %   Toolbox provides a variety of commands to analyze how uncertainty 
   %   affects stability and performance.
   % 
   %   All uncertain Control Design blocks derive from the @UncertainBlock 
   %   superclass. This class is not user-facing and cannot be instantiated. 
   %   User-facing subclasses include:
   %      * Uncertain parameters: UREAL, UCOMPLEX, UCOMPLEXM
   %      * Uncertain dynamics: ULTIDYN, UMARGIN, UDYN.
   %
   %   You can combine uncertain Control Design blocks with numeric LTI models 
   %   to construct uncertain models of your control system (see ULTI).
   %
   %   See also UREAL, UCOMPLEX, UCOMPLEXM, ULTIDYN, UMARGIN, UDYN, 
   %   ControlDesignBlock, ULTI, ROBSTAB, ROBGAIN, WCGAIN.
   
%   Copyright 2010-2012 The MathWorks, Inc.
   
   properties (Access = public)
      % Block simplification level [{'basic'} | 'off' | 'full'].
      %
      % This property controls how expressions involving uncertain blocks
      % are simplified. Its default value is 'basic', which means elementary
      % simplification methods are applied after each arithmetic or
      % interconnection operation. Other values are 'off' (no simplification
      % performed) and 'full' (model-reduction-like techniques are applied).
      AutoSimplify = 'basic';
   end
   
   properties (Access = protected)
      % Nominal value
      NominalValue_
   end
   
   methods
      
      function blk = set.AutoSimplify(blk,Value)
         % SET method for obsolete AutoSimplify property
         AS = ltipack.matchKey(Value,{'off','obvious','basic','full'});
         if isempty(AS)
            ctrlMsgUtils.error('Robust:umodel:ublock1')
         else
            if strcmp(AS,'obvious')
               AS = 'basic';
            end
            blk.AutoSimplify = AS;
         end
      end
      
      function M = normalize(blk)
         %NORMALIZE   Normalizes an atom
         %   AN = NORMALIZE(A) returns an uncertain matrix which depends on the
         %   atom, A. AN is normalized in its dependence on A, i.e. as A sweeps
         %   through its range of values and if:
         %     A is a ureal, then AN takes on values in [-1,1].
         %     A is a ucomplex, then AN takes on values in the unit disk.
         %     A is a ultidyn, then AN sweeps out systems norm-bounded by 1.
         %     A is a ucomplexm, then AN sweeps out matrices norm-bounded by 1.
         %
         %   % EXAMPLE 1: (CUT/PASTE)
         %   %  if A is ureal, then AN is normalized to [-1 1]
         %   a = ureal('a',5,'Range',[4 8]);
         %   an = normalize(a);
         %   ansamp=usample(an,100);
         %   [min(ansamp(:)) max(ansamp(:))]
         %
         %   % EXAMPLE 2: (CUT/PASTE)
         %   %  if A is ucomplex, then AN is normalized to the unit disk.
         %   a = ucomplex('a',4+j,'Radius',2.5);
         %   an = normalize(a);
         %   ansamp=usample(an,200);
         %   plot(real(ansamp(:)),imag(ansamp(:)),'bx');
         %   hold on;
         %   x=linspace(-1,1);
         %   plot(x,sqrt(1-x.^2),'r',x,-sqrt(1-x.^2),'r')
         %
         %   % EXAMPLE 3: (CUT/PASTE)
         %   %  if A is ultidyn, then AN sweeps out systems norm-bounded by 1.
         %   a = ultidyn('a',[3 4],'Bound',3);
         %   an = normalize(a);
         %   wcgain(an)
         %
         %   See also LFT
         [R,S] = normalizeBlock(blk);
         M = lft(R,blk-S);
      end
      
   end
   
   
   %% DATA ABSTRACTION INTERFACE
   methods (Access = protected)
      
      blk = uscale_(blk,fact)
      
      function boo = isUncertain_(~)
         boo = true;
      end
      
      function varargout = lftdata_(blk,varargin)
         % Support for uncertain blocks
         % Convert to UMAT or USS
         if isa(blk,'StaticModel')
            M = umat(blk);
         else
            M = uss(blk);
         end
         [varargout{1:nargout}] = lftdata_(M,varargin{:});
      end
      
      function blk = setValue_(blk,NewVal)
         if isa(NewVal,'ControlDesignBlock')
            blk.NominalValue = getValue(NewVal);
         else
            blk.NominalValue = NewVal;
         end
      end

      function Val = getNominal_(blk)
         Val = blk.NominalValue;
      end

   end
      
end
