function M = usubs(M,varargin)
%USUBS   Substitutes values for uncertain elements.
%
%   B = USUBS(A,NAME1,VALUE1,NAME2,VALUE2,...) instantiates the uncertain 
%   elements NAME1, NAME2,... of A to the values VALUE1, VALUE2,...
%   Set VALUE to 'NominalValue' or 'Random' to use the nominal value or
%   a random instance of a particular element. USUBS works on uncertain 
%   matrices, models, and elements (see UMAT, USS, UFRD, UREAL,...).  
%
%   B = USUBS(A,S) instantiates the uncertain elements of A to the values 
%   specified in the structure S. This structure typically comes from 
%   commands like USAMPLE, WCGAIN, ROBSTAB, etc. 
%
%   B = USUBS(A,...,'-once') performs vectorized substitution in arrays A   
%   of uncertain models. Each uncertain element is replaced by a single 
%   value, but this value may change across the model array. Use a struct 
%   array S or cell arrays VALUE1,VALUE2,... to specify substitute values 
%   for each model in A. For example, if A is a 2x3 array of models, then
%     * A 2x3 struct array S specifies one set of substitute values S(k) 
%       for each model A(:,:,k)
%     * A 2x3 cell array VALUE1 causes the uncertain element NAME1 to be 
%       instantiated to VALUE1{k} in A(:,:,k).
%   Numeric array formats are also accepted for VALUE1,VALUE2,... For 
%   example, VALUE1 can be a 2x3 array of LTI models, a numeric array of  
%   size [SIZE(NAME1) 2 3], or a 2x3 matrix when the uncertain element 
%   NAME1 is scalar-valued. The array sizes of A,S,VALUE1,VALUE2,... must 
%   agree along non-singleton dimensions and scalar expansion takes place 
%   along singleton dimensions.
%
%   B = USUBS(A,...,'-batch') performs batch substitution in arrays A of 
%   uncertain models. Each uncertain element is replaced by an array of 
%   values, and the same values are used for all models in A. This returns
%   a model array B of size [SIZE(A) VS] where VS is the size of the array 
%   of substitute values. 
%
%   Note that USUBS(A,...,'-once') and USUBS(A,...,'-batch') produce the  
%   same result for a single model A and that '-once' is the default 
%   behavior when no flag is specified.
%
%   Example 1:
%      % Create uncertain matrix M = [a b] with uncertain elements a,b
%      a = ureal('a',5);  b = ureal('b',-3);
%      M = [a b];
%      % Evaluate m for 4 different (a,b) combinations:
%      Mab = usubs(M, 'a',[1;2;3;4], 'b',[10;11;12;13])
%      % This returns a 1x2x4 DOUBLE array (4 values of M)
%
%   Example 2:
%      % Evaluate M = [a b;0 a*b] over a 3x4 grid of (a,b) values
%      M = [a b;0 a*b];  % 2-by-2 UMAT
%      aval = rand(3,1);  bval = rand(4,1);
%      % Build arrays of (a,b) values
%      [A,B] = ndgrid(aval,bval);
%      Mab = usubs(M,'a',A,'b',B)    
%      % This returns a 1x2x3x4 DOUBLE array (3x4 array of M values)
%
%   Example 3: 
%      % Instantiate a to the values 1,2,3,4 in M = [a b]
%      M = [a b];
%      Ma = usubs(M,'a',[1;2;3;4])  % 1x2x4 UMAT dependent on b
%      % For each model in the resulting UMAT array, evaluate b at 10,20,30:
%      Mab = usubs(Ma,'b',[10;20;30],'-batch')  
%      % This returns a 1x2x4x3 DOUBLE array (3x4 array of M values) and
%      % is equivalent to 
%      Mab = usubs(M,'a',[1;2;3;4],'b',[10 20 30])  
%
%   See also USAMPLE, REPLACEBLOCK, UMAT, USS, UFRD.

%   Copyright 2003-2011 The MathWorks, Inc.
try
   M = replaceBlock(M,varargin{:});
catch ME
   if strcmp(ME.identifier,'Control:general:InvalidSyntaxForCommand')
      ctrlMsgUtils.error('Control:general:InvalidSyntaxForCommand','usubs','usubs')
   else
      throw(ME)
   end
end
