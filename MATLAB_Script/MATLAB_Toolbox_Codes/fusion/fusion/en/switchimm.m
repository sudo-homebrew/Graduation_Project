%SWITCHIMM Model conversion function for trackingIMM object
%   x = SWITCHIMM(modelType1, x1, modelType2) converts the State or
%   StateCovariance from modelType1 state definition to modelType2 state definition.
%   modelType1 - specifies the string name of the current motion model.
%   x1         - specifies State or StateCovariance corresponding to modelType1
%   modelType2 - specifies the string name of motion model to which x1 needs to be converted.
%
%   x = SWITCHIMM(...,x2), additionally, lets you specify the size and type of the
%   output. When not specified, x has the same type and dimensionality as x1.
%   x2 specifies State or StateCovariance corresponding to modelType2.
%
%   x1 - a L-by-1 real vector or L-by-L real matrix in modelType1 state space.
%   x  - a M-by-1 real vector or M-by-M real matrix in modelType2 state space.
%
%   Notes:
%   ------
%   1. x2 is an optional input parameter. By default, the function preserve
%   the dimensionality of x1 to convert the state or stateCovariance.
%   2. x2 is a M-by-1 real vector or M-by-M real matrix in modelType2 state
%   space.
%
%   % Example 1: Converting state from constant acceleration to constant
%   % turn with x2 as an input parameter
%   % --------------------------------------------------------------------
%   %   The input parameters for switchimm are
%   modelType1 = 'constacc';
%   modelType2 = 'constturn';
%   x1 = [1;2;3;4;5;6];
%   x2 = [0;0;0;0;0;0;0];
%   x  = switchimm(modelType1,x1,modelType2,x2)
%
%   %   In this case, the model conversion function converts the 2-D
%   %   constant acceleration input to a 3-D constant-turn model output.
%   %   Size and class of x will always be same as that of x2 when x2 is
%   %   given in input parameter.
%
%   % Example 2: Converting state from constant acceleration to constant
%   % velocity without x2 as an input parameter
%   % --------------------------------------------------------------------
%   %   The input parameters for switchimm are
%   modelType1 = 'constacc';
%   modelType2 = 'constvel';
%   x1 = single([1;2;3;4;5;6]);
%   x = switchimm(modelType1,x1,modelType2)
%
%   %   In this case, the output is a 2-D constant velocity state, because
%   %   the input is a 2-D constant acceleration. Class and Dimensionality
%   %   of x will always be same as x1 when x2 is not the input parameter.
%
%   See also: trackingIMM, initekfimm, constvel, constacc, constturn.

 
%   Copyright 2018 The MathWorks, Inc.

