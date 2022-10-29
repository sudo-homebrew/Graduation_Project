function [GS,GF] = slowfast(G,Ncut)
%SLOWFAST  Slow-fast decomposition.
%
%   [GS,GF] = SLOWFAST(G,CUT) decomposes the LTI model G into the sum of a
%   slow part GS and a fast part GF:
%
%                 G(s) = GS(s) + GF(s)
%
%   The poles of GS have smaller natural frequency than the poles of GF.
%   The integer CUT specifies the order of GS.
%
%   Note: Use FREQSEP to directly specify the cutoff frequency.
%
%   See also FREQSEP, DAMP.

% Copyright 1988-2013 The MathWorks, Inc.
narginchk(2,2)
if ~(isscalar(Ncut) && isnumeric(Ncut) && isreal(Ncut) && Ncut>=0 && Ncut<Inf && rem(Ncut,1)==0)
   error(message('Robust:transformation:slowfast1'))
end

try
   % Hand over to FREQSEP_ code
   % Note: fCut=-Ncut interpreted as cutoff order rather than cutoff frequency
   [GS,GF] = freqsep_(G,-Ncut,freqsepOptions());
catch ME
   if any(strcmp(ME.identifier,{'MATLAB:class:undefinedMethod','MATLAB:UndefinedFunction'}))
      error(message('Control:general:NotSupportedModelsofClass','slowfast',class(G)))
   else
      error(ME.identifier,strrep(ME.message,'freqsep','slowfast'))
   end
end

% Clear notes, userdata, etc
GS.Name = '';  GS.Notes = {};  GS.UserData = [];
GF.Name = '';  GF.Notes = {};  GF.UserData = [];