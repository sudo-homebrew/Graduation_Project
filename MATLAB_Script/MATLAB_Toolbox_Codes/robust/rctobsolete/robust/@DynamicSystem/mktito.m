function sys=mktito(sys,nmeas,ncont)
% SYS=MKTITO(SYS,NMEAS,NCONT) adds TITO (two-input-two-output)
%    partitioning to an LTI system by setting its InputGroup 
%    and OutputGroup properties as follows:
%         [r,c]=size(SYS);
%         sys.InputGroup = struct('U1',1:c-NCONT,'U2',c-NCONT+1:c));
%         sys.OutputGroup = struct('Y1',1:r-NMEAS,'Y2',r-NMEAS+1:r));
%    Any pre-existing partition of SYS is overwritten.
% 
% See also AUGW HINFSYN H2SYN SDHFSYN

% Copyright 1988-2004 The MathWorks, Inc.
[r,c]=size(sys);
if r<nmeas, error('NMEAS exceeds SYS output dimension'), end
if c<ncont, error('NCONT exceeds SYS input dimension'), end
sys.InputGroup = struct('U1', 1:c-ncont, 'U2', c-ncont+1:c);
sys.OutputGroup = struct('Y1', 1:r-nmeas, 'Y2', r-nmeas+1:r);
