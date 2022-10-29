%MUSSVEXTRACT  Extracts the INFO structure returned by MUSSV.
%   [VDelta,VSigma,VLmi] = MUSSVEXTRACT(MUINFO)
%
%   Consider [BNDS,MUINFO] = MUSSV(M,BLK), followed by the extraction
%   command listed above.  The following verifications hold.
%   Verification of Lower Bound:
%     VDelta is a matrix whose structure is given by BLK;
%     NORM(VDelta) equals 1/BND(1,2);
%     EIG(M*VDelta) has at least one eigenvalue exactly at 1; and
%     (equivalently) DET(EYE(SIZE(M,1)) - M*VDelta) is zero.
%
%   Verification of Upper Bound by Newlin/Young Singular Value formula
%     The fields of VSigma are DLeft, DRight, GLeft, GMiddle, GRight;
%     DRight*DELTA equals DELTA*DLeft for all DELTA described by BLK;
%     NORM(LScale*MiddleTerm*RScale) is less than or equal 1, where
%        LScale     = inv(sqrtm(sqrtm(eye(size(M,1))+GLeft*GLeft)))
%        MiddleTerm = (1/BND(1,1))*DLeft*M/DRight - j*GMiddle
%        RScale     = inv(sqrtm(sqrtm(eye(size(M,2))+GRight*GRight)))
%     
%   Verification of Upper Bound by LMI formula
%     The fields of VLmi are Dr, Dc, Grc, Gcr;
%     Dc*DELTA equals DELTA*Dr for all DELTA described by BLK;
%     Grc*DELTA equals DELTA'*Gcr for all DELTA described by BLK;
%     Grc' equals Gcr;
%     M'*Dr*M - (BND(1,1)^2)*Dc + j*(Gcr*M-M'*Grc) is negative semidefinite.        
%
%	 See also: MUSSV

% Copyright 2003-2006 The MathWorks, Inc.


function [delta,sigma,lmi] = mussvextract(muinfo)

nout = nargout;
wantdelta = nout>=0;
wantsigma = nout>=2;
wantlmi = nout>=3;

if wantdelta
   delta = mussvunwrap(muinfo);
end
if wantsigma
   [sigma.DLeft,sigma.DRight,sigma.GLeft,sigma.GMiddle,sigma.GRight] ...
      = mussvunwrap(muinfo);
end
if wantlmi
   [lmi.Dr,lmi.Dc,lmi.Grc,lmi.Gcr] = mussvunwrap(muinfo);
end
