function x = DG2x(DGLMI,Dx,Gx)
% Converts values of compressed D,G into value of decision vector x.
% Used for LMI solver initialization.

%   Copyright 2019 The MathWorks, Inc.

% Complex to real
if DGLMI.cplxDG
   Dx = [real(Dx) imag(Dx);-imag(Dx) real(Dx)];
   Gx = [real(Gx) imag(Gx);-imag(Gx) real(Gx)];
end

% Use MAT2DEC to populate X
if isempty(Gx)
   x = mat2dec(DGLMI.lmisysInit,Dx);
else
   x = mat2dec(DGLMI.lmisysInit,Dx,Gx);
end