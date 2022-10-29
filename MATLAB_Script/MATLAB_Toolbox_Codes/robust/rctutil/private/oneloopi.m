function out = oneloopi(m,ridx,cidx,delta)
%
%   Close (with open) around the RIDX rows and CIDX columns
%   of M, with DELTA.  Implicitly assumes (code may be wrong for
%   other situations) that RIDX and CIDX are each vectors of
%   contiguous integers.  DELTA may be a scalar, or should be the
%   correct dimension.

% Copyright 2003-2004 The MathWorks, Inc.

szm = size(m);
npts = prod(szm(3:end));
ne2 = length(ridx);
%ne1 = ridx(1)-1;
%nd1 = cidx(1)-1;
%ne2 = ridx(end)-ne1;
%nd2 = cidx(end)-nd1;
        
out = m;
if ~isempty(ridx) & ~isempty(cidx)
   for i=1:npts
      out(ridx,:,i) = zeros(ne2,szm(2));
      left = [m(1:ridx(1)-1,cidx,i)*delta;eye(ne2);m(ridx(end)+1:szm(1),cidx,i)*delta];
      mid = eye(ne2) - m(ridx,cidx,i)*delta;
      out(:,:,i) = out(:,:,i) + (left/mid)*m(ridx,:,i);
   end
end
    
    
