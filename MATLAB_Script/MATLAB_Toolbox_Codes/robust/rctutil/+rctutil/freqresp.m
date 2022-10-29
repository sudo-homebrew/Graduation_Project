function h = freqresp(a,b,c,d,Ts,freqs)
% Fast frequency response evaluation at a few points.

%   Copyright 2004-2015 The MathWorks, Inc.
nf = numel(freqs);
[ny,nu] = size(d);
nx = size(a,1);
h = zeros(ny,nu,nf);
if Ts>0
   freqs = Ts*freqs;
   freqs = complex(cos(freqs),sin(freqs));
else
   freqs = complex(0,freqs);
end
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
for ct=1:nf
   s = freqs(ct);
   if isinf(s)
      h(:,:,ct) = d;
   else
      if ny>nu
         fresp = d + c * ((s*eye(nx)-a)\b);
      else
         fresp = d + (c/(s*eye(nx)-a)) * b;
      end
      if all(isfinite(fresp),'all')
         h(:,:,ct) = fresp;
      else
         h(:,:,ct) = Inf;
      end
   end
end
