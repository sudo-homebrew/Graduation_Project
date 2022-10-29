function UBcert = insertUB(UBcert,UBnew)
% Inserts new interval bound into cover of Focus interval.

%   Copyright 1986-2016 The MathWorks, Inc.

% Quick exit if ptUB=Inf (FV portion of the LMI does not hold)
if isinf(UBnew.ptUB)
   UBcert = UBnew;   return
end

% Note: UBnew.gUB is applied only to overlapping intervals with
% gUB=Inf. This ensures gUBmax is monotonically increasing and 
% Jump=true at the peak frequency.
nint = numel(UBcert);
wIntervals = cat(1,UBcert.Interval);
wBP = [wIntervals(:,1) ; wIntervals(nint,2)];

% Locate new breakpoints
wNew = UBnew.Interval;
gUBnew = UBnew.gUB;
iL = find(wBP<=wNew(1),1,'last');
iR = find(wBP>=wNew(2),1,'first');

% Quick exit
if iR==1
   UBcert = cat(1,UBnew,UBcert);  return
elseif iL==nint+1
   UBcert = cat(1,UBcert,UBnew);  return
end
UBmod = UBcert(iL:iR-1);  % modified section

% Insert new breakpoints
% Note: Cache UBmod at end points for proper handling of NMOD=1
NewL = [];  NewR = [];
if wNew(1)>wBP(iL) && isinf(UBmod(1).gUB)
   % Split first interval
   NewL = UBmod(1);  NewL.Interval = [wBP(iL) wNew(1)];
   UBmod(1).Interval = [wNew(1) wBP(iL+1)];
   wBP(iL) = wNew(1); % for proper handling of NMOD=1
end
if wNew(2)<wBP(iR) && isinf(UBmod(end).gUB)
   % Split last interval
   NewR = UBmod(end);  NewR.Interval = [wNew(2) wBP(iR)];
   UBmod(end).Interval = [wBP(iR-1) wNew(2)];
end

% Use gUBnew for covered intervals with gUB=Inf
for ct=1:numel(UBmod)
   if isinf(UBmod(ct).gUB)
      UBmod(ct).gUB = gUBnew;
      UBmod(ct).VLmi = UBnew.VLmi;
   end
end

% Insert pointwise bounds
UBmod = cat(1,NewL,UBmod,NewR);
wBP = cat(1,UBmod.Interval);
ipt = find(UBnew.w>=wBP(:,1),1,'last');
if ~isempty(ipt)
   UBmod(ipt).w = UBnew.w;
   UBmod(ipt).ptUB = UBnew.ptUB;
   UBmod(ipt).Jump = UBnew.Jump;
end
      
% Update cover
UBcert = cat(1,UBcert(1:iL-1,:),UBmod,UBcert(iR:nint,:));