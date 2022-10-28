function [a2new,b2new,listnew,dimnew] = icsigbin(a,dima,b,dimb)

% Copyright 2003-2004 The MathWorks, Inc.

na = length(a);
nb = length(b);

comloca = [];
comlocb = [];
locua = [];
locub = 1:nb;

for i=1:na
   idx = strmatch(a{i},b,'exact');
   if length(idx)==1
      if isequal(dima(i),dimb(idx))
         comloca = [comloca i];
         comlocb = [comlocb idx];
         locub(idx) = 0;
      else
         error('Incompatible dimension data')
      end
   elseif isempty(idx)
      locua = [locua i];
   elseif length(idx)>1
      error('Invalid Names in B');
   end
end

tmpub = find(locub==0);
locub(tmpub) = [];

aidx = cumsum([1;dima]);
bidx = cumsum([1;dimb]);
listnew = [a(comloca);a(locua);b(locub)];
dimnew = [dima(comloca);dima(locua);dimb(locub)];
nidx = cumsum([1;dimnew]);

a2new = zeros(sum(dima),sum(dimnew));
b2new = zeros(sum(dimb),sum(dimnew));
bigE = eye(sum(dimnew));

lcom = length(comloca);
lua = length(locua);
lub = length(locub);

for i=1:lcom
    a2new(aidx(comloca(i)):aidx(comloca(i)+1)-1,:) = ...
        bigE(nidx(i):nidx(i+1)-1,:);
    b2new(bidx(comlocb(i)):bidx(comlocb(i)+1)-1,:) = ...
        bigE(nidx(i):nidx(i+1)-1,:);
end
for i=1:lua
%[aidx(locua(i)) aidx(locua(i)+1)-1 nidx(lcom+i) nidx(lcom+i+1)-1]
    a2new(aidx(locua(i)):aidx(locua(i)+1)-1,:) = ...
        bigE(nidx(lcom+i):nidx(lcom+i+1)-1,:);
end
for i=1:lub
    b2new(bidx(locub(i)):bidx(locub(i)+1)-1,:) = ...
        bigE(nidx(lcom+lua+i):nidx(lcom+lua+i+1)-1,:);
end

