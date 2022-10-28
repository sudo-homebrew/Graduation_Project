function [N,V,eflag] = struct2nv(A)
% If A is a struct array (array dimension AD), and fn is a field
% of A.  If every value of A(i).(fn) is (say) a DOUBLE array of
% size SZ, then V{i} will be a DOUBLE array of size [SZ AD].

%   Copyright 2003-2010 The MathWorks, Inc.
sza = size(A);
if ~isempty(A)
   N = fieldnames(A);
   V = cell(length(N),1);
   eflag = zeros(length(N),1);
   for i=1:length(N)
      fn = N{i};
      e1 = A(1).(fn);
      % Need to change test since there are no more 'atom's
      if isa(e1,'UncertainBlock')
         e1 = umat(e1);
      end
      se1 = size(e1);
      switch class(e1)
         case 'double'
            try
               tmp = {A.(fn)};
               V{i} = reshape(cat(length(se1)+1,tmp{:}),[se1 sza]);
            catch %#ok<*CTCH>
               V{i} = [];
               eflag(i) = 1;
            end
         case {'ss' 'tf' 'frd' 'zpk'}
            try
               tmp = {A.(fn)};
               V{i} = reshape(stack(length(se1)-2+1,tmp{:}),[se1(3:end) sza]);
            catch
               V{i} = [];
               eflag(i) = 2;
            end
         case {'uss' 'ufrd'}
            try
               tmp = {A.(fn)};
               V{i} = reshape(stack(length(se1)-2+1,tmp{:}),[se1(3:end) sza]);
            catch
               V{i} = [];
               eflag(i) = 3;
            end
         case {'umat'}
            try
               tmp = {A.(fn)};
               % RESHAPE for UMATs works like RESHAPE for USS/UFRD, in
               % previous RCT version RESHAPE for UMATs worked like DOUBLEs
               %V{i} = reshape(stack(length(se1)-2+1,tmp{:}),[se1(1:2) se1(3:end) sza]);
               V{i} = reshape(stack(length(se1)-2+1,tmp{:}),[se1(3:end) sza]);
            catch
               V{i} = [];
               eflag(i) = 3;
            end
         case {'char'}
            if isequal(sza,[1 1])
               V{i} = A.(fn);
            else
               V{i} = [];
               eflag(i) = 5;
            end
         otherwise
            V{i} = [];
            eflag(i) = 4;
      end
   end
else
   N = cell(0,1);
   V = cell(0,1);
   eflag = zeros(0,1);
end

