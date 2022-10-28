function [names,values,grouping,instruct] = nvlist(varargin)
%

% Copyright 2003-2004 The MathWorks, Inc.

% XXXMINOR this is an internal routine, but used heavily in USAMPLE and USUBS
% to allow for flexible syntax.  We need to get the error-messages
% improved.
nin = nargin;
pt = 1;
cnt = 1;
names = cell(0,1);
values = cell(0,1);
grouping = zeros(0,1);
instruct = zeros(0,1);
while pt<=nin
   argname = varargin{pt};
   structflag = 0;
   if isa(argname,'cell')
      sza = size(argname);
      if length(sza)==2
         % 1-by-2 of names, should be 2x1
         if isequal(sza,[1 2])
            if isa(argname{2},'char')
               argname = argname';
               argvalue = varargin{pt+1};
               pt = pt + 2;
            else
               argvalue = argname(2); % as a cell;
               argname = argname(1);  % as a cell;
               pt = pt + 1;
            end
         elseif sza(2)==2 % Something-by-2
            argvalue = argname(:,2);
            argname = argname(:,1);
            pt = pt + 1;
         elseif sza(1)==1 % 1-by-something(not 2)
            argname = argname';
            argvalue = varargin{pt+1};
            pt = pt + 2;
         elseif sza(2)==1
            argvalue = varargin{pt+1};
            pt = pt + 2;
         elseif isempty(argname)
            argvalue = varargin{pt+1};
            if isempty(argvalue)
               pt = pt + 2;
            else
               error('Empty Names should come with Empty Values');
            end
         else
            error('Cell argument either be CHAR, or N-by-2');
         end
         %argname = argname(:);
      else
         error('NAMES arguments should be 2-d');
      end
   elseif isa(argname,'struct')
      [argn,argvalue,eflag] = struct2nv(argname);
      structflag = 1;
      bloc = find(eflag>0);
      if length(bloc)>0
         error(['Unable to properly form data associated with ' argn{bloc(1)}]);
      end
%       argn = fieldnames(argname);
%       argvalue = cell(length(argn),1);
%       for i=1:length(argn)
%          [argvalue{i},eflag] = LOCALfield2array(argname,argn{i});
%          switch eflag
%             case 1
%                error(['Unable to properly form data associated with ' argn{i}]);
%             case 2
%                error(['Unable to properly form data associated with ' argn{i}]);
%             case 3
%                error(['Unable to properly form data associated with ' argn{i}]);
%             case 4
%                error(['Invalid data to replace ' argn{i}]);
%          end
%       end
      pt = pt + 1;
      argname = argn;
   else
      if isa(argname,'char')
         argname = cellstr(argname);
      else % XXX not sure whats happening here
         argname = {argname};
      end
      argvalue = varargin{pt+1};
      pt = pt + 2;
   end
   if isa(argvalue,'cell')
      argvalue = argvalue(:);
   elseif isa(argvalue,'char') % Could this happen in our use of this routine?
      argvalue = cellstr(argvalue);
   else
      argvalue = {argvalue};
   end
   szn = size(argname);
   szv = size(argvalue);
   if szv(1)==1 & szv(2)>1
      argvalue = argvalue(:);
      szv = [szv(2) szv(1)];
   end
   if szn(1)==szv(1)
      names = [names;argname];
      values = [values;argvalue];
   elseif szv(1)==1
      names = [names;argname];
      values = [values;repmat(argvalue,[szn(1) 1])];
   else
      error('Incompatible');
   end
   grouping = [grouping;repmat(cnt,[szn(1) 1])];
   instruct = [instruct;repmat(structflag,[szn(1) 1])];
   cnt = cnt + 1;
end

