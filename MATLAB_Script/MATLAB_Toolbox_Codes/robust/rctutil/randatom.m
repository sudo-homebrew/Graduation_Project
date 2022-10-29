function out = randatom(type,sz)
%RANDATOM  Generates random uncertain ATOM.
%
%  OUT = RANDATOM(TYPE) generates a 1-by-1 uncertain object of type TYPE.
%  Valid types of uncertain objects are 'ureal', 'ultidyn', 'ucomplex', and
%  'ucomplexm'.
%
%  OUT = RANDATOM(TYPE,SZ) generates a SZ(1)-by-SZ(2) uncertain object 
%  of type 'ultidyn' or 'ucomplexm'. If TYPE is 'ureal' or 'ucomplex' then
%  OUT is a 1-by-1 uncertain object.
%
%  If RANDATOM is called with no input arguments, a 1-by-1 uncertain object
%  whose type is randomly selected between 'ureal', 'ultidyn' and
%  'ucomplex'.
%
%  See also RAND, RANDN, RANDUMAT, RANDUSS, UCOMPLEX, ULTIDYN, UREAL, 
%           UCOMPLEXM.

% Copyright 2003-2004 The MathWorks, Inc.

if nargin==0
   type = [];
   sz = [1 1];
elseif nargin==1
   sz = [1 1];
end
type = convertStringsToChars(type);

if strcmp(type,'ucomplexm') || strcmp(type,'ultidyn')
   if isa(sz,'double') && length(sz)==2 && all(sz>0) && all(floor(sz)==ceil(sz))
      % NU is an integer
   else
      error('SZ must be 1-by-2 vector of positive integers')
   end
end

if isempty(type)
   ltype = {'ureal' 'ucomplex' 'ultidyn'};
   lr = ceil(3*rand);
   type = ltype{lr};
end
switch type
   case 'ureal'
      modech = {'PlusMinus' 'Range' 'Percentage'};
      asch = {'off' 'basic' 'full'};
      mr = ceil(3*rand);
      ar = ceil(3*rand);
      mode = modech{mr};
      autos = asch{ar};
      nom = 5*randn;
      lstart = 65;
      name = char(lstart+floor(26*rand(1,5*ceil(rand))));
      if abs(rand)<.1
         nom = 0;
         mr = ceil(2*rand);
         mode = modech{mr};
      end
      switch mr
         case 1
            pm = [-4*rand 5*rand];
            out = ureal(name,nom,'PlusMinus',pm,'AutoSimplify',autos);
         case 2
             pm = [-4*rand 5*rand];
            out = ureal(name,nom,'Range',nom+pm,'AutoSimplify',autos);
        case 3
            perc = [-20*rand 25*rand];
            out = ureal(name,nom,'Percentage',perc,'AutoSimplify',autos);
       end
   case 'ucomplex'
      modech = {'Radius' 'Percentage'};
      asch = {'off' 'basic' 'full'};
      mr = ceil(2*rand);
      ar = ceil(3*rand);
      mode = modech{mr};
      autos = asch{ar};
      nom = 5*(randn + j*randn);
      lstart = 65;
      name = char(lstart+floor(26*rand(1,5*ceil(rand))));
      if abs(rand)<.1
         nom = 0;
         mr = 1;
         mode = 'Radius';
      end
      switch mr
         case 1 %radius
            pm = abs(2*rand);
            out = ucomplex(name,nom,'Radius',pm,'AutoSimplify',autos);
         case 2
            pm = abs(30*rand);
            out = ucomplex(name,nom,'Percentage',pm,'AutoSimplify',autos);
       end
   case 'ucomplexm'
      asch = {'off' 'basic' 'full'};
      ar = ceil(3*rand);
      autos = asch{ar};
      nom = 5*(randn + j*randn);
      lstart = 65;
      name = char(lstart+floor(26*rand(1,5*ceil(rand))));
      nom = randn(sz(1),sz(2)) + j*randn(sz(1),sz(2));
      WL = randn(sz(1),sz(1)) + j*randn(sz(1),sz(1));
      WR = randn(sz(2),sz(2)) + j*randn(sz(2),sz(2));
      out = ucomplexm(name,nom,'WL',WL,'WR',WR,'AutoSimplify',autos);
   case 'ultidyn'
      modech = {'GainBounded' 'PositiveReal'};
      asch = {'off' 'basic' 'full'};
      mr = ceil(2*rand);
      ar = ceil(3*rand);
      if sz(1)~=sz(2)
         mr = 1;
      end
      mode = modech{mr};
      autos = asch{ar};
      bnd = 4*abs(rand);
      lstart = 65;
      name = char(lstart+floor(26*rand(1,5*ceil(rand))));
      ssdim = ceil(5*rand);
      out = ultidyn(name,sz,'Type',mode,'AutoSimplify',autos,'SampleStateDim',ssdim,...
                     'Bound',bnd);
   otherwise
      error(['''' type ''' is an invalid uncertain atom.']);
end
