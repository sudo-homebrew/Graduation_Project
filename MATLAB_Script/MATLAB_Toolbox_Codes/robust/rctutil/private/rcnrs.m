function newm = rcnrs(m,idx,lval,rval)
%   Recenter and normalize all real scalars (sreal and repreal).  the vectors
%   LVAL and RVAL are [srealLeft reprealLeft], and [srealRight reprealRight]

% Copyright 2003-2004 The MathWorks, Inc.

    newm = m;
    for i=1:idx.sreal.num
        center = 0.5*(lval(i)+rval(i));
        radius = 0.5*(rval(i)-lval(i));
        newm = oneloopi(newm,idx.sreal.rows{i},idx.sreal.cols{i},center);
        newm(idx.sreal.rows{i},:) = sqrt(radius)*newm(idx.sreal.rows{i},:);
        newm(:,idx.sreal.cols{i}) = sqrt(radius)*newm(:,idx.sreal.cols{i});
    end
    nsr = idx.sreal.num;
    for i=1:idx.repreal.num
        center = 0.5*(lval(nsr+i)+rval(nsr+i));
        radius = 0.5*(rval(nsr+i)-lval(nsr+i));
        newm = oneloopi(newm,idx.repreal.rows{i},idx.repreal.cols{i},center);
        newm(idx.repreal.rows{i},:) = sqrt(radius)*newm(idx.repreal.rows{i},:);
        newm(:,idx.repreal.cols{i}) = sqrt(radius)*newm(:,idx.repreal.cols{i});
    end
        
