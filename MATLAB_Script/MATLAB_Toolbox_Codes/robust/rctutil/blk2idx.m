function index = blk2idx(blk)
%

% Copyright 2003-2004 The MathWorks, Inc.

    siz = size(blk);
    if siz(2) ~= 3
        error('Incorrect column dimension of block structure');
    end
    
    colloc = 1;
    rowloc = 1;
    nblk = siz(1);
    rpertdim = blk(:,1).*abs(blk(:,3));     % rows of DELTA, cols of M
    cpertdim = blk(:,2).*abs(blk(:,3));     % cols of DELTA, rows of M
    index.rdimm = sum(cpertdim);            % rows of M (cols of DELTA)
    index.cdimm = sum(rpertdim);            % cols of M (rows of DELTA)
    index.cidxm = cumsum([1;rpertdim]);     % cols of M (rows of DELTA)
    index.ridxm = cumsum([1;cpertdim]);     % rows of M (cols of DELTA)
    index.simpleblk = zeros(0,2);


    % ALL ROWS/COLS refer to M        
    index.repfull.num = 0;
    index.repfull.rows = {};
    index.repfull.cols = {};
    index.repfull.dim = zeros(0,2);
    index.repfull.origloc = zeros(0,1);
    index.repfull.repeated = zeros(0,1);
    
    index.full.num = 0;
    index.full.rows = {};
    index.full.cols = {};
    index.full.dim = zeros(0,2);
    index.full.origloc = zeros(0,1);
    
    index.sreal.num = 0;
    index.sreal.rows = {};
    index.sreal.cols = {};
    index.sreal.origloc = zeros(0,1);

    index.repcomp.num = 0;
    index.repcomp.rows = {};
    index.repcomp.cols = {};
    index.repcomp.origloc = zeros(0,1);
    index.repcomp.repeated = zeros(0,1);

    index.repreal.num = 0;
    index.repreal.rows = {};
    index.repreal.cols = {};
    index.repreal.origloc = zeros(0,1);
    index.repreal.repeated = zeros(0,1);
    
    for i = 1:nblk
        if blk(i,1)*blk(i,2) == 1   % scalar
            if blk(i,3) > 1         % complex , repeated
                index.repcomp.num = index.repcomp.num + 1;
                index.repcomp.rows{index.repcomp.num} = rowloc:rowloc+blk(i,3)-1;
                index.repcomp.cols{index.repcomp.num} = colloc:colloc+blk(i,3)-1;
                index.repcomp.origloc = [index.repcomp.origloc; i];
                index.repcomp.repeated = [index.repcomp.repeated; blk(i,3)];
                rowloc = rowloc + blk(i,3);
                colloc = colloc + blk(i,3);
                index.simpleblk = [index.simpleblk;blk(i,3) 0];
            elseif blk(i,3) == 1         % complex, scalar, not repeated, shows up as full
                index.full.num = index.full.num + 1;
                index.full.rows{index.full.num} = rowloc;
                index.full.cols{index.full.num} = colloc;
                index.full.origloc = [index.full.origloc; i];
                index.full.dim = [index.full.dim; 1 1];
                rowloc = rowloc + blk(i,3);
                colloc = colloc + blk(i,3);
                index.simpleblk = [index.simpleblk;1 1];
            elseif blk(i,3) == -1         % real, scalar
                index.sreal.num = index.sreal.num + 1;
                index.sreal.rows{index.sreal.num} = rowloc;
                index.sreal.cols{index.sreal.num} = colloc;
                index.sreal.origloc = [index.sreal.origloc; i];
                rowloc = rowloc + abs(blk(i,3));
                colloc = colloc + abs(blk(i,3));
                index.simpleblk = [index.simpleblk;-1 0];
             elseif blk(i,3) < -1         % real, repeated
                index.repreal.num = index.repreal.num + 1;
                index.repreal.rows{index.repreal.num} = rowloc:rowloc+abs(blk(i,3))-1;
                index.repreal.cols{index.repreal.num} = colloc:colloc+abs(blk(i,3))-1;
                index.repreal.origloc = [index.repreal.origloc; i];
                index.repreal.repeated = [index.repreal.repeated; abs(blk(i,3))];
                rowloc = rowloc + abs(blk(i,3));
                colloc = colloc + abs(blk(i,3));
                %index.simpleblk = [index.simpleblk;-blk(i,3) 0];
                index.simpleblk = [index.simpleblk;blk(i,3) 0];
             elseif blk(i,3) == 0
                error('Scalar block repeated zero times...');
             end
          else                          % full block
            if blk(i,3) > 1             % complex , repeated
                index.repfull.num = index.repfull.num + 1;
                index.repfull.rows{index.repfull.num} = rowloc:rowloc+(blk(i,2)*blk(i,3))-1;
                index.repfull.cols{index.repfull.num} = colloc:colloc+(blk(i,1)*blk(i,3))-1;
                index.repfull.origloc = [index.repfull.origloc; i];
                index.repfull.repeated = [index.repfull.repeated; blk(i,3)];
                index.repfull.dim = [index.repfull.dim; blk(i,[1 2])];
                rowloc = rowloc + (blk(i,2)*blk(i,3));
                colloc = colloc + (blk(i,1)*blk(i,3));
                index.simpleblk = [index.simpleblk;ones(blk(i,3),1)*blk(i,[1 2])];
            elseif blk(i,3) == 1         % complex full (once)
                index.full.num = index.full.num + 1;
                index.full.rows{index.full.num} = rowloc:rowloc+blk(i,2)-1;
                index.full.cols{index.full.num} = colloc:colloc+blk(i,1)-1;
                index.full.origloc = [index.full.origloc; i];
                index.full.dim = [index.full.dim; blk(i,[1 2])];
                rowloc = rowloc + blk(i,2)*blk(i,3);
                colloc = colloc + blk(i,1)*blk(i,3);
                index.simpleblk = [index.simpleblk;blk(i,[1 2])];
            elseif blk(i,3) < 0         % real, full
                error('Real full block currently not allowed ...');
            elseif blk(i,3) == 0
                error('Scalar block repeated zero times...');
            end
        end
    end
             
 
             
