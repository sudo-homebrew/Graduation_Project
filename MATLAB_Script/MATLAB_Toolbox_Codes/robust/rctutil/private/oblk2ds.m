function [sb,fb,perfe,perfd,rmpert,cmpert,whichtype] = oblk2ds(blk,ne,nd)

% Copyright 2003-2004 The MathWorks, Inc.

    sbcnt = 0;
    fbcnt = 0;
    rmpt = 1;
    cmpt = 1;
    nblk = size(blk,1);
    sb.azidx = {};
    sb.bwidx = {};
    fb.azidx = {};
    fb.bwidx = {};
    sb.loc = [];
    fb.loc = [];
    whichtype = zeros(nblk,1);
        
    for i=1:nblk
        if blk(i,2)==0
            sbcnt = sbcnt + 1;
            sb.azidx{sbcnt} = [rmpt:rmpt+blk(i,1)-1]';
            sb.bwidx{sbcnt} = [cmpt:cmpt+blk(i,1)-1]';
            rmpt = rmpt + blk(i,1);
            cmpt = cmpt + blk(i,1);
            sb.loc = [sb.loc;i];
            whichtype(i) = sbcnt;
        else
            fbcnt = fbcnt + 1;
            fb.azidx{fbcnt} = [rmpt:rmpt+blk(i,2)-1]';
            fb.bwidx{fbcnt} = [cmpt:cmpt+blk(i,1)-1]';
            rmpt = rmpt + blk(i,2);
            cmpt = cmpt + blk(i,1);
            fb.loc = [fb.loc;i];
            whichtype(i) = -fbcnt;
        end
    end
    rmpert = rmpt-1;
    cmpert = cmpt-1;
    perfe = [rmpt:rmpt+ne-1]';
    perfd = [cmpt:cmpt+nd-1]';
            
