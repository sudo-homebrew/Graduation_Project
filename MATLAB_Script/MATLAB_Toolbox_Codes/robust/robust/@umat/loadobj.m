function M = loadobj(s)
%LOADOBJ  Load filter for UMAT objects

%   Copyright 1986-2010 The MathWorks, Inc.
if isa(s,'umat')
   M = s;
   M.Version_ = ltipack.ver();
else
   % Issue warning
   updatewarn
   
   if isfield(s,'NDlftData')
      % Pre-MCOS
      Data = s.NDlftData;
      IC = Data.MatrixData;
      % Construct block vector
      BlockVector = ltipack.LFTBlockWrapper.readOldFormat(...
         Data.DepData.AtomCell,Data.Copies,Data.Literal);
      % Move DELTA channels to bottom
      [nw,nz] = iosize(BlockVector);
      sIC = size(IC);  nyz = sIC(1);  nuw = sIC(2);
      IC = IC([nz+1:nyz,1:nz],[nw+1:nuw,1:nw],:);
      % Construct model
      if any(logicalfun(@(x) isa(x,'DynamicSystem'),BlockVector))
         % Reloading UMAT that contains ULTIDYN objects: return USS
         D = ltipack.lftdataSS.array([sIC(3:end) 1]);
         for ct=1:numel(D)
            D(ct).IC = ltipack.ssdata([],zeros(0,nuw),zeros(nyz,0),IC(:,:,ct),[],0);
            D(ct).Blocks = BlockVector;
            D(ct) = normalizeUncertainty(D(ct));
         end
         M = uss.make(D,[nyz-nz,nuw-nw]);
      else
         D = ltipack.lftdataM.array([sIC(3:end) 1]);
         for ct=1:numel(D)
            D(ct).IC = IC(:,:,ct);
            D(ct).Blocks = BlockVector;
            D(ct) = normalizeUncertainty(D(ct));
         end
         M = umat.make(D,[nyz-nz,nuw-nw]);
      end
   end
end
