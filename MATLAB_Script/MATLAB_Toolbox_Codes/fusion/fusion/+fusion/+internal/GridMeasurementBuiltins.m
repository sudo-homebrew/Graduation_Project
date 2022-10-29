classdef GridMeasurementBuiltins
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % This class provides a list of Static methods used by trackerGridRFS
    % and associated objects to perform some routine operations.
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    methods (Static)
        function xy = SensorDataToLocalGrid(sensorData, sensorConfig)
            % xy = SensorDataToLocalGrid transforms sensorData to local x,y
            % coordinate data.
            %
            % sensorData is a struct with fields Measurement and
            % MeasurementParameters.
            %
            % sensorConfig is a trackingSensorConfiguration object.
            
            if isfield(sensorData,'MeasurementParameters')...
                    && ~isempty(sensorData.MeasurementParameters)
                measParams = sensorData.MeasurementParameters;
            else
                measParams = struct;
            end
            [isRect, origin, ~, orientation, hasAz, hasEl, ~, hasRange] = ...
                matlabshared.tracking.internal.fusion.parseMeasurementParameters(measParams,mfilename,class(sensorData.Measurement));
            
            cond = isRect || (hasAz && hasRange);
            coder.internal.assert(cond,'fusion:GridTracker:InvalidMeasurementParameters');
            expMeasSize = matlabshared.tracking.internal.fusion.getExpectedMeasurementSize(isRect,hasAz,hasEl,false,hasRange);
            coder.internal.assert(size(sensorData.Measurement,1) == expMeasSize,'fusion:GridTracker:InvalidMeasurementSize', expMeasSize);
            
            if ~isRect
                % Spherical data.
                az = sensorData.Measurement(1,:);
                if hasEl
                    % If elevation is available, use it
                    el = sensorData.Measurement(2,:);
                    r = sensorData.Measurement(3,:);
                else
                    % Assume zero elevation
                    el = zeros(1,numel(az),'like',az);
                    r = sensorData.Measurement(2,:);
                end
                % Convert to Cartesian coordinates
                [x,y,z] = sph2cart(deg2rad(az),deg2rad(el),r);
                xyzMeas = [x;y;z];
            else
                xyzMeas = sensorData.Measurement;
            end
            
            % If no measurements, simply return empty of right size
            if isempty(xyzMeas)
                xy = zeros(2,0,'like',xyzMeas);
                return;
            end
            
            % Convert measurement to tracking coordinates
            xyzTracking = matlabshared.tracking.internal.fusion.local2globalcoord(xyzMeas,'rr',origin,orientation);
            
            % Convert tracking coordinates to local coordinates
            gridToTracking = sensorConfig.SensorTransformParameters(2:end);
            
            if isempty(gridToTracking)
                xyzGrid = xyzTracking;
            else
                [~, gridOrigin, ~, gridOrientation] = ...
                    matlabshared.tracking.internal.fusion.parseMeasurementParameters(gridToTracking,mfilename,class(sensorData.Measurement));
                
                xyzGrid = matlabshared.tracking.internal.fusion.global2localcoord(xyzTracking,'rr',gridOrigin,gridOrientation);
            end
            
            % Output 2-D locations in local coordinates
            xy = xyzGrid(1:2,:)';
        end
    end
    
    %% ParticlePositionFcn
    methods (Static)
        % Extracting particle positions in local coordinates. particle
        % states are in global coordinates
        function pos = cvParticlePositionFcn(state,params)
            pos = fusion.internal.GridMeasurementBuiltins.PositionTransformFcn(state([1 3],:),params);
        end
        
        function pos = caParticlePositionFcn(state,params)
            pos = fusion.internal.GridMeasurementBuiltins.PositionTransformFcn(state([1 4],:),params);
        end
        
        function pos = ctParticlePositionFcn(state,params)
            pos = fusion.internal.GridMeasurementBuiltins.PositionTransformFcn(state([1 3],:),params);
        end
        
        function posLocal = PositionTransformFcn(posState, params)
            classToUse = class(posState);
            if isa(posState,'gpuArray')
                undClass = classUnderlying(posState);
            else
                undClass = classToUse;
            end
            n = size(posState, 2);
            posState3 = [posState;zeros(1,n,'like',posState)];
            if ~isempty(params)
                [~,origin,~,orientation] = matlabshared.tracking.internal.fusion.parseMeasurementParameters(params,mfilename,undClass);
            else
                origin = zeros(3,1,'like',posState);
                orientation = eye(3,'like',posState);
            end
            pos3 = matlabshared.tracking.internal.fusion.global2localcoord(posState3,'rr',origin,orientation);
            posLocal = pos3(1:2,:);
        end
    end
    
    %% ParticlePositionSamplingFcns
    methods (Static)
        function state = cvParticleSamplingFcn(pos,params,stateLimits,varargin)
            posState = fusion.internal.GridMeasurementBuiltins.InversePositionFcn(pos,params);
            n = size(pos,2);
            if n > 0
                ndist = matlabshared.tracking.internal.UniformDistribution(2);
                ndist.RandomVariableLimits = stateLimits;
                velSamples = ndist.sample(n);
            else
                velSamples = zeros(0,2);
            end
            
            n = size(pos,2);
            state = zeros(4,n,'like',posState);
            state([1 3],:) = posState;
            state(2,:) = velSamples(:,1);
            state(4,:) = velSamples(:,2);
        end
        
        function state = caParticleSamplingFcn(pos,params,stateLimits,varargin)
            posState = fusion.internal.GridMeasurementBuiltins.InversePositionFcn(pos,params);
            n = size(pos,2);
            if n > 0
                udist = matlabshared.tracking.internal.UniformDistribution(4);
                udist.RandomVariableLimits = stateLimits;
                samples = udist.sample(n);
                velSamples = samples(:,[1 2]);
                accSamples = samples(:,[3 4]);
            else
                velSamples = zeros(0,2,'like',posState);
                accSamples = zeros(0,2,'like',posState);
            end
            
            n = size(pos,2);
            state = zeros(6,n,'like',posState);
            state([1 4],:) = posState;
            state(2,:) = velSamples(:,1);
            state(3,:) = accSamples(:,1);
            state(5,:) = velSamples(:,2);
            state(6,:) = accSamples(:,2);
        end
        

        function state = ctParticleSamplingFcn(pos,params,stateLimits,varargin)
            % Compute position in tracking frame using params
            posState = fusion.internal.GridMeasurementBuiltins.InversePositionFcn(pos,params);
            n = size(pos,2);
            if n > 0
                udist = matlabshared.tracking.internal.UniformDistribution(3);
                udist.RandomVariableLimits = stateLimits;
                samples = udist.sample(n);
                velSamples = samples(:,[1 2]);
                turnrateSamples = samples(:,3);
            else
                velSamples = zeros(0,2,'like',posState);
                turnrateSamples = zeros(0,1,'like',posState);
            end
            state = zeros(5,n,'like',posState);
            state([1 3],:) = posState;
            state(2,:) = velSamples(:,1);
            state(4,:) = velSamples(:,2);
            state(5,:) = turnrateSamples(:,1);
        end
        
        function posState = InversePositionFcn(pos, params)
            % Compute position in tracking frame using params
            if isa(pos,'gpuArray')
                undClass = classUnderlying(pos);
            else
                undClass = class(pos);
            end
            n = size(pos,2);
            if ~isempty(params)
                [~,origin,~,orientation] = matlabshared.tracking.internal.fusion.parseMeasurementParameters(params,mfilename,undClass);
            else
                origin = zeros(3,1,'like',pos);
                orientation = eye(3,'like',pos);
            end
            pos3 = [pos;zeros(1,n,'like',pos)];
            posState3 = matlabshared.tracking.internal.fusion.local2globalcoord(pos3,'rr',origin,orientation);
            posState = posState3(1:2,:);
        end
    end
    
    %% LikelihoodFcn
    methods (Static)
        function likelihood = CVLikelihood(dynamicCells, tracks)
            % Get track state and state covariances from the track
            [trackBox, trackBoxCov, trackVel, trackVelCov] = fusion.internal.GridMeasurementBuiltins.CVTrackBoxStates(tracks);
            
            % Get cell and corresponding state covariance
            [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = fusion.internal.GridMeasurementBuiltins.CVCellStates(dynamicCells);
            
            % Occupancy likelihood
            occLikelihood = fusion.internal.GridMeasurementBuiltins.OccupancyLikelihood(xCell,yCell,occCell,trackBox,trackBoxCov);
            
            % Velocity likelihood
            velLikelihood = fusion.internal.GridMeasurementBuiltins.VelocityLikelihood(vxCell, vyCell, Pvx, Pvy, Pvxvy, trackVel, trackVelCov);
            
            likelihood = occLikelihood.*velLikelihood;
        end
        
        function likelihood = CALikelihood(dynamicCells, tracks)
            % Get track state and state covariances from the track
            [trackBox, trackBoxCov, trackVel, trackVelCov] = fusion.internal.GridMeasurementBuiltins.CATrackBoxStates(tracks);
            
            % Get cell and corresponding state covariance
            [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = fusion.internal.GridMeasurementBuiltins.CACellStates(dynamicCells);
            
            % Occupancy likelihood
            occLikelihood = fusion.internal.GridMeasurementBuiltins.OccupancyLikelihood(xCell,yCell,occCell,trackBox,trackBoxCov);
            
            % Velocity likelihood
            velLikelihood = fusion.internal.GridMeasurementBuiltins.VelocityLikelihood(vxCell, vyCell, Pvx, Pvy, Pvxvy, trackVel, trackVelCov);
            
            likelihood = occLikelihood.*velLikelihood;
        end
        
        function likelihood = CTLikelihood(dynamicCells, tracks)
            % Get track state and state covariances from the track
            [trackBox, trackBoxCov, trackVel, trackVelCov] = fusion.internal.GridMeasurementBuiltins.CTTrackBoxStates(tracks);
            
            % Get cell and corresponding state covariance
            [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = fusion.internal.GridMeasurementBuiltins.CTCellStates(dynamicCells);
            
            % Occupancy likelihood
            occLikelihood = fusion.internal.GridMeasurementBuiltins.OccupancyLikelihood(xCell,yCell,occCell,trackBox,trackBoxCov);
            
            % Velocity likelihood
            velLikelihood = fusion.internal.GridMeasurementBuiltins.VelocityLikelihood(vxCell, vyCell, Pvx, Pvy, Pvxvy, trackVel, trackVelCov);
            
            likelihood = occLikelihood.*velLikelihood;
        end
        
        function occLikelihood = OccupancyLikelihood(xCell, yCell, occCell, trackBox, trackBoxCov)
            % xCell is GridSize matrix
            % yCell is GridSize matrix
            
            % TrackBox is a 5-dimensional state with [x;y;theta;L;W] of the
            % track.
            % TrackBoxCov is the covariances in these states
            
            xBox = trackBox(1);
            yBox = trackBox(2);
            theta = trackBox(3);
            lBox = trackBox(4);
            wBox = trackBox(5);
            
            sigmaX = trackBoxCov(1,1);
            sigmaY = trackBoxCov(2,2);
            
            cosTheta = cosd(theta);
            sinTheta = sind(theta);
            
            x = xCell - xBox;
            y = yCell - yBox;
            
            xR = x.*cosTheta + y.*sinTheta;
            yR = -x.*sinTheta + y.*cosTheta;
            
            isInside = abs(xR) < lBox/2 & abs(yR) < wBox/2;
            
            dL = (abs(xR) - lBox/2).^2;
            dL = dL./(sigmaX);
            dW = (abs(yR) - wBox/2).^2;
            dW = dW./(sigmaY);
            
            occLikelihood = exp(-dL./2).*exp(-dW./2);
            occLikelihood(isInside) = 1;
            
            occLikelihood = reshape(occLikelihood,size(occCell)).*occCell;
        end
        
        function velLikelihood = VelocityLikelihood(vxCell, vyCell, Pvx, Pvy, Pvxvy, trackVelState, trackVelStateCov)
            evx = vxCell - trackVelState(1);
            evy = vyCell - trackVelState(2);
            
            Pvx = Pvx + trackVelStateCov(1,1);
            Pvy = Pvy + trackVelStateCov(2,2);
            Pvxvy = Pvxvy + trackVelStateCov(1,2);
            detS = Pvx.*Pvy - Pvxvy.^2;
            
            logLhood = 1/2*(Pvy.*evx.^2 + Pvx.*evy.^2 - 2*Pvxvy.*evy.*evy)./detS + 1/2*real(log(2*pi*detS));
            
            velLikelihood = exp(-logLhood);
        end
    end
    
    %% Track Parsing
    methods (Static)
        function [trackBox, trackBoxCov, trackVel, trackVelCov] = CVTrackBoxStates(track)
            idx = [1 3 5 6 7];
            idxVel = [2 4];
            [trackBox, trackBoxCov, trackVel, trackVelCov] = fusion.internal.GridMeasurementBuiltins.TrackBoxState(track,idx,idxVel);
        end
        
        function [trackBox, trackBoxCov, trackVel, trackVelCov] = CATrackBoxStates(track)
            idx = [1 4 7 8 9];
            idxVel = [2 5];
            [trackBox, trackBoxCov, trackVel, trackVelCov] = fusion.internal.GridMeasurementBuiltins.TrackBoxState(track,idx,idxVel);
        end
        
        function [trackBox, trackBoxCov, trackVel, trackVelCov] = CTTrackBoxStates(track)
            idx = [1 3 6 7 8];
            idxVel = [2 4];
            [trackBox, trackBoxCov, trackVel, trackVelCov] = fusion.internal.GridMeasurementBuiltins.TrackBoxState(track,idx,idxVel);
        end
        
        function [trackBox, trackBoxCov, trackVel, trackVelCov] = TrackBoxState(track,idx,idxVel)
            trackBox = track.State(idx);
            trackBoxCov = track.StateCovariance(idx,idx);
            trackVel = track.State(idxVel);
            trackVelCov = track.StateCovariance(idxVel,idxVel);
        end
    end
    
    %% Cell Parsing
    methods (Static)
        function [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = CVCellStates(dynamicCells)
            posIdx = [1 3];
            velIdx = [2 4];
            [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = fusion.internal.GridMeasurementBuiltins.CellState(dynamicCells,posIdx,velIdx);
        end
        
        function [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = CACellStates(dynamicCells)
            posIdx = [1 4];
            velIdx = [2 5];
            [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = fusion.internal.GridMeasurementBuiltins.CellState(dynamicCells,posIdx,velIdx);
        end
        
        function [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = CTCellStates(dynamicCells)
            posIdx = [1 3];
            velIdx = [2 4];
            [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = fusion.internal.GridMeasurementBuiltins.CellState(dynamicCells,posIdx,velIdx);
        end
        
        function [occCell, xCell, yCell, vxCell, vyCell, Pvx, Pvy, Pvxvy] = CellState(dynamicCells,posIdx,velIdx)
            occCell = dynamicCells.OccupancyEvidences;
            n = numel(occCell);
            classToUse = class(dynamicCells.OccupancyEvidences);
            xCell = zeros(n,1,classToUse);
            yCell = zeros(n,1,classToUse);
            vxCell = zeros(n,1,classToUse);
            vyCell = zeros(n,1,classToUse);
            Pvx = zeros(n,1,classToUse);
            Pvy = zeros(n,1,classToUse);
            Pvxvy = zeros(n,1,classToUse);
            
            xCell(:) = dynamicCells.States(posIdx(1),:);
            yCell(:) = dynamicCells.States(posIdx(2),:);
            vxCell(:) = dynamicCells.States(velIdx(1),:);
            vyCell(:) = dynamicCells.States(velIdx(2),:);
            Pvx(:) = dynamicCells.StateCovariances(velIdx(1),velIdx(1),:);
            Pvy(:) = dynamicCells.StateCovariances(velIdx(2),velIdx(2),:);
            Pvxvy(:) = dynamicCells.StateCovariances(velIdx(1),velIdx(2),:);
        end
    end
    
    %% Motion models
    methods (Static)
        function state = constvelrect(state, varargin)
            coder.internal.assert(size(state,1) == 7,'fusion:GridTracker:InvalidTrackStateSize',7);
            % Cv states
            cvstate = state(1:4,:);
            cvstate = constvel(cvstate, varargin{:});
            
            % box states (theta, l, w);
            
            % Box states remain constant
            boxState = state(5:7,:);
            state = [cvstate;boxState];
        end
        
        function [A, B] = constvelrectjac(state, varargin)
            coder.internal.assert(numel(state) == 7,'fusion:GridTracker:InvalidTrackStateSize',7);
            % Cv states
            cvstate = state(1:4);
            [cvstatejac,cvnoisejac] = constveljac(cvstate, varargin{:});
            
            % box states (theta, l, w);
            
            % Box states remain constant
            boxstatejac = eye(3,class(state));
            A = blkdiag(cvstatejac,boxstatejac);
            B = [cvnoisejac;zeros(3,2,class(state))];
        end
        
        function state = constaccrect(state, varargin)
            coder.internal.assert(size(state,1) == 9,'fusion:GridTracker:InvalidTrackStateSize',9);
            % Ca states
            castate = state(1:6,:);
            castate = constacc(castate, varargin{:});
            
            % box states (theta, l, w);
            
            % Box states remain constant
            boxState = state(7:9,:);
            state = [castate;boxState];
        end
        
        function [A, B] = constaccrectjac(state, varargin)
            coder.internal.assert(numel(state) == 9,'fusion:GridTracker:InvalidTrackStateSize',9);
            % Cv states
            castate = state(1:6);
            [castatejac, canoisejac] = constaccjac(castate, varargin{:});
            
            % box states (theta, l, w);
            
            % Box states remain constant
            boxstatejac = eye(3,class(state));
            A = blkdiag(castatejac,boxstatejac);
            B = [canoisejac;zeros(3,2,class(state))];
        end
        
        function state = constturnrect(state, varargin)
            coder.internal.assert(size(state,1) == 8,'fusion:GridTracker:InvalidTrackStateSize',8);
            % Ct states
            ctstate = state(1:5,:);
            ctstate = constturn(ctstate, varargin{:});
            
            % Box states (theta, l, w)
            % Theta changes with omega
            theta = state(6,:);
            theta = theta + state(5,:)*varargin{end};
            lw = state(7:8,:);
            state = [ctstate;theta;lw];
        end
        
        function [A, B] = constturnrectjac(state, varargin)
            coder.internal.assert(numel(state) == 8,'fusion:GridTracker:InvalidTrackStateSize',8);
            % Cv states
            ctstate = state(1:5);
            [ctstatejac, ctnoisejac] = constturnjac(ctstate, varargin{:});
            
            % box states (theta, l, w);
            
            % Box states (theta, l, w)
            % Theta changes with omega
            thetajac = zeros(1,8,class(state));
            thetajac(6) = 1;
            thetajac(5) = varargin{end};
            lwjac = [zeros(2,6,class(state)) eye(2,class(state))];
            A = [ctstatejac zeros(5,3);thetajac;lwjac];
            B = [ctnoisejac;zeros(3,3,class(state))];
        end
    end
    
    %% Track Initialization and Update Functions
    methods (Static)
        function track = initRectangularTrack(gridCells)
            % It is assumed that State of grid cells and state of track are in the same
            % coordinate system
            
            [xTrack, PTrack] = fusion.internal.GridMeasurementBuiltins.extractGaussianEstimateFromCell(gridCells);
            
            xTrack = gather(xTrack);
            PTrackPosDef = fusion.internal.GridMeasurementBuiltins.ensurePosDef(gather(PTrack));
            
            track = objectTrack('State',xTrack,...
                'StateCovariance',PTrackPosDef);
        end
        
        function track = updateRectangularTrack(track, gridCells) %#ok<INUSL>
            % Update simply creates a new track.
            % This is equivalent to simply overwriting the value of State
            % and StateCovariance with the new grid cell estimate. 
            track = fusion.internal.GridMeasurementBuiltins.initRectangularTrack(gridCells);
        end
        
        function [x, P] = extractGaussianEstimateFromCell(gridCells)
            state = gridCells.States;
            stateCov = gridCells.StateCovariances;
            cellWidth = gridCells.CellWidth;
            
            x = state(:,1);
            if numel(x) == 4 || numel(x) == 5
                vIdx = [2 4];
                posIdx = [1 3];
            elseif numel(x) == 6
                vIdx = [2 5];
                posIdx = [1 4];
            end
            
            n = size(state,2);
            
            weights = ones(n,1,'like',state)/n;
            
            % Merge states and state covariances. These are mean target states
            [~, x, P] = fusion.internal.ggiwMerge(weights, state, stateCov);
            
            vx = x(vIdx(1));
            vy = x(vIdx(2));
            yaw = atan2d(vy,vx);
            
            % Now extract the box center and dimension using all the cells again
            xCells = state(posIdx(1),:);
            yCells = state(posIdx(2),:);
            
            [dim, center] = fusion.internal.GridMeasurementBuiltins.extractBoxFromCell(xCells,yCells,cellWidth,yaw);
            
            % Assemble the state
            x(posIdx) = center;
            % Positional covariance limited to cellWidth.
            P(posIdx(1),posIdx(1)) = max(cellWidth.^2, P(posIdx(1),posIdx(1)));
            P(posIdx(2),posIdx(2)) = max(cellWidth.^2, P(posIdx(2),posIdx(2)));
            
            x = [x;yaw;dim(1);dim(2)];
            
            % Covariance. Orientation sigma can be calculated using sigma in vx and vy
            speed = sqrt(vx.^2 + vy.^2);
            velCov = P(vIdx,vIdx);
            yawCov = 1/speed^2*[cosd(yaw)^2 sind(yaw)^2 -2*cosd(yaw)*sind(yaw)]*[velCov(1,1);velCov(2,2);velCov(1,2)];
            yawCov(~isfinite(yawCov)) = 100; % unobservable yaw
            
            % Choose some appropriate value for lSigma and wSigma
            lSigma = 2;
            wSigma = 2;
            P = blkdiag(P,yawCov,lSigma,wSigma);
        end
        
        function [dim, center] = extractBoxFromCell(xCells, yCells, cellWidth, orient)
            xy = [xCells(:) yCells(:)]';
            
            R = [cosd(orient) -sind(orient);sind(orient) cosd(orient)];
            
            xyphi = R*xy;
            
            deltad = cellWidth*(abs(sind(orient)) + abs(cosd(orient)));
            
            maxPt = max(xyphi,[],2);
            
            minPt = min(xyphi,[],2);
            
            dim = maxPt - minPt + deltad;
            
            center = R'*(minPt + 1/2*(dim - deltad));
        end
        
        function P = ensurePosDef(P)
            P = fusion.internal.ensurePosDefMatrix(P);
        end
    end
    
    %% Track distance function
    methods (Static)
        function d = mapToTrackDistance(map, track) 
            % Simply call -log likelihood on the map.
            d = -log(gridTrackLikelihood(map,track));
        end
    end
end
