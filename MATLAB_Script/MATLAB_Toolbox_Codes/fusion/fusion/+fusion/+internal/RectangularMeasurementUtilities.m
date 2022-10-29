classdef (Hidden) RectangularMeasurementUtilities
    % This is an internal class and may be removed or modified in a future
    % release
    
    % Copyright 2019 The MathWorks, Inc.
    
    % This is a utility class to compute rectangular target measurements,
    % corners etc
    
    %#codegen
    
    methods (Static)
        function zOut = computeTwoSidedRectangularMeasurement(z, states, etas, sideLengths, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange)
            % computeTwoSidedRectangularMeasurement computes expected measurements from
            % a state when the obtained measurements are from a two-sided detection.
            %
            % z is the measurements in Cartesian coordinates in the sensor coordinate
            % system. A 2XM matrix
            %
            % states is the states of rectangular constant turn-rate target model
            % states. A 7XN matrix.
            %
            % etas are the normals of the rectangular sides. A 4xN matrix describing
            % the normal of each side. Side 1 is the front side and index increases
            % anti-clockwise looking through the rectangle's own coordinate frame.
            %
            % sideLengths are the lengths of those side. A 4XN matrix.
            %
            % isRect - hasRange (standard measurement parameters across detections);
            
            classToUse = class(states);
            
            % Step 1. We need to sort the measurements in their scan order. The
            % assumption that measurements sorted in the order of the scan (azimuth)
            % angle from the sensor does not work for edge cases. The scan order is
            % obtained by using the azimuth of measurements from their
            % "bounding box" mean
            xDets = z(1,:);
            yDets = z(2,:);
            lookPointX = (max(xDets) + min(xDets))/2;
            lookPointY = (max(yDets) + min(yDets))/2;
            azSort = atan2d(yDets - lookPointY, xDets - lookPointX);
            [azSorted,scanSortID] = sort(azSort,'ascend');
            
            % Currently the measurements are sorted by their azimuths as viewed from
            % the mean center. However, the start index is not known. For example,
            % the measurement may currently sorted like [5 6 7 8 1 2 3 4]. To find the
            % start index, we will minimize the distance from first to last
            % measurement i.e d12 + d23 + ... d78 will be the minimum distance.
            
            % Current order of the detections
            xDetsCirc = xDets(scanSortID);
            yDetsCirc = yDets(scanSortID);
            
            % Find extremum point by minimizing inter distance between points
            d = zeros(numel(azSorted),1);
            for i = 1:numel(azSorted)
                thisXSorted = circshift(xDetsCirc,i);
                thisYSorted = circshift(yDetsCirc,i);
                d(i) = sum(diff(thisXSorted).^2 + diff(thisYSorted).^2);
            end
            [~,startID] = min(d);
            scanSortID = circshift(scanSortID,startID);
            zSorted = z(:,scanSortID);
            
            % Step 2. Compute the corner index
            cornerIndex = fusion.internal.RectangularMeasurementUtilities.rectangleMeasCornerIndex(zSorted);
            
            % Now the measurements are sorted in two sets
            z1 = zSorted(:,1:cornerIndex);
            z2 = zSorted(:,cornerIndex:end); % Keep the corner here to have a "line"
            
            % Step 3. Use the first set and compute measurements originating from one
            % side.
            [zExp1, associatedSide] = fusion.internal.RectangularMeasurementUtilities.computeOneSidedRectangularMeasurement(z1, states, etas, sideLengths, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange, true);
            
            % Step 4. Use the second set and compute measurements originating from the
            % second side by using the associated side. The second set must come from
            % the adjacent side.
            associatedSide = associatedSide + 1;
            associatedSide(associatedSide > 4) = 1; % anticlockwise adjacent from side 4 is side 1.
            
            zExp2 = fusion.internal.RectangularMeasurementUtilities.computeOneSidedRectangularMeasurement(z2, states, etas, sideLengths, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange, true, associatedSide);
            
            % Remove corner which was added twice and concatenate them.
            zExp = cat(3,zExp1,zExp2(:,:,2:end));
            
            % Step 4. Output measurements in the right order
            zOut = zeros(size(zExp),classToUse);
            zOut(:,:,scanSortID) = zExp;
            
        end
        
        
        function [zOut, associatedSide] = computeOneSidedRectangularMeasurement(z, states, etas, sideLengths, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange, isSorted, varargin)
            % computeTwoSidedRectangularMeasurement computes expected measurements from
            % a state when the obtained measurements are from a two-sided detection.
            %
            % z is the measurements in Cartesian coordinates in the sensor coordinate
            % system. A 2XM matrix
            %
            % states is the states of rectangular constant turn-rate target model
            % states. A 7XN matrix.
            %
            % etas are the normals of the rectangular sides. A 4xN matrix describing
            % the normal of each side. Side 1 is the front side and index increases
            % anti-clockwise looking through the rectangle's own coordinate frame.
            %
            % sideLengths are the lengths of those side. A 4XN matrix.
            %
            % isRect - hasRange (standard measurement parameters across detections);
            %
            % isSorted means that measurements are sorted in anti-clockwise order
            % looking from the rectangular targets frame.
            %
            % associatedSide means information about associated side is already known.
            
            %#codegen
            
            % Number of samples to generate
            numSamples = size(z,2);
            
            % Number of states
            N = size(states,2);
            
            % Class to use
            classToUse = class(states);
            
            if ~isSorted
                azCell = atan2d(z(2,:),z(1,:));
                % Sort them in clockwise order. This results in
                % anti-clockwise order in the rectangular's forward frame.
                [~,scanSortID] = sort(azCell,'descend');
            else
                % Already sorted
                scanSortID = 1:numSamples;
            end
            z = z(:,scanSortID);
            
            % Length of the side measured according to detections.
            dy = z(2,end) - z(2,1);
            dx = z(1,end) - z(1,1);
            measLength = sqrt(dx.^2 + dy.^2);
            
            % Compute associated side if not provided as an input
            if numel(varargin) == 0
                beta = wrapTo2Pi(atan2(dy,dx) - pi/2);
                betaMinusEta = bsxfun(@minus, beta, etas);
                [~,associatedSide] = min(abs(wrapToPi(betaMinusEta)),[],1);
                isCornerObserved = false;
            else
                % Associated side is provided and corner was observed.
                associatedSide = varargin{1};
                isCornerObserved = true;
            end
            
            % Actual lengths of the side.
            selectSideID = sub2ind([4 N],associatedSide,1:N);
            associatedSideLength = sideLengths(selectSideID);
            
            % Check if measured length is smaller than actual length. Lets call the
            % ratio of measured/actual length as alpha.
            alpha = measLength./associatedSideLength;
            
            % A detection set may also be measuring an occluded side of the side. Lets
            % call the ratio of distance from first observed point to the first corner
            % in clockwise direction as d and define an offset beta as ratio of d/L.
            % Note than alpha + beta must be less than 1.
            beta = zeros(1,N,classToUse);
            
            % Test if the length is partially observed. A side is declared
            % partially observed if the ratio of observed length vs actual
            % length lie below a certain threshold 0.6. This is to prevent
            % the effect of noise causing the dimensions to change.
            partialObserved = alpha <= 0.6;
            alpha(~partialObserved) = 1;
            
            % For any partially observed side and unobserved corner, the
            % offset is calculated using the following logic:
            %
            % Considered are three possibilities
            % 1. Top corner was observed
            % 2. Bottom corner was observed
            % 3. Or mean of the side was observed.
            % 
            % Three distances are calculated and a decision about 1, 2 or 3
            % is made.
            %
            if any(partialObserved) && ~isCornerObserved
                statesPartial = states(:,partialObserved);
                associatedSidePartial = associatedSide(partialObserved);
                alphaPartial = alpha(partialObserved);
                
                % Corner positions of partially observed states
                cornerPosPartial = fusion.internal.RectangularMeasurementUtilities.sampledRectangularMeasurements(statesPartial, ...
                    associatedSidePartial, 2,  true, sensorPos, sensorVel, ...
                    sensorOrient, hasAz, hasEl, false, hasRange);
                
                % Bottom corner
                cornerPosFirst = cornerPosPartial(1:2,:,1);
                
                % Top corner
                cornerPosSecond = cornerPosPartial(1:2,:,2);
                
                % Mean of corners
                meanCorner = cornerPosFirst/2 + cornerPosSecond/2;
                
                % Distance from each value
                dFromCornerMeas = zeros(3,sum(partialObserved),classToUse);
                
                % Distance of bottom measurement to bottom corner
                dFromCornerMeas(1,:) = sqrt(sum(bsxfun(@minus,z(:,1),cornerPosFirst).^2,1));
                % Distance of top measurement to top corner
                dFromCornerMeas(2,:) = sqrt(sum(bsxfun(@minus,z(:,end),cornerPosSecond).^2,1));
                % Distance of mean measurement to mean of corners
                meanMeas = mean(z,2);
                dFromCornerMeas(3,:) = sqrt(sum(bsxfun(@minus,meanCorner,meanMeas).^2,1));
                
                % Find which distance is closest
                [~,minDistanceIndex] = min(dFromCornerMeas,[],1);
                offset = zeros(1,sum(partialObserved),classToUse);
                
                % Bottom corner is closest
                offset(minDistanceIndex == 1) = 0;
                
                % Top corner is closest
                offset(minDistanceIndex == 2) = (1 - alphaPartial(minDistanceIndex == 2));
                
                % Mean is closest
                offset(minDistanceIndex == 3) = 1/2*(1 - alphaPartial(minDistanceIndex == 3));
                beta(partialObserved) = offset;
            end
            
            % The detections may also not be uniformly generated on the side. A
            % non-uniform sample is generated by using the inter-distance between
            % observations.
            dxdy = diff(z,[],2);
            interDistance = [0 sqrt(sum(dxdy.^2,1))];
            zSample = fusion.internal.RectangularMeasurementUtilities.sampledRectangularMeasurements(states, associatedSide,...
                numSamples, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange, beta, alpha, interDistance(:));
            
            % Output with right order.
            zOut = zeros(size(zSample),classToUse);
            zOut(:,:,scanSortID) = zSample;
        end
        
        function zExp = computePointRectangularMeasurement(zMeas, states, isRect, sensorpos, sensorvel, localAxes, hasAz, hasEl, hasVel, hasRange)
            % computePointRectangularMeasurement returns the closest center
            % of the side to the actual measurement. 
            % 
            % zMeas is the actual measurement obtained in the Sensor
            % coordiante system (not Cartesian as in other compute
            % functions)
            
            numSamples = 1;
            numSides = 4;
            N = size(states,2);
            
            % Take advantage of vectorized sampling for each state
            zSamples = zeros(size(zMeas,1),N,numSamples*numSides,class(states));
            stateFourSides = [states states states states];
            sideIndexFourSides = [ones(1,N) 2*ones(1,N) 3*ones(1,N) 4*ones(1,N)];
            sample = fusion.internal.RectangularMeasurementUtilities.sampledRectangularMeasurements(stateFourSides, sideIndexFourSides, numSamples,...
                    isRect, sensorpos, sensorvel, localAxes, hasAz, hasEl, hasVel, hasRange);
            
            % Distribute each side sample to third dimension
            for i = 1:4
                index = ((i-1)*N + 1):i*N;
                zSamples(:,:,i) = sample(:,index,1:numSamples);
            end
            
            % Distance from measurement and samples
            d = sum((bsxfun(@minus,zSamples,zMeas)).^2,1);
            
            % Output closest sample
            [~,indices] = min(d,[],3);
            selectID = sub2ind([N numSamples*numSides],1:N,indices);
            zExp = zSamples(:,selectID);
        end

        function zSample = sampledRectangularMeasurements(states, sideIndex, numSamples, isRect, sensorpos, sensorvel, localAxes, hasAz, hasEl, hasVel, hasRange, varargin)
            % sampledRectangularMeasurements computes the sampled measurements from a
            % particular side of a constant turn-rate rectangular target model.
            %
            % This is an internal function and may be removed or modified in a future
            % release.
            %
            % states are all states following the state convention of ctrect
            % sideIndex is the index of the side which needs to be sampled.
            % numSamples are number of samples to generate from the side.
            % Other inputs are regular inputs generated by measurement parameters.
            
            % No validation is performed in this function
            N = size(states,2);
            classToUse = class(states);
            
            if nargin > 11
                offset = varargin{1};
                shrinkage = varargin{2};
            else
                offset = zeros(1,N,classToUse);
                shrinkage = ones(1,N,classToUse);
            end
            
            if numSamples == 1
                % Center of the side
                sampleSpacing = cast(1/2,classToUse);
            elseif nargin > 13
                % Non-uniform sampling across the side
                d = varargin{3};
                sampleSpacing = cast(cumsum(d/sum(d)),classToUse);
            else
                % Uniform sampling across the side
                sampleSpacing = cast((0:1/(numSamples-1):1)',classToUse);
            end
            
            % Compute corner measurements in rectangular frame
            zCornerRectangular = fusion.internal.RectangularMeasurementUtilities.computeCornerMeasurements(states, true, sensorpos, sensorvel, localAxes, hasAz, hasEl, hasVel, hasRange);
            
            % Two corners per side, lets select those corners
            corner1Index = sideIndex;
            corner2Index = sideIndex + 1;
            corner2Index(corner2Index > 4) = 1;
            
            corner1Selector = sub2ind([N 4],1:N,corner1Index);
            corner2Selector = sub2ind([N 4],1:N,corner2Index);
            zCorner1 = zCornerRectangular(:,corner1Selector);
            zCorner2 = zCornerRectangular(:,corner2Selector);
            
            % corner difference
            deltaCorner = zCorner2 - zCorner1;
            
            % Account for offset
            zCorner1 = zCorner1 + bsxfun(@times,offset,(deltaCorner));
            
            % Account for shrinkage
            deltaCorner = bsxfun(@times,deltaCorner,shrinkage);
            
            % Compute samples
            if coder.target('MATLAB')
                zSampleCart = zCorner1 + deltaCorner.*reshape(sampleSpacing,1,1,numSamples);
            else
                zSampleCart = repmat(zCorner1,[1 1 numSamples]);
                for i = 1:numSamples
                    zSampleCart(:,:,i) = zCorner1 + deltaCorner*sampleSpacing(i);
                end
            end
            
            % If coordinates were rectangular, nothing left to do. If
            % coordinates are spherical, transform to spherical
            % coordinates. zSampled is currently in sensor Cartesian
            % coordinates.
            if ~isRect
                pos = zSampleCart(1:3,:);
                if hasVel
                    vel = zSampleCart(4:6,:);
                else
                    vel = zeros(3,N*numSamples);
                end
                meas = posvel2Measurement(pos, vel, false, zeros(3,1,classToUse), ...
                    zeros(3,1,classToUse),eye(3,classToUse), hasAz, hasEl, hasVel, hasRange);
                zSampleSph = reshape(meas,size(meas,1),N,numSamples);
                zSample = zSampleSph;
            else
                zSample = zSampleCart;
            end
        end
        
        function zCorners = computeCornerMeasurements(stateCol, isRect, sensorpos, sensorvel, localAxes, hasAz, hasEl, hasVel, hasRange)
            % computeCornerMeasurements computes the measurements from corners of a
            % constant turn-rate rectangular model.
            
            % No validation is done here.
            numVecs = size(stateCol,2);
            classToUse = class(stateCol);
            
            % Parse states into physical variables.
            xC = stateCol(1,:);
            yC = stateCol(2,:);
            s = stateCol(3,:);
            omega = deg2rad(stateCol(5,:));
            L = stateCol(6,:);
            W = stateCol(7,:);
            theta = stateCol(4,:);
            sinThetas = sind(theta);
            cosThetas = cosd(theta);
            vx = s.*cosThetas;
            vy = s.*sinThetas;
            
            cornerPos = zeros(3,numVecs,4,classToUse);
            cornerVel = zeros(3,numVecs,4,classToUse);
            
            % Position and velocity of corners.
            % Corner 1 is right forward of the rectangle and index goes up
            % anticlockwise in rectangle's forward frame.
            cornerPos(1,:,1) = xC + L/2.*cosThetas + (-W/2).*(-sinThetas);
            cornerPos(1,:,2) = xC + L/2.*cosThetas + (W/2).*(-sinThetas);
            cornerPos(1,:,3) = xC + (-L/2).*cosThetas + (W/2).*(-sinThetas);
            cornerPos(1,:,4) = xC + (-L/2).*cosThetas + (-W/2).*(-sinThetas);
            
            cornerPos(2,:,1) = yC + L/2.*sinThetas + (-W/2).*cosThetas;
            cornerPos(2,:,2) = yC + L/2.*sinThetas + (W/2).*cosThetas;
            cornerPos(2,:,3) = yC + (-L/2).*sinThetas + (W/2).*cosThetas;
            cornerPos(2,:,4) = yC + (-L/2).*sinThetas + (-W/2).*cosThetas;
            
            cornerVel(1,:,1) = vx + L/2.*(-sinThetas).*omega + (-W/2).*(-cosThetas).*omega;
            cornerVel(1,:,2) = vx + L/2.*(-sinThetas).*omega + (W/2).*(-cosThetas).*omega;
            cornerVel(1,:,3) = vx + (-L/2).*(-sinThetas).*omega + (W/2).*(-cosThetas).*omega;
            cornerVel(1,:,4) = vx + (-L/2).*(-sinThetas).*omega + (-W/2).*(-cosThetas).*omega;
            
            cornerVel(2,:,1) = vy + L/2.*(cosThetas).*omega + (-W/2).*(-sinThetas).*omega;
            cornerVel(2,:,2) = vy + L/2.*(cosThetas).*omega + (W/2).*(-sinThetas).*omega;
            cornerVel(2,:,3) = vy + (-L/2).*(cosThetas).*omega + (W/2).*(-sinThetas).*omega;
            cornerVel(2,:,4) = vy + (-L/2).*(cosThetas).*omega + (-W/2).*(-sinThetas).*omega;
            
            tgtpos = zeros(3,numVecs*4,classToUse);
            tgtvel = zeros(3,numVecs*4,classToUse);
            
            tgtpos(1:2,:) = cornerPos(1:2,:);
            tgtvel(1:2,:) = cornerVel(1:2,:);
            
            measurement = posvel2Measurement(tgtpos, tgtvel, isRect, sensorpos, sensorvel, localAxes, hasAz, hasEl, hasVel, hasRange);

            zCorners = reshape(measurement,size(measurement,1),numVecs,4);            
        end
        
        function nHat = rectangleMeasCornerIndex(z)
            % This function computes the index of the measurement corresponding to the
            % corner of a rectangle. It is assumed that measurements are sorted by
            % their scan angle in any direction.
            % z is a 2xN matrix, where first row represents x position and second row
            % represents y position.
            
            % Reference:
            % Granström, Karl, Christian Lundquist, and Umut Orguner. "Tracking
            % rectangular and elliptical extended targets using laser measurements."
            % 14th International Conference on Information Fusion. IEEE, 2011.
            
            classToUse = class(z);
            dMin = cast(inf,classToUse);
            d = cast(0,classToUse);
            Nz = size(z,2);
            nHat = nan(classToUse);
            
            for n = 2:(Nz-1)
                d(1) = 0;
                for k = 2:(Nz-1)
                    if k < n
                        d = d + point2LineDist(z(:,1),z(:,n),z(:,k));
                    else
                        d = d + point2LineDist(z(:,n),z(:,Nz),z(:,k));
                    end
                end
                if d < dMin
                    nHat = n;
                    dMin = d;
                end
            end
        end
    end
end


function z = wrapTo2Pi(z)
z = fusion.internal.UnitConversions.interval(z,[0 2*pi]);
end
function z = wrapToPi(z)
z = fusion.internal.UnitConversions.interval(z,[-pi pi]);
end

function d = point2LineDist(z1,z2,z3)
    d = abs((z2(1) - z1(1))*(z1(2) - z3(2)) - (z1(1) - z3(1))*(z2(2) - z1(2)))/(sqrt((z2(1) - z1(1))^2 + (z2(2) - z1(2))^2));
end

function measurement = posvel2Measurement(tgtpos, tgtvel, isRect, sensorpos, sensorvel, localAxes, hasAz, hasEl, hasVel, hasRange)
% Convert position, velocity to measurements in sensor frame.
if isRect
    if ~hasVel % Only position is measured
        measurement = matlabshared.tracking.internal.fusion.global2localcoord(tgtpos,'rr',...
            sensorpos,localAxes);
    else % Both position and velocity are measured
        posMeas = matlabshared.tracking.internal.fusion.global2localcoord(tgtpos,'rr',...
            sensorpos,localAxes);
        velMeas = matlabshared.tracking.internal.fusion.global2localcoord(tgtvel,'rr',...
            sensorvel,localAxes);
        measurement = [posMeas;velMeas];
    end
else % Spherical coordinate system.
    
    % Range-rate available if hasRange is true
    hasRangeRate = hasVel && hasRange;
    
    % Define the size of the measurement
    measSize = hasAz + hasEl + hasRange + hasRangeRate;
    
    measurement = zeros(measSize,size(tgtpos,2),class(tgtpos));
    
    meas = matlabshared.tracking.internal.fusion.global2localcoord(tgtpos,'rs',...
        sensorpos,localAxes);
    
    measLogicalIndex = [hasAz hasEl hasRange];
    
    if ~hasRangeRate
        % Preserve data type, size of measurement
        measurement(:) = meas(measLogicalIndex,:);
    else
        measurement(1:end-1,:) = meas(measLogicalIndex,:);
        measurement(end,:) = calcRadialSpeed(tgtpos, tgtvel, sensorpos, sensorvel);
    end
end
end
            
function rspeed = calcRadialSpeed(tgtpos, tgtvel, refpos, refvel)
% calcRadialSpeed calculate the range rate in a desired frame.
% No validation is performed here.

tgtdirec = bsxfun(@minus,tgtpos,refpos);
veldirec = bsxfun(@minus,tgtvel,refvel);

%Take the 2-norm of veldirec and tgtdirec
rn = sqrt(sum(tgtdirec.^2));
sn = sqrt(sum(veldirec.^2));

% outgoing relative speed is positive
rspeed = (sum(veldirec.*tgtdirec)./rn);

%now take care of rspeed corner cases:
rspeed(sn==0) = 0;
% when co-located, the radial speed is relative speed, but departing
rnEq0 = (rn==0);
rspeed(rnEq0) = -sn(rnEq0);

%make a column vector
rspeed = reshape(rspeed, [],1);

end
