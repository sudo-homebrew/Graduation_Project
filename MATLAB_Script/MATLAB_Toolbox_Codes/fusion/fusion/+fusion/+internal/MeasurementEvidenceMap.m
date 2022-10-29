classdef MeasurementEvidenceMap < fusion.internal.MapInterface
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    % Thic class represents the evidence and free belief masses for the
    % measurement grid. 
    %

    %#codegen
    properties (Access = {?fusion.internal.MeasurementEvidenceMap,...
            ?matlab.unittest.TestCase})
        OccupiedEvidence
        FreeEvidence
    end
    
    % Local x, y of the cell with respect to center
    properties (Access = {?fusion.internal.MeasurementEvidenceMap,?matlab.unittest.TestCase})
        XCell
        YCell
        CellWidth
    end
    
    properties (Access = protected)
        ClassToUse
        UseGPU
    end

    % Parameters that define the transformation from state of particles to
    % position on the grid
    properties
        PositionParameters;
    end
    
    methods
        function obj = MeasurementEvidenceMap(l,w,res,origin,varargin)
            obj.Length = l;
            obj.Width = w;
            obj.Resolution = res;
            obj.GridSize = [w*res l*res];
            obj.GridOriginInLocal = origin;
            
            % Find UseGPU using N/V pairs
            idxGPU = fusion.internal.findProp('UseGPU',varargin{:});
            if idxGPU < numel(varargin)
                useGPU = varargin{idxGPU+1};
            else
                useGPU = false;
            end
            obj.UseGPU = useGPU;
            
            % Find ClassToUse using N/V pairs
            idxClassToUse = fusion.internal.findProp('ClassToUse',varargin{:});
            if idxClassToUse < numel(varargin)
                classToUse = varargin{idxClassToUse+1};
            else
                classToUse = 'double';
            end
            obj.ClassToUse = classToUse;
            
            if obj.UseGPU
                dummyVar = zeros(1,obj.ClassToUse,'gpuArray');
            else
                dummyVar = zeros(1,obj.ClassToUse);
            end
            
            % Values for evidences for each grid cell
            obj.OccupiedEvidence = zeros(obj.GridSize,'like',dummyVar);
            obj.FreeEvidence = zeros(obj.GridSize,'like',dummyVar);
            
            % Cell properties
            [obj.XCell,obj.YCell,obj.CellWidth] = calculateMapProperties(obj);
        end
    end
    
    methods
        function [P_occ] = getOccupancy(obj, varargin)
            [m_occ, m_free] = getEvidences(obj, varargin{:});
            P_occ = m_occ + 0.5*(1 - m_occ - m_free);
        end
        
        function [m_occ, m_free] = getEvidences(obj, varargin)
            m_occ = obj.OccupiedEvidence;
            m_free = obj.FreeEvidence;
        end
    end
    
    methods (Access = protected)
        function [xCell, yCell, cellWidth] = calculateMapProperties(map)
            xCell = zeros(map.GridSize,'like',map.OccupiedEvidence);
            yCell = zeros(map.GridSize, 'like', map.OccupiedEvidence);
            cellWidth = ones(1,'like',map.OccupiedEvidence);
            [xCell(:),yCell(:),cellWidth(:)] = calculateMapProperties@fusion.internal.MapInterface(map);
        end
    end
    
    methods
        function insertData(obj, xyGrid, sensorPos, Pd, maxRange, fov, hasOcc) 
            % insertData(obj, xyGrid, sensorPos, Pd, maxRange, fov, hasOcc)
            % insert data on the grid using inputs.
            
            % xyGrid is a N-by-2 matrix defining the x,y position of the
            % measurements in the local coordinate system.
            %
            % sensorPos is the position and orientation of the sensor in the local
            % coordinate system specied as [x y theta].
            %   
            % Pd is the detection probability of the sensor.
            %
            % maxRange is the maximum detection range of the sensor.
            %
            % fov is the azimuth field of view of the sensor.
            %
            % hasOcc is a logical flag if occlusions should be considered
            % by projecting data.
            %
            % The method sequentially adds data on the grid by using D/S
            % combination rule.

            m_sensor_free = zeros(obj.GridSize,'like',obj.OccupiedEvidence);
            m_sensor_occ = zeros(obj.GridSize,'like',obj.OccupiedEvidence);
            
            
            rows = obj.GridSize(1);
            cols = obj.GridSize(2);
            gridOrigin = obj.GridOriginInLocal;
            r = obj.Resolution;
                
            dx = obj.XCell - sensorPos(1);
            dy = obj.YCell - sensorPos(2);
            rSensor2 = (dx).^2 + (dy).^2;
            R = fusion.internal.frames.ypr2rotmat([sensorPos(3) 0 0],'deg');
            R2 = R(1:2,1:2)';
            xSensor = dx*R2(1,1) + dy*R2(2,1);
            ySensor = dx*R2(1,2) + dy*R2(2,2);
            az = atan2d(ySensor,xSensor);
            withinR = rSensor2 < maxRange^2;
            withinFov = az >= fov(1) & az <= fov(2);
            m_sensor_free(withinR & withinFov) = Pd;
            
            [m_occ, m_free] = getEvidences(obj);
            
            if ~isempty(xyGrid)
                startPoint = sensorPos(1:2)';
                endPoint = xyGrid;

                endPts = local2gridImpl(obj,xyGrid);
                valid = endPts(:,1) <= obj.GridSize(1) & endPts(:,1) > 0 & endPts(:,2) <= obj.GridSize(2) & endPts(:,2) > 0;
                endPts = endPts(valid,:);


                % A point in endPts means hit
                if ~isempty(endPts)
                    idxEndPts = sub2ind(obj.GridSize,endPts(:,1),endPts(:,2));
                    m_sensor_occ(unique(idxEndPts)) = Pd;
                end

                % Now consider occlusion
                if hasOcc
                    [uniqueaz,uniquer] = computeUniqueBeams(obj,endPoint,startPoint);
                    for i = 1:numel(uniqueaz)
                        az = uniqueaz(i);
                        pti = cast(startPoint + [uniquer(i)*cosd(az) uniquer(i)*sind(az)],'double');
                        ptiend = cast(startPoint + [1.1*maxRange*cosd(az) 1.1*maxRange*sind(az)],'double');
                        % raycastCells only supports double-precision
                        % inputs
                        [~, midi] = fusion.internal.raycastCells(pti, ptiend, rows, cols, r, gridOrigin, false);
                        idx = sub2ind(obj.GridSize,midi(:,1),midi(:,2));
                        m_sensor_free(idx) = 0;
                    end
                end
            end
            
            m_sensor_free = min(1 - m_sensor_occ, m_sensor_free);
            [m_occ, m_free] = fusion.internal.DempsterShaferOccupancyUpdate(m_occ, m_free, m_sensor_occ, m_sensor_free);
            
            obj.OccupiedEvidence = m_occ;
            obj.FreeEvidence = m_free;
        end
        
        function [uqaz, uqr] = computeUniqueBeams(obj, endPt, startPt)
            d = bsxfun(@minus, endPt, startPt);
            r = sqrt(sum(d.^2,2));
            az = atan2d(d(:,2), d(:,1));
            az = az(~isnan(az));
            r = r(~isnan(r));
            % Minimum azimuth difference on the grid
            minAzDiff = atan2d(obj.CellWidth,max(obj.Length,obj.Width));
            [uqaz,~,ic] = fusion.internal.uniquetolcg(az,minAzDiff);
            uqr = accumarray(ic,r,[numel(uqaz) 1],@min);
        end
        
        function nullify(obj)
            obj.OccupiedEvidence(:) = 0;
            
            obj.FreeEvidence(:) = 0;
        end
    end
    
    methods
        function newObj = clone(obj)
            newObj = fusion.internal.MeasurementEvidenceMap(obj.Length,obj.Width,obj.Resolution,obj.GridOriginInLocal,...
                'UseGPU',obj.UseGPU,...
                'ClassToUse',obj.ClassToUse);
            newObj.OccupiedEvidence = obj.OccupiedEvidence;
            newObj.FreeEvidence = obj.FreeEvidence;
            if coder.internal.is_defined(obj.PositionParameters)
                newObj.PositionParameters = obj.PositionParameters;
            end
        end
    end
    
    methods (Static)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'Length','Width','Resolution','GridSize','UseGPU','ClassToUse','GridOriginInLocal'};
        end
    end
end
