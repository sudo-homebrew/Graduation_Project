classdef Error < handle%
%

%   Copyright 2018 The MathWorks, Inc.
        
    properties
        IdentifierFcn
        InvalidID
        IDLabel
    end
    
    properties (Access = private)
        LastID
        LastCount
        LastMetrics
        CumulativeID
        CumulativeCount
        CumulativeMetrics
    end
    
    methods
        function loadError(obj, s)
            obj.LastID = s.LastID;
            obj.LastCount = s.LastCount;
            obj.LastMetrics = s.LastMetrics;
            obj.CumulativeID = s.CumulativeID;
            obj.CumulativeCount = s.CumulativeCount;
            obj.CumulativeMetrics = s.CumulativeMetrics;
        end
        
        function s = saveError(obj)
            s.LastID = obj.LastID;
            s.LastCount = obj.LastCount;
            s.LastMetrics = obj.LastMetrics;
            s.CumulativeID = obj.CumulativeID;
            s.CumulativeCount = obj.CumulativeCount;
            s.CumulativeMetrics = obj.CumulativeMetrics;
        end
        
        function resetMetrics(obj,numResults)
            obj.LastID            = repmat(obj.InvalidID,0,numResults);
            obj.LastCount         = zeros(0,1);
            obj.LastMetrics       = zeros(0,numResults);
            obj.CumulativeID      = repmat(obj.InvalidID,0,numResults);
            obj.CumulativeCount   = zeros(0,1);
            obj.CumulativeMetrics = zeros(0,numResults);
        end
        
        function tm = buildTable(obj, id, m, fields, isbuiltin)
            if isbuiltin
                % perform RMSE 
                m(:,1:end/2) = sqrt(m(:,1:end/2));
            end
            
            t1 = table(id(:),'VariableNames',{obj.IDLabel});
            t2 = array2table(m,'VariableNames',fields);
            tm = horzcat(t1,t2);
        end

        function tm = currentMetricsTable(obj, fields, isbuiltin)
            m = obj.LastMetrics./obj.LastCount;
            id = obj.LastID;
            tm = buildTable(obj, id, m, fields, isbuiltin);
        end
        
        function tm = cumulativeMetricsTable(obj, fields, isbuiltin)
            m = obj.CumulativeMetrics./obj.CumulativeCount;
            id = obj.CumulativeID;
            tm = buildTable(obj, id, m, fields, isbuiltin);
        end
        
        function current = selectCurrent(obj, array, currentID)
            arrayID = getArrayIdentifer(obj, array);
            [isMember,iTrack] = ismember(currentID, arrayID);
            if ~all(isMember)
                error(message('fusion:trackErrorMetrics:MissingIdentifiers',obj.IDLabel));
            end
            current = array(iTrack(isMember));
        end
        
        
        function updateErrors(obj, results, currentID)
            if ~isempty(currentID)
                updateLastErrors(obj, results, currentID);
                updateCumulativeErrors(obj, results, currentID);
            end
        end

        function updateLastErrors(obj, metrics, currentID)
            uniqueID = unique(currentID);
            n = size(metrics,2);
            m = numel(uniqueID);
            [~,iUnique] = ismember(currentID, uniqueID);
            
            subs = zeros(numel(metrics),2);
            subs(:,1) = repmat(iUnique(:),n,1);
            cols = (1:n) + zeros(size(metrics,1),1);
            subs(:,2) = cols(:);
            
            obj.LastMetrics = accumarray(subs, metrics(:), [m n]);
            obj.LastCount = accumarray(subs, 1, [m n]);
            obj.LastID = uniqueID;
        end
        
        function updateCumulativeErrors(obj, metrics, currentID)
            % copy over class information if empty
            if isempty(obj.CumulativeID)
                obj.CumulativeID = repmat(currentID(1), 0, 1);
            end
            
            % create union of both pre-existing and current track ID
            allID = union(obj.CumulativeID, currentID);
            n = size(metrics,2);
            m = numel(allID);
            
            % copy over current information
            [~,iAll] = ismember(obj.CumulativeID,allID);
            allCount = zeros(m,n);
            allMetrics = zeros(m,n);
            allMetrics(iAll,:) = obj.CumulativeMetrics;
            allCount(iAll,:) = obj.CumulativeCount;
                        
            % generate accum subscripts
            [~,iAll] = ismember(currentID, allID);
            subs = zeros(numel(metrics),2);
            subs(:,1) = repmat(iAll(:),n,1);
            cols = (1:n) + zeros(size(metrics,1),1);
            subs(:,2) = cols(:);
            
            % accumulate with current
            allMetrics = allMetrics + accumarray(subs, metrics(:),[m n]);
            allCount = allCount + accumarray(subs,1,[m n]);
            
            % assign to output
            obj.CumulativeID = allID;
            obj.CumulativeCount = allCount;
            obj.CumulativeMetrics = allMetrics;
        end
    end
        
    methods (Access = protected)
        function id = getArrayIdentifer(obj, array)
            id = obj.IdentifierFcn(array);
            if numel(id) ~= numel(array) || ~isstring(id) && ~isnumeric(id)
                error(message('fusion:trackErrorMetrics:InvalidIDResult',char(obj.IdentifierFcn)));
            end
        end
    end
end
