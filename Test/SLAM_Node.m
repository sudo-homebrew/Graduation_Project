function [isSuccess] = SLAM_Node(duration)
    
    isSuccess = 1;
    setenv('ROS_DOMAIN_ID', '30')
    
    
    n = ros2node('MATLAB_SLAM_node', 30);
    
    
    lidarSub = ros2subscriber(n, "/scan","sensor_msgs/LaserScan", "Reliability","besteffort","Durability","volatile","Depth",5);

    
    
    %%  SLAM Section  %%
%     sc = {};
    maxLidarRange = 8;
    mapResolution = 20;
    maxNumScans = 360;

    slamAlg = lidarSLAM(mapResolution, maxLidarRange, maxNumScans);
    
    
    slamAlg.LoopClosureThreshold = 210;  
    slamAlg.LoopClosureSearchRadius = 8;
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Pause Option
    % pause(4)
    
    
    
    

    tic
%     cnt = 0;
    
    while toc <= duration
    %%%%%%%%%%%%%%%%%%%%%%%    SLAM Section     %%%%%%%%%%%%%%%%%%%%%%%%%%%

        [scanMsge, isScaned] = receive(lidarSub);

%         before = toc;
        scan_temp = rosReadLidarScan(scanMsge);
%         sc = [sc, scan_temp];
        isScanAccepted = addScan(slamAlg, scan_temp);

        [scans, optimizedPoses]  = scansAndPoses(slamAlg);
        map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
%         occMatrix = getOccupancy(map);
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        if(~isScanAccepted || ~isScaned)
            fprintf(2, "Error occured");
            break;
        end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    end
    
    

    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    
    
    filename = ['./Maps/map'];

    figure; 
%     h = show(map);

    saveas(show(map), [filename, '.pgm'])
%     show(map)
    data.image = [filename, '.pgm'];
    data.mode = 'trinary';
    data.resolution = map.Resolution;
    data.origine = map.LocalOriginInWorld;
    data.origine = [data.origine, 0];
    data.negate = 0;
    data.occupied_thresh = map.OccupiedThreshold;
    data.free_thresh = map.FreeThreshold;

    yaml.dumpFile(filename + ".yaml", data)
end
