function [isSuccess] = SLAM_Node(duration)
    
    isSuccess = 1;
    setenv('ROS_DOMAIN_ID', '30')
    
    
    n = ros2node('MATLAB_SLAM_node', 30);
    
    
    lidarSub = ros2subscriber(n, "/scan","sensor_msgs/LaserScan", "Reliability","besteffort","Durability","volatile","Depth",5);

    
    
    %%%%%%%%%%%%%%%%%%%  SLAM Section  %%%%%%%%%%%%%%%%%%%%%
    maxLidarRange = 3.5;
    mapResolution = 20;
%     maxNumScans = 360;

%     slamAlg = lidarSLAM(mapResolution, maxLidarRange, maxNumScans);
    slamAlg = lidarSLAM(mapResolution, maxLidarRange);
    
    
    slamAlg.LoopClosureThreshold = 210;  
    slamAlg.LoopClosureSearchRadius = 3.5;
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    tic
    before = 0;
    while toc <= duration
    %%%%%%%%%%%%%%%%%%%%%%%    SLAM Section     %%%%%%%%%%%%%%%%%%%%%%%%%%%

        [scanMsge, isScaned] = receive(lidarSub);
        scan_temp = rosReadLidarScan(scanMsge);
        isScanAccepted = addScan(slamAlg, scan_temp);

        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        if(~isScanAccepted || ~isScaned)
            fprintf(2, "Error occured");
            break;
        end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        if(round(toc) ~= before)
    	    disp(['Duration = ', int2str(toc)])
            before = round(toc);
        end
    end
    

    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    
    
    filename = [pwd, '/Maps/map'];

    figure; 

    saveas(show(map), [filename, '.pgm'])
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

