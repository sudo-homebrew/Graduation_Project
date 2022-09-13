function success = leftT(sec, slamAl, scan, velMs, velPu, lidarSu, vel)
    velMs.angular.z = vel;
    send(velPu,velMs)
    tic;
    
    while toc < sec
      scanMsge = receive(lidarSu);
      rosPlot(scanMsge)
      scan_temp = rosReadLidarScan(scanMsge);
      scan = [scan, scan_temp];
      [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAl, scan_temp)

      if(~isScanAccepted)
          fprintf("Error occured");
          break;
      end
    end
    
    velMs.angular.z = 0;
    send(velPu,velMs)

    success = isScanAccepted;
end
