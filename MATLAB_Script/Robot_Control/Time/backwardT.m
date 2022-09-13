function success = backwardT(sec, slamAl, scan, velMs, velPu, lidarSu, vel)
    velMs.linear.x = -vel;
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
    
    velMs.linear.x = 0;
    send(velPu,velMs)

    success = isScanAccepted;
end

