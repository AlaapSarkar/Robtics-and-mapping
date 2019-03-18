function testbeacon(serPort)
[cam_angle,cam_dist,cam_col]=CameraSensorCreate(serPort);
while true
    [cam_angle,cam_dist,cam_col]=CameraSensorCreate(serPort);
    Lidar=LidarSensorCreate(serPort);
    disp(cam_dist)
    disp(cam_angle*180/pi)
    if cam_dist>3
        disp('cam_dist greater')
        turnAngle (serPort, .2, (cam_angle));
        SetDriveWheelsCreate(serPort, 0.2, 0.2);
    elseif cam_dist <=0.3
        break
    else
        n=floor(cam_angle*9*227/(4*pi))+341;
        disp(n)
        Lb=Lidar((n-10):(n+10));
        [Lbm_min,Lbd_min]=min(Lb);
        disp([Lbm_min,Lbd_min])
        if Lbm_min==Lidar(n)
            Lbd_min=11;
        end
        vh=(11-Lbd_min);
        Diff_angle=0.4*atan(vh/(cam_dist+1));
        D_angle=cam_angle+Diff_angle;
        a=D_angle*180/pi;
        disp(a)
        turnAngle(serPort,0.2,a);
        SetDriveWheelsCreate(serPort,0.2,0.2);
    end
    SetDriveWheelsCreate(serPort, 0.2, 0.2);
    pause(0.1)
end
disp('KYTJYJ')
if cam_dist<=0.3
    disp('reached')
end
SetDriveWheelsCreate(serPort, 0, 0);
end