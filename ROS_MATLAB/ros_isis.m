%ros_isis.m
close all; clearvars;

rosinit
topic_scan_sim = 'scan';
topic_yaw_sim = 'yaw'; %Angle in degrees
topic_cmd_sim = 'cmd_vel';
u = [0; 0];
dt=1;

send_command(topic_cmd_sim, u, dt);

pt_lst = receive_scan(topic_scan_sim);
pose_gt = receive_pose(topic_yaw_sim);

% save teste_isis_corredor/teste4/ScanDataPoints82 pt_lst pose_gt;
save ScanDataPoints pt_lst pose_gt;

figure
plot(pt_lst(1, :), pt_lst(2, :),'.', 'MarkerSize', 5)
grid on
axis equal
title('Scan no SC do robô');xlabel('x');ylabel('y');

rosshutdown

function send_command(topic_name, u, dt)
    pub = rospublisher(topic_name, 'geometry_msgs/Twist', 'DataFormat', 'struct');
    msg = rosmessage(pub);

    msg.Linear.X = u(1);
    msg.Angular.Z = u(2);
    send(pub, msg);

    pause(dt);

    msg.Linear.X = 0;
    msg.Angular.Z = 0;
    send(pub, msg);

end

function point_list = receive_scan(topic_name)
    sub = rossubscriber(topic_name, 'DataFormat', 'struct');
    pause(1)
    [msg, status, statustext] = receive(sub, 10);
    pause(0.1);
    if status
        ranges = msg.Ranges';
        angles = (msg.AngleMin+pi):msg.AngleIncrement:(msg.AngleMax+pi);
        
        [x, y] = pol2cart(angles,ranges);
        point_list = [x; y];
    else
        point_list = [];
    end
end

function pose = receive_pose(topic_name)
    sub = rossubscriber(topic_name, 'DataFormat', 'struct');
    [msg, status, statustext] = receive(sub, 10);
    pause(0.1);
    if status
        theta = deg2rad(msg.Data); %Degrees to rad
   
%         x = msg.Pose.Pose.Position.X;
%         y = msg.Pose.Pose.Position.Y;
%         qx = msg.Pose.Pose.Orientation.X;
%         qy = msg.Pose.Pose.Orientation.Y;
%         qz = msg.Pose.Pose.Orientation.Z;
%         qw = msg.Pose.Pose.Orientation.W;
%         eul = quat2eul([qw, qx, qy, qz]);
%         theta = eul(1);
        pose = theta;
%         pose = [x; y; theta];
    else
        pose = [];
    end
end
