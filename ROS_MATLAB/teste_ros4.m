close all; clearvars;
load speed1.mat
% rosinit
% topic_scan_sim = 'scan';
% topic_odom_sim = 'odom';
% topic_cmd_sim = 'cmd_vel';

aux=1;
for i = 1:size(u,2)

pause(1)

rosinit
topic_scan_sim = 'scan';
topic_odom_sim = 'odom';
topic_cmd_sim = 'cmd_vel';

% u = [vx; wz], dt = t (seg)
% u = [1; -0.5];
% u = [0; 0];
% u = [0; 0.2];
% u = [0; -0.2];
% u = [0; -0.02];

% u = [0; -0.2];
% u = [0.5; 0];
% u = [0.0005; 0];
% u = [0; -0.0005];
% u = [0; -0.55];
% u = [1; 0];
% u = [0; 0.4];
% u = [0; 0.5];
% u = [0; 1.5];
% u = [0; 1];
% u = [0; -1];
% u = [1.5; 0];
% u = [2; 0];
% u = [0.8; 0];
% u = [0.5; 0];

dt = 1;
send_command(topic_cmd_sim, u(:,i), dt);

pt_lst = receive_scan(topic_scan_sim);
pause(0.2);
pose_gt = receive_pose(topic_odom_sim);
pause(0.2);
disp(['X: ', num2str(pose_gt(1)), ', Y: ', num2str(pose_gt(2)), ', Th: ', num2str(rad2deg(pose_gt(3)))])


if u(2,i)==0
    aux1=num2str(aux);
    eval(['save scans_teste/teste5/ScanDataPoints' aux1 ' pt_lst pose_gt;']);
    aux=aux+1;
%     pause

end
% save ScanDataPoints pt_lst pose_gt;

% Gráfico do Scan do Robô em relacao a pose dele (SC do robô)
figure
plot(pt_lst(1, :), pt_lst(2, :),'.', 'MarkerSize', 8)
grid on
axis equal
title('Scan no SC do robô');xlabel('x');ylabel('y');

% end
rosshutdown
end
%-------------------------------------------------------------------------
function send_command(topic_name, u, dt)
    pause(0.2);
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
    [msg, status, statustext] = receive(sub, 10);
    pause(0.1);
    if status
        ranges = msg.Ranges';
        angles = msg.AngleMin:msg.AngleIncrement:msg.AngleMax;
        
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
        x = msg.Pose.Pose.Position.X;
        y = msg.Pose.Pose.Position.Y;
        qx = msg.Pose.Pose.Orientation.X;
        qy = msg.Pose.Pose.Orientation.Y;
        qz = msg.Pose.Pose.Orientation.Z;
        qw = msg.Pose.Pose.Orientation.W;
        eul = quat2eul([qw, qx, qy, qz]);
        theta = eul(1);
        
        pose = [x; y; theta];
    else
        pose = [];
    end
end