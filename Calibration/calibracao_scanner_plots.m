% CLNJ, 23/11/2022
clear all; close all;
load distances_calibration.mat;
% Gera��o do 1o plot
distances=[[0;0] distances];
erro=distances(2,:)-distances(1,:);
figure(1);
plot(distances(1,:),distances(2,:),'*-r'); hold on;
plot([0 10],[0 10],'-b');
plot(distances(1,:),erro,'*-k');
xlabel('Dist�ncia real [m]'); ylabel('Dist�ncia medida pelo sensor [m]');
title('Dist�ncia real por dist�ncia medida pelo sensor');
legend('Pontos de medida','Sensor ideal','Erro de medida','Location','northwest');
grid on; axis equal;
axis([0 10 0 10]);
set(gca,'Xtick',[0:10]);
% Gera��o do 2o plot
% vq = interp1(x,v,xq)
dist=distances(2,:);
dist_corrigido=interp1(distances(2,:),distances(1,:),dist);
save distances distances
figure(2);
plot(dist,dist_corrigido,'*-r'); hold on;
plot([0 10],[0 10],'b-');
xlabel('Dist�ncia medida pelo sensor [m]'); ylabel('Dist�ncia corrigida [m]');
title('Dist�ncia medida pelo sensor por dist�ncia corrigida');
legend('Pontos de medida','Sensor ideal','Location','northwest');
grid on; axis equal;
axis([0 10 0 10]);
set(gca,'Xtick',[0:10]);
figure(3);
plot(distances(1,:),dist_corrigido,'*-b'); hold on;
xlabel('Dist�ncia real [m]'); ylabel('Dist�ncia medida corrigida [m]');
title('Calibra��o do sensor de dist�ncia');
legend('Pontos de medida','Location','northwest');
grid on; axis equal;
axis([0 10 0 10]);
set(gca,'Xtick',[0:10]);


