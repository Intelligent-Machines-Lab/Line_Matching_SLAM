%Paredes ordenadas na segunda medição 21/11/2022
%Horizontais
%clear all; close all;
% Horizontais
seg1 = [0 14.54; 0 0];
seg2 = [0 14.54; 2.85 2.85];
seg3 = [0 1.2; 1.55 1.55];
% Verticais
seg4 = [0 0; 0 2.85];
seg5 = [14.54 14.54; 0 2.85];
seg6 = [2.43 2.43; 2.85 1.65];
seg7 = [8.36 8.36; 0 1];
seg8 = [8.34 8.34; 2.85 1.85];
% Diagonais
seg9 = [4.15 4.95; 0.98 1.83];
seg10 = [11.02 11.75; 1.9 0.92];
%---------------------------------
seg = [NaN;NaN];
real_segments_global = [seg1 seg seg2 seg seg3 seg seg4 seg seg5 seg seg6 seg seg7 seg seg8 seg seg9 seg seg10];
% real_segments_local = real_segments_global - [0.723274;2.2989];
real_segments_local = real_segments_global - [0.72;2.30];
%real_segments_local = real_segments_global;
% plot(real_segments_global(1,:),real_segments_global(2,:));
plot(real_segments_local(1,:),real_segments_local(2,:),'r--','LineWidth',3); hold on;
plot(real_segments_local(1,:),real_segments_local(2,:),'ro','LineWidth',3,'MarkerFaceColor','r');
% grid on; axis equal;
% axis([-2 16 -2 5]);