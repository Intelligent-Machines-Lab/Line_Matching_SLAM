close all;clear all;

% load scans_teste/ScanDataPoints5.mat;
% load scan_teste_isis/teste3/ScanDataPoints2.mat
% load teste_isis_corredor/ScanDataPoints35.mat
% load ScanDataPoints.mat;
% load ScanDataPoints1.mat;
% load ScanDataPoints2.mat
% load ScanDataPoints3.mat;
% load ScanDataPoints4.mat;
% load ScanDataPoints5.mat;
% load ScanDataPoints6.mat;
% load ScanDataPoints10.mat;
% load ScanDataPoints17.mat;
% load ScanDataPoints18.mat;
% load ScanDataPoints19.mat;

fprintf('\n ==> Passo 1: Carregamento dos Dados Reais');
NScans=1;
for iScan=1:NScans 
    str_iScan=num2str(iScan);
    eval(['load teste_isis_corredor/teste7_calibracao_sensor/ScanDataPointsCorrigido' str_iScan '.mat;']);
   


points = pt_lst';
inliers = points;

figure(1)
plot(points(:,1),points(:,2),'ro');
grid on; 
axis equal;xlabel('x (m)');ylabel('y (m)');
title('Pontos do Scan de P1 em SC0');

line_segment=zeros(2,0);
coeff_seg = zeros(0,3);
% Variavel auxiliar
aux = 1;
aux1 = 1;

save_inliers = struct;
save_inliers.inliers = cell(0,0);
outlier_detect = [];

figure(2)
plot(points(:,1),points(:,2),'bo');
leg = 'Scan Points';
leg2{aux,1} = leg;
grid on; hold on;
while size(points,1)>25 && size(inliers,1)>3
%     [best_sample, best_cost, best_consensus, segment, coeff,inliers,outliers] = line_RANSAC4(points, 1000,0.035);
%     [best_sample, best_cost, best_consensus, segment, coeff,inliers,outliers] = line_RANSAC4(points, 1000,0.025);
%     [best_sample, best_cost, best_consensus, segment, coeff,inliers,outliers] = line_RANSAC4(points, 1000,0.023);
    [best_sample, best_cost, best_consensus, segment, coeff,inliers,outliers] = line_RANSAC4(points, 1000,0.02);
% [best_sample, best_cost, best_consensus, segment, coeff,inliers,outliers] = line_RANSAC4(points, 1000,0.019);
    line1 = segment;   
%------------------------
    [inlier_points,outlier_detect]=discrete_segment3(segment,inliers,coeff);

    %---------------------
%     [inlier_points,outlier_detect]=detect_real_segment(line1,inliers);
%     inliers = inlier_points;
    if ~isempty(outlier_detect)
        dec = 3; %casa decimal apos a virgula
        inliers = inlier_points;
        for i = 1:size(outlier_detect,1)
            index_out = find(points(:,1)==outlier_detect(i,1) & points(:,2)==outlier_detect(i,2));
            best_consensus(index_out,1) = 0;
        end    
        index_o = 1;
        index_i = 1;
        outliers = [];
        inliers = [];
        for i = 1:size(best_consensus,1)
            if best_consensus(i,1)==1
               inliers(index_i,:) = points(i,:);
               index_i = index_i+1;
            else
               outliers(index_o,:) = points(i,:);
               index_o = index_o+1;
            end
        end
        if round(coeff(1),dec)~=0 && round(coeff(2),dec)~=0
            if round(coeff(1),0)==0 && round(coeff(2),0)~=0
                x_min = min(inliers(:,1));
                x_max = max(inliers(:,1));
                x2=x_min;
                x1=x_max;
                for i = 1:size(inliers,1)
                    if inliers(i,1)==x_min
                        y_min = inliers(i,2);
                    end
                end
                for i = 1:size(inliers,1)
                    if inliers(i,1)==x_max
                        y_max = inliers(i,2);
                    end
                end
                y2=y_min;
                y1=y_max;
            elseif round(coeff(1),0)~=0 && round(coeff(2),0)==0
                y_min = min(inliers(:,2));
                y_max = max(inliers(:,2));
                for i = 1:size(inliers,1)
                    if inliers(i,2)==y_min
                        x_min = inliers(i,1);
                    end
                end
                for i = 1:size(inliers,1)
                    if inliers(i,2)==y_max
                        x_max = inliers(i,1);
                    end
                end
                x2=x_min;
                x1=x_max;
                y2=y_min;
                y1=y_max;
            else
                x_min = min(inliers(:,1));
                x_max = max(inliers(:,1));
                x2=x_min;
                x1=x_max;
                for i = 1:size(inliers,1)
                    if inliers(i,1)==x_min
                        y_min = inliers(i,2);
                    end
                end
                for i = 1:size(inliers,1)
                    if inliers(i,1)==x_max
                        y_max = inliers(i,2);
                    end
                end
                y2=y_min;
                y1=y_max;
            end
        elseif round(coeff(2),dec)==0
            y_min = min(inliers(:,2));
            y_max = max(inliers(:,2));
            for i = 1:size(inliers,1)
                if inliers(i,2)==y_min
                    x_min = inliers(i,1);
                end
            end
            for i = 1:size(inliers,1)
                if inliers(i,2)==y_max
                    x_max = inliers(i,1);
                end
            end
            x2=x_min;
            x1=x_max;
            y2=y_min;
            y1=y_max;
        elseif round(coeff(1),dec)==0
            x_min = min(inliers(:,1));
            x_max = max(inliers(:,1));
            for i = 1:size(inliers,1)
                if inliers(i,1)==x_min
                    y_min = inliers(i,2);
                end
            end
            for i = 1:size(inliers,1)
                if inliers(i,1)==x_max
                    y_max = inliers(i,2);
                end
            end
            x2=x_min;
            x1=x_max;
            y2=y_min;
            y1=y_max;
        end
        L_dist = sqrt((x_min-x_max)^2+(y_min-y_max)^2);

        line1 = [x1, y1; x2, y2];
    end
    %---------------------
    save_inliers.inliers{aux1,1} = inliers;
    aux1 = aux1+1;
    tol = 1e-4;
    segment_X_min = min(inliers(:,1));
    segment_X_max = max(inliers(:,1));
    if abs(segment_X_min - segment_X_max)<tol
        segment_Y_min = min(inliers(:,2));
        segment_Y_max = max(inliers(:,2));
    else
        index = find(inliers(:,1)==segment_X_min);
        segment_Y = inliers(index,2);
        segment_Y_min = segment_Y;
        index2 = find(inliers(:,1)==segment_X_max);
        segment_Y = inliers(index2,2);
        segment_Y_max = segment_Y;
    end
    dist1 = sqrt((segment_X_min-segment_X_max)^2+(segment_Y_min-segment_Y_max)^2);
    dist2 = sqrt((line1(1,1)-line1(2,1))^2+(line1(1,2)-line1(2,2))^2);
    if isempty(line1) && dist1>0.7
        line_segment = [line_segment, [segment_X_min segment_X_max;segment_Y_min segment_Y_max]];
    elseif dist2>0.7
        line_segment = [line_segment, [line1(1,1) line1(2,1);line1(1,2) line1(2,2)]];
    end
    points = outliers;
    coeff_seg = [coeff_seg; [coeff(1) coeff(2) coeff(3)]];

    %Plot dos segmentos de reta no grafico de pontos do Scan de P
    figure(2)
    hold on;
    if isempty(line1)
        plot(inliers(:,1),inliers(:,2),'LineWidth',2);
        leg1 = ['Segmento ',num2str(aux)];
        leg1 = string(leg1);
        leg2{aux+1,1} = leg1;
        aux = aux+1;
    else
        plot(line1(:,1),line1(:,2),'LineWidth',2);
        leg1 = ['Segmento ',num2str(aux)];
        leg1 = string(leg1);
        leg2{aux+1,1} = leg1;
        aux = aux+1;
    end
    hold on
end
axis equal;xlabel('x (m)');ylabel('y (m)');
title('Segmentos gerados pelo algoritmo de RANSAC');
legend(leg,leg2);
% axis([-inf inf -inf inf]);
hold off;

new_line_segment=corner_detection2(line_segment,coeff_seg,pt_lst');
line_segment=new_line_segment;
corner = obtain_corners(line_segment);
% save segments_sc line_segment pose_gt;
% save segments_sc line_segment;
eval(['save teste_isis_corredor/teste7_calibracao_sensor/scan' str_iScan '.mat line_segment pose_gt;']);
% pause(1);
% close all; 
% clear leg leg1 leg2;
end
