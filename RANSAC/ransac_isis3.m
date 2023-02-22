close all;clear all;

% load scans_teste/ScanDataPoints5.mat;
% load scan_teste_isis/teste3/ScanDataPoints2.mat
load ScanDataPoints.mat;
% load ScanDataPoints1.mat;
% load ScanDataPoints2.mat;
% load ScanDataPoints3.mat;
% load ScanDataPoints4.mat;
% load ScanDataPoints5.mat;
% load ScanDataPoints6.mat;
% load ScanDataPoints10.mat;
% load ScanDataPoints17.mat;
% load ScanDataPoints18.mat;
% load ScanDataPoints19.mat;

points = pt_lst';

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
while size(points,1)>20
    [best_sample, best_cost, best_consensus, segment, coeff,inliers,outliers] = line_RANSAC2(points, 1000,0.02);
    line1 = segment;   
%------------------------
    [inlier_points,outlier_detect]=discrete_segment2(segment,inliers,coeff);

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
        end
        L_dist = sqrt((x_min-x_max)^2+(y_min-y_max)^2);
        if round(coeff(1),dec)~=0 && round(coeff(2),dec)==0
            % Obter MAX e MIN de Y
            [y1, i1] = max(points(best_consensus, 2));
            [y2, i2] = min(points(best_consensus, 2));
            x1 = (coeff(3) - coeff(2) * y1) / coeff(1);
            x2 = (coeff(3) - coeff(2) * y2) / coeff(1);
            L_points = sqrt((x1-x2)^2+(y1-y2)^2);
        end
        if round(coeff(2),dec)~=0 && round(coeff(1),dec)==0
            % Obter MAX e MIN de X
            [x1, i1] = max(points(best_consensus, 1));
            [x2, i2] = min(points(best_consensus, 1));
            y1 = (coeff(3) - coeff(1) * x1) / coeff(2);
            y2 = (coeff(3) - coeff(1) * x2) / coeff(2);
            L_points = sqrt((x1-x2)^2+(y1-y2)^2);
        end
        if round(coeff(1),dec)~=0 && round(coeff(2),dec)~=0 
            % Obter MAX e MIN de X
            [y1, i1] = max(points(best_consensus, 2));
            [y2, i2] = min(points(best_consensus, 2));
            x1 = (coeff(3) - coeff(2) * y1) / coeff(1);
            x2 = (coeff(3) - coeff(2) * y2) / coeff(1);
            L_points = sqrt((x1-x2)^2+(y1-y2)^2);
            if L_points>L_dist
                [x1, i1] = max(points(best_consensus, 1));
                [x2, i2] = min(points(best_consensus, 1));
                y1 = (coeff(3) - coeff(1) * x1) / coeff(2);
                y2 = (coeff(3) - coeff(1) * x2) / coeff(2);
            end
        end
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
    
    if isempty(line1)
        line_segment = [line_segment, [segment_X_min segment_X_max;segment_Y_min segment_Y_max]];
    else
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
% save segments_sc line_segment pose_gt;
save segments_sc line_segment;

function [real_inlier,real_outlier]=discrete_segment2(new_segment,new_inliers,coeff)
    L_segment = sqrt((new_segment(1,1)-new_segment(2,1))^2+(new_segment(1,2)-new_segment(2,2))^2);
%     L = 1*size(new_inliers,1)/3;
    L = 2*size(new_inliers,1);
    y = linspace(new_segment(1,2),new_segment(2,2),L);
    x = linspace(new_segment(1,1),new_segment(2,1),L);
    points = [x;y]';

    if coeff(1)~=0
        %Ordenar inliers
            y_sort = sort(new_inliers(:,2));
            for i = 1:length(y_sort)
                index = find(new_inliers(:,2)==y_sort(i));
                x_sort(i,:) = new_inliers(index,1);
            end
            inlier_sort = [x_sort,y_sort];
    elseif coeff(2)~=0
        %Ordenar inliers
            x_sort = sort(new_inliers(:,1));
            for i = 1:length(x_sort)
                index = find(new_inliers==x_sort(i));
                y_sort(i,:) = new_inliers(index,2);
            end
            inlier_sort = [x_sort,y_sort];
    end
    
%     [Idx,D] = knnsearch(points,inlier_sort);
    if size(new_inliers,1)<10
        Idx = rangesearch(new_inliers,points,0.2);
%         Idx = rangesearch(new_inliers,points,0.4);
    else
        Idx = rangesearch(new_inliers,points,0.1);
%         Idx = rangesearch(new_inliers,points,0.3);
    end
    
    for i = 1:size(Idx,1)
        if isempty(Idx{i,1})
            consensus(i,1) = 0;
        else
            consensus(i,1) = 1;
        end
    end
    
    aux3=1;
    aux4=1;
    for i = 1:size(consensus,1)-1
        if consensus(i,1)>0 && consensus(i+1,1)>0
            teste(i,1) = aux3;
            teste(i+1,1) = aux3;
            aux3 = aux3+0;
        end
        if consensus(i,1)>0 && consensus(i+1,1)==0
            teste(i,1) = aux3;
            teste(i+1,1) = 0;
            aux3 = aux3+1;
        end
        if consensus(i,1)==0 && consensus(i+1,1)>0
            teste(i,1) = 0;
            teste(i+1,1) = aux3;
            aux3 = aux3+0;
        end
        if consensus(i,1)==0 && consensus(i+1,1)==0
            teste(i,1) = 0;
            teste(i+1,1) = 0;
            aux3 = aux3+0;
        end
    end
    cont = 1;
    for i = 1:size(teste,1)
        if teste(i)>0
            teste1(cont,1)=teste(i,1);
            cont = cont+1;
        end
    end
    v = unique(teste);
    v1 = unique(teste1);
    c = hist(teste1,1:max(teste1));
    number_max_points = max(c);
    index_number_max_points = find(c==number_max_points);
    ind = v1(index_number_max_points,1);
    ind2 = find(teste==ind(1));
    
real_consensus = zeros(size(consensus,1),1);
aux2 = 0;
    if size(consensus,1)>5
        for i=1:size(consensus,1)
            for j=1:size(ind2,1)
                if i==ind2(j)
                    real_consensus(i,1) = 1;
                end
            end
        end        
    else
       real_consensus = ones(size(consensus,1),1); 
    end

%----------------------------------------------------------
    index_array=[];
    index_array2 = [];
    for i = 1:size(Idx,1)
        if real_consensus(i,1)>0
            index_array = Idx{i,1};
            index_array2 = [index_array2,[index_array]];
        end
    end
    real_index = unique(index_array2)';
    real_consensus_function = zeros(size(new_inliers,1),1);
    aux_in=1;
    aux_out=1;
    real_inlier = [];
    real_outlier = [];
    for i = 1:size(new_inliers,1)
        for j = 1:size(real_index,1)
            if i == real_index(j,1)
                real_inlier(aux_in,:) = new_inliers(i,:);
                real_consensus_function(i,1) = 1;
                aux_in=aux_in+1;
            end
        end
    end

    for i = 1:size(real_consensus_function,1)
       if real_consensus_function(i,1)==0
           real_outlier(aux_out,:)=new_inliers(i,:);
           aux_out=aux_out+1;
       end
    end

    figure(3)
    plot(x,y,'*')
    hold on
    plot(new_inliers(:,1),new_inliers(:,2),'o')
    hold off
end


function [new_segments,new_corner]=corner_detection2(line_segment,coeff_seg,points)
    segments = double(line_segment);
    coeffs = double(coeff_seg);
    N = size(segments,2)/2;
    aux = 1;
    segment_comparation = zeros(0,5);
    for i = 1:N
        for j = 1:N
            if i~=j
                for k=1:2
                    if k==1
                        segment1_endpoint = segments(:,2*i-1);
                    else
                        segment1_endpoint = segments(:,2*i);
                    end
                    for l=1:2
                        if l==1
                            segment2_endpoint = segments(:,2*j-1);
                        else
                            segment2_endpoint = segments(:,2*j);
                        end
                        distance_endpoint(aux) = sqrt((segment1_endpoint(1,1)-segment2_endpoint(1,1))^2+(segment1_endpoint(2,1)-segment2_endpoint(2,1))^2); 
%                         distance_endpoint(aux) = abs((segments(1,2*j) - segments(1,2*j-1))*(segments(2,2*j-1)-segment1_endpoint(2,1)) - (segments(1,2*j-1) - segment1_endpoint(1,1))*(segments(2,2*j) - segments(2,2*j-1)))/(sqrt((segments(1,2*j) - segments(1,2*j-1))^2 + (segments(2,2*j) - segments(2,2*j-1))^2));
%                         distance_endpoint(aux)=abs(coeffs(j,1)*segment1_endpoint(1,1)+coeffs(j,2)*segment1_endpoint(2,1)+coeffs(j,3))/sqrt(coeffs(j,1)^2 + coeffs(j,2)^2);    
                        aux = aux+1;
                    end
                    aux=1;
                    min_distance = min(distance_endpoint);
                    index = find(distance_endpoint==min_distance);
                    segment_comparation = [segment_comparation;[i,j,k,index,min_distance]];               
                end
             end
        end
    end
    aux2=1;
    aux3=1;
    segment_comparated = zeros(0,5);
    for i = 1:N
        aux2=1;
        comparation = segment_comparation(:,1)==i;
        for j = 1:size(segment_comparation,1)
            if comparation(j,1)==1            
                if segment_comparation(j,3)==1
                    distance(2*i-1,aux2) = segment_comparation(j,5); 
                else
                    distance(2*i,aux2) = segment_comparation(j,5); 
                    aux2 = aux2+1;
                end
            end            
        end
        distance_min_seg_endpoint1 = min(distance(2*i-1,:));
        distance_min_seg_endpoint2 = min(distance(2*i,:));
        index_comparation_endpoint1 = find(segment_comparation(:,5)==distance_min_seg_endpoint1 & segment_comparation(:,1)==i);
        index_comparation_endpoint2 = find(segment_comparation(:,5)==distance_min_seg_endpoint2 & segment_comparation(:,1)==i);
        segment_comparated(aux3,:)=segment_comparation(index_comparation_endpoint1,:);
        segment_comparated(aux3+1,:)=segment_comparation(index_comparation_endpoint2,:);
        aux3 = aux3+2;
    end
    consensu = segment_comparated(:,5)<0.2;
    cont=1;
    for i=1:size(segment_comparated,1)
        if consensu(i,1)==1
            segment_comparated2(cont,:) = segment_comparated(i,:);
            cont = cont+1;
        end   
    end
    

    %Interseccao dos segmentos
    delta = 0.5;
    tol = 1e-2;
    for i = 1:N
        if segments(1,2*i-1)>segments(1,2*i) && abs(segments(1,2*i-1)-segments(1,2*i))>tol
            x1 = segments(1,2*i-1)+delta;
            x2 = segments(1,2*i)-delta;
            y1 = (coeffs(i,3)-coeffs(i,1)*x1)/coeffs(i,2);
            y2 = (coeffs(i,3)-coeffs(i,1)*x2)/coeffs(i,2);
        elseif segments(1,2*i-1)<segments(1,2*i) && abs(segments(1,2*i-1)-segments(1,2*i))>tol
            x1 = segments(1,2*i-1)-delta;
            x2 = segments(1,2*i)+delta;
            y1 = (coeffs(i,3)-coeffs(i,1)*x1)/coeffs(i,2);
            y2 = (coeffs(i,3)-coeffs(i,1)*x2)/coeffs(i,2);
        elseif abs(segments(1,2*i-1)-segments(1,2*i))<tol
            if segments(2,2*i-1)>segments(2,2*i)
                y1 = segments(2,2*i-1)+delta;
                y2 = segments(2,2*i)-delta;
                x1 = (coeffs(i,3)-coeffs(i,2)*y1)/coeffs(i,1);
                x2 = (coeffs(i,3)-coeffs(i,2)*y2)/coeffs(i,1);
            else
                y1 = segments(2,2*i-1)-delta;
                y2 = segments(2,2*i)+delta;
                x1 = (coeffs(i,3)-coeffs(i,2)*y1)/coeffs(i,1);
                x2 = (coeffs(i,3)-coeffs(i,2)*y2)/coeffs(i,1);
            end            
        end
        big_new_segments(:,2*i-1:2*i) =[x1,x2;y1,y2]; 
    end
    
%     new_segments = zeros(2,2*N);
%     corner = zeros(0,2);
%     cont2=1;
    new_segments=segments;
    for i = 1:size(segment_comparated2,1)
        seg1 = segment_comparated2(i,1);
        seg2 = segment_comparated2(i,2);
        end_point1 = segment_comparated2(i,3);
        end_point2 = segment_comparated2(i,4);
        [xi,yi] = polyxpoly(big_new_segments(1,2*seg1-1:2*seg1),big_new_segments(2,2*seg1-1:2*seg1),big_new_segments(1,2*seg2-1:2*seg2),big_new_segments(2,2*seg2-1:2*seg2));
%         corner(cont2,:) = [xi,yi];
%         cont2=cont2+1;
        if end_point1==1
            new_segments(:,2*seg1-1)=[xi;yi];
        else
            new_segments(:,2*seg1)=[xi;yi];
        end
        if end_point2==1
            new_segments(:,2*seg2-1)=[xi;yi];
        else
            new_segments(:,2*seg2)=[xi;yi];
        end
    end
%     [new_corner, ia, ic] = unique(corner,'rows','stable');
    aux4=1;
    figure(4)
    plot(points(:,1),points(:,2),'bo');
    leg = 'Scan Points';
    leg4{aux4,1} = leg;
    hold on;
    for i = 1:N
        plot(new_segments(1,(2*i-1):(2*i)),new_segments(2,(2*i-1):(2*i)),'LineWidth',2);
        leg3 = ['Segmento ',num2str(aux4)];
        leg3 = string(leg3);
        leg4{aux4+1,1} = leg3;
        aux4 = aux4+1;
        hold on;
    end
    grid on;
    axis equal;xlabel('x (m)');ylabel('y (m)');
    title('Segmentos ajustados após algoritmo de RANSAC');
    legend(leg,leg4);
    hold off;
end