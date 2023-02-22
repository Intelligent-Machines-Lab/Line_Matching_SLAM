function [new_segments]=corner_detection2(line_segment,coeff_seg,points)
    segments = double(line_segment);
    coeffs = double(coeff_seg);
    
    for i=1:size(segments,2)/2
        L(i,:) = sqrt((segments(1,2*i-1)-segments(1,2*i))^2 + (segments(2,2*i-1)-segments(2,2*i))^2);
    end
    aux_L=1;
    L_consensu = L>0.15;
    for i=1:length(L_consensu)
        if L_consensu(i,1)>0
            segments2(:,(2*aux_L-1):(2*aux_L)) = segments(:,(2*i-1):(2*i));
            coeffs2(aux_L,:) = coeffs(i,:);
            aux_L=aux_L+1;
        end
    end
    segments=segments2;
    coeffs=coeffs2;
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
%     segment_comparation_end_point = zeros(0,4);
    aux4=1;
    for i = 1:N
        for j = 1:N
            if i~=j
                for k=1:2
                    if k==1
                        segment1_endpoint = segments(:,2*i-1);
                    else
                        segment1_endpoint = segments(:,2*i);
                    end
                    distance_endpoint_to_segment=abs(coeffs(j,1)*segment1_endpoint(1,1)+coeffs(j,2)*segment1_endpoint(2,1)-coeffs(j,3))/sqrt(coeffs(j,1)^2 + coeffs(j,2)^2);    
                    segment_comparation(aux4,6)=distance_endpoint_to_segment;
                    aux4=aux4+1;
%                     min_distance_endpoint_to_segment = min(distance_endpoint_to_segment);
%                     index = find(distance_endpoint_to_segment==min_distance_endpoint_to_segment);
%                     segment_comparation_end_point = [segment_comparation_end_point;[i,j,k,distance_endpoint_to_segment]];               
                end
             end
        end
    end
    

    aux2=1;
    aux3=1;
    segment_comparated = zeros(0,6);
%     segment_comparated_endpoint = zeros(0,4);
    for i = 1:N
        aux2=1;
        comparation = segment_comparation(:,1)==i;
%         comparation2 = segment_comparation_end_point(:,1)==i;
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
    consensu = segment_comparated(:,6)<0.45;
    consensu2 = segment_comparated(:,5)<0.45;
    cont=1;
    for i=1:size(segment_comparated,1)
        if consensu(i,1)==1 && consensu2(i,1)==1 
            segment_comparated2(cont,:) = segment_comparated(i,:);
            cont = cont+1;
        end   
    end
    

    for i = 1:N
        [a,b,c]=fit_line_least_squares(segments(:,(2*i-1):(2*i))');
        coeffs3(i,:) = [a,b,c]; 
    end

    %Interseccao dos segmentos
    delta = 0.4;
    tol = 1e-2;
    for i = 1:N
        if segments(1,2*i-1)>segments(1,2*i) && abs(segments(1,2*i-1)-segments(1,2*i))>tol
            x1 = segments(1,2*i-1)+delta;
            x2 = segments(1,2*i)-delta;
            y1 = (coeffs3(i,3)-coeffs3(i,1)*x1)/coeffs3(i,2);
            y2 = (coeffs3(i,3)-coeffs3(i,1)*x2)/coeffs3(i,2);
        elseif segments(1,2*i-1)<segments(1,2*i) && abs(segments(1,2*i-1)-segments(1,2*i))>tol
            x1 = segments(1,2*i-1)-delta;
            x2 = segments(1,2*i)+delta;
            y1 = (coeffs3(i,3)-coeffs3(i,1)*x1)/coeffs3(i,2);
            y2 = (coeffs3(i,3)-coeffs3(i,1)*x2)/coeffs3(i,2);
        elseif abs(segments(1,2*i-1)-segments(1,2*i))<tol
            if segments(2,2*i-1)>segments(2,2*i)
                y1 = segments(2,2*i-1)+delta;
                y2 = segments(2,2*i)-delta;
                x1 = (coeffs3(i,3)-coeffs3(i,2)*y1)/coeffs3(i,1);
                x2 = (coeffs3(i,3)-coeffs3(i,2)*y2)/coeffs3(i,1);
            else
                y1 = segments(2,2*i-1)-delta;
                y2 = segments(2,2*i)+delta;
                x1 = (coeffs3(i,3)-coeffs3(i,2)*y1)/coeffs3(i,1);
                x2 = (coeffs3(i,3)-coeffs3(i,2)*y2)/coeffs3(i,1);
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
        if (~isempty(xi) && ~isempty(yi)) && (length(xi)==1 && length(yi)==1) 
            if segment_comparated2(i,5)<0.3        
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
            elseif segment_comparated2(i,5)>1.5
                if end_point1==1
                    new_segments(:,2*seg1-1)=[xi;yi];
                else
                    new_segments(:,2*seg1)=[xi;yi];
                end
            end
        end
    end
%     for i = 1:size(segment_comparated_endpoint,1)
%         seg1 = segment_comparated_endpoint(i,1);
%         seg2 = segment_comparated_endpoint(i,2);
%         end_point1 = segment_comparated_endpoint(i,3);
%         [xi,yi] = polyxpoly(big_new_segments(1,2*seg1-1:2*seg1),big_new_segments(2,2*seg1-1:2*seg1),big_new_segments(1,2*seg2-1:2*seg2),big_new_segments(2,2*seg2-1:2*seg2));
%         if end_point1==1
%             new_segments(:,2*seg1-1)=[xi;yi];
%         else
%             new_segments(:,2*seg1)=[xi;yi];
%         end
%     end
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