function [real_inlier,real_outlier]=discrete_segment3(new_segment,new_inliers,coeff)
    L_segment = sqrt((new_segment(1,1)-new_segment(2,1))^2+(new_segment(1,2)-new_segment(2,2))^2);
%     L = 1*size(new_inliers,1)/3;
    L = 2*size(new_inliers,1);
    y = linspace(new_segment(1,2),new_segment(2,2),L);
    x = linspace(new_segment(1,1),new_segment(2,1),L);
    points = [x;y]';
    dec = 3; %casa decimal apos a virgula
% round(coeff(1),dec)~=0
    if round(coeff(1),dec)~=0
        %Ordenar inliers
            y_sort = sort(new_inliers(:,2));
            for i = 1:length(y_sort)
                index = find(new_inliers(:,2)==y_sort(i));
                x_sort(i,:) = new_inliers(index,1);
            end
            inlier_sort = [x_sort,y_sort];
    elseif round(coeff(2),dec)~=0
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
        Idx = rangesearch(new_inliers,points,0.05);
%         Idx = rangesearch(new_inliers,points,0.15);
%         Idx = rangesearch(new_inliers,points,0.2);
%         Idx = [];
%         Idx = rangesearch(new_inliers,points,0.4);
    else
        Idx = rangesearch(new_inliers,points,0.25);
%         Idx = rangesearch(new_inliers,points,0.15);
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