% ransac implementation
% input: x - pontos no formato [x1, y1; x2, y2;...]
%        maxIter - maximo de interacoes no algoritmo
%        cost_th - threshold para a funcao de custo
% output: best_sample - melhor amostra de pontos no formato [x1, y1; x2, y2]
%         best_cost - melhor custo para todos os pontos dos dados
%         best_consensus - melhores inliers de acordo com o threshold de
%         inliers (inlier_th)
%         segment - [x1, y1; x2, y2] segmento da linha montado usando apenas inliers
%         coeff - coeficientes da melhor reta cuja equacao possui a estrutura Ax + Bx = C
%         inliers - pontos que estao de acordo com o threshold
%         outliers - pontos que NAO estao de acordo com o threshold

function [best_sample, best_cost, best_consensus, segment, coeff,inliers,outliers] = line_RANSAC2(points, maxIter, cost_th)
    % Pre alocacao dos vetores e variáveis 
    cost_arr = zeros(size(points, 1), 1);
    best_consensus = [];
    best_sample = [];
    best_cost = [];
    segment = [];
    coeff = [];
    
    % Parâmetros do Algoritmo (tune this!)
%     delta_huber = 1.0;

    % Considera um limite maximo de 80% dos pontos como inliers
    inlier_th = 1* size(points, 1);
    
    % Variáveis auxiliares
    interrupted = 0;
    
    % Loop para linhas retas
    for i=1:maxIter
        % Amostra de 2 pontos dos dados
        sample = datasample(points, 2, 'Replace', false);

        % Verificar todos os pontos no conjunto de dados e minimize a distância, maximizando os inliers
        for j=1:size(points, 1)
            pt = points(j, :);
            % Distância do ponto a linha reta
            d = distance_point_to_line(pt, sample(1, :), sample(2, :));
            % Cost function for distance
%             cost_arr(j) = huber_cost(d, delta_huber);
            cost_arr(j) = d;
        end
        % O consenso entá dentro do limite (threshold) de custo
        consensus = cost_arr <= cost_th;

        % Verificar se o consenso atual corresponde à condição da proposta
        % do threshold definido para numero de pontos inliers
        if (nnz(consensus) >= inlier_th && nnz(consensus)<20)
            best_sample = sample;
            best_consensus = consensus;
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
            interrupted = 1;
            disp('LINE - Finished via inlier threshold!');
            break;
        end
        % Verificar se o consenso atual é melhor do que o melhor consenso
        % já encontrado
        if((nnz(consensus) >= nnz(best_consensus)) && (nnz(consensus) >= 5))
            best_sample = sample;
            best_consensus = consensus;
            best_cost = cost_arr;
        end
        
    end
    
    if(~interrupted && ~isempty(best_consensus))  
        % Estrategia descobrindo os coeficientes da equacao da reta por meio de minimos quadrados  
        % Ax + By = C
        [a, b, c] = fit_line_least_squares(points(best_consensus, :));
        % Se 'a' for 0 a reta é horizontal e se 'b' for 0 a reta é vertical
        if a~=0
            % Obter MAX e MIN de Y
            [y1, i1] = max(points(best_consensus, 2));
            [y2, i2] = min(points(best_consensus, 2));
            x1 = (c - b * y1) / a;
            x2 = (c - b * y2) / a;
        elseif b~=0
            % Obter MAX e MIN de X
            [x1, i1] = max(points(best_consensus, 1));
            [x2, i2] = min(points(best_consensus, 1));
            y1 = (c - a * x1) / b;
            y2 = (c - a * x2) / b;
        end
        
        aux = 1;
        dist = [];
        points_inlier = points(best_consensus, :);
        for i = 1:size(points_inlier,1)
            for j = 1:size(points_inlier,1)
                dist(aux) = sqrt((points_inlier(i,1)-points_inlier(j,1))^2 + (points_inlier(i,2)-points_inlier(j,2))^2);
                aux = aux+1;
            end
        end
        %Comprimento maximo do segmento de reta
        dist_max = max(dist);
        %Comprimento do segmento de reta com os pontos x1,x2,y1,y2
        %encontrados
        dist_points_found = sqrt((x1-x2)^2 + (y1-y2)^2);

        tol = 1e-2;
        if dist_points_found>dist_max
            if abs(dist_points_found - dist_max)>tol
                x1 = max(points(best_consensus, 1));
                x2 = min(points(best_consensus, 1));
                y1 = (c - a * x1) / b;
                y2 = (c - a * x2) / b;
            end
        end

        segment = [x1, y1; x2, y2];
        
        %Coeficientes da equacao da reta
        coeff = [a, b, c];
        index_o = 1;
        index_i = 1;
        outliers = [];
        inliers = [];
        
        for i = 1:size(best_consensus,1)
            %Pontos considerados Inliers
            if best_consensus(i,1)==1
                inliers(index_i,:) = points(i,:);
                index_i = index_i+1;
            else
                %Pontos considerados Outliers
                outliers(index_o,:) = points(i,:);
                index_o = index_o+1;
            end
        end
        disp('LINE - Finished via max iteration!');
    end
    if (isempty(best_consensus))
        disp('LINE - Error 404');
    end
end

% function cost = huber_cost(e, delta_huber)
%     % Wikipedia
%     abs_e = abs(e);
%     if(abs_e <= delta_huber)
%         cost = (e^2) / 2;
%     else
%         cost = (delta_huber * abs_e) - (delta_huber^2) / 2;
%     end
% end

