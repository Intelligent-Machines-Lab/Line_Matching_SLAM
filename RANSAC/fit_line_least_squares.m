% Referencia do livro Graphics Gem V
% Chap III.1 - The Best Least-Squares Line Fit
function [A, B, C] = fit_line_least_squares(points)
    % Na página 94 do livro, onde ele pega os pontos para a origem subtraindo pela média:
    centroid = [mean(points(:, 1)), mean(points(:, 2))];
    points_aux = [points(:, 1) - centroid(1), points(:, 2) - centroid(2)];
    
    % Eq. 14: Obtém a'
    data_aux_2 = points_aux.^2;
    sum_data_aux = sum(data_aux_2); % sum columns
    a = sum_data_aux(1) - sum_data_aux(2);
    
    % Eq. 14: Obtem b'
    b = 0;
    for i=1:size(points_aux, 1)
        b = b + points_aux(i, 1) * points_aux(i, 2);
    end

    % Eq. 23 : Obtem os coeficientes da equacao da reta
    if (a == 0 && b == 0)
        A = []; B = []; C = [];
    else
        A = 2 * b;
        B = -(a + sqrt(a^2 + 4*(b^2)));
        C = A*centroid(1) + B*centroid(2);
    end
end