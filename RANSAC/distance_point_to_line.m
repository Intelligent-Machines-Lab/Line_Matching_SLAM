% Funcao que calcula a distância entre e o ponto e uma linha reta
function distance = distance_point_to_line(pt,seg1,seg2)
    %Referencia: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    % A distancia entre um ponto e uma reta formada por 2 pontos se da pela
    % formula: com P1 = [x1, y1] e P2 = [x2, y2]
    % distancia(P1,P2,(x0,y0)) = |(x2-x1)(y1-y0) - (x1 -x0)(y2-y1)|/ sqrt((x2-x1)² + (y2-y1)²)
    
    distance = abs((seg2(1) - seg1(1))*(seg1(2)-pt(2)) - (seg1(1) - pt(1))*(seg2(2) - seg1(2)))/(sqrt((seg2(1) - seg1(1))^2 + (seg2(2) - seg1(2))^2));
    
%     pt1 = [];
%     if seg1(1)==seg2(1)
%         % y = Ax + B
%         % x = ?
%         distance = abs((seg2(1) - seg1(1))*(seg1(2)-pt(2)) - (seg1(1) - pt(1))*(seg2(2) - seg1(2)))/(sqrt((seg2(1) - seg1(1))^2 + (seg2(2) - seg1(2))^2));
%     else
%  
%     %Definir os coeficientes da reta que possui os 2 pontos seg1 e seg2 por
%     %meio do seguinte sistema de equacoes lineares:
%     % A*x1 + B = y1
%     % A*x2 + B = y2
%     syms A B
%     eqn1 = seg1(1)*A + B == seg1(2);
%     eqn2 = seg2(1)*A + B == seg2(2);
%     sol = solve([eqn1, eqn2], [A, B]);
% 
%     %Os coeficientes do sistema sao:
%     % y = A1*x + B1
%     A1 = double(sol.A);
%     B1 = double(sol.B);
%     
%     %Definir a equacao da reta perpendicular a reta obtida anteriormente
%     %que possui o ponto pt
%     %Tem q definir os coeficientes dessa reta
%     % y1 = -(1/A1)*x + C
%     %O coeficiente A2 é o inverso negativo do Coeficiente A1 da reta
%     %anterior por ser 90 graus em relacao a ela
%     A2 = -(1/A1);
%     C = pt(2)-(A2*pt(1));
%     
%     %Encontrar o ponto pertencente a reta que as retas se cruzam
% %     (A1-A2)*x = C - B1
% %     syms x
% %     eqn3 = A1*x + B1 - A2*x - C == 0;
% %     sol1 = solve(eqn3, x);
% %     aux = double(sol1)
%     x = (C-B1)/(A1-A2);
%     if isempty(x)
%         a;
%     end
% %     pt1(1) = double(x);
%     pt1(1) = x;
%     pt1(2) = A1*pt1(1) + B1;
%     
%     %Calcular a distancia entre o ponto pt e o ponto pertencente a reta 
%     distance = sqrt((pt(1) - pt1(1))^2+(pt(2)-pt1(2))^2);
%     end
end