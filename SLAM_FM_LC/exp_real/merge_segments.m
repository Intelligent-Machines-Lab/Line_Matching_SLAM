% Função que tenta fazer "merge" de 2 segmentos de reta
% Entradas: coordenadas (x,y) dos 2 segmentos Seg1 e Seg2
%           Seg1 = [x1 x2; y1 y2] onde (x1,y1) e (x2,y2) = "end points" 1 e 2
%           Seg2 = [x3 x4; y3 y4] onde (x3,y3) e (x4,y4) = "end points" 3 e 4
% Saídas: Seg3 =[x5 x6; y5 y6] com as coordenadas do segmento resultante
%         Seg3 = [] se não for possível fazer o "merge" de Seg1 e Seg2
function Seg3=merge_segments(Seg1,Seg2)
dist_min = 0.5; %metros
ang_min = 30; %graus

% Se segmentos forem proximos da vertical, redefinir x com a redefinicao
% dos endpoints
if abs(Seg1(1,1)-Seg1(1,2))<dist_min
    x_med = (Seg1(1,1)+Seg1(1,2))/2;
    Seg1(1,:) = [x_med x_med];
    if Seg1(2,1)>Seg1(2,2)
        aux1=Seg1(2,1);
        aux2=Seg1(2,2);
        Seg1(2,1)=aux2;
        Seg1(2,2)=aux1;
    end
end

if abs(Seg2(1,1)-Seg2(1,2))<dist_min
    x_med = (Seg2(1,1)+Seg2(1,2))/2;
    Seg2(1,:) = [x_med x_med];
    if Seg2(2,1)>Seg2(2,2)
        aux1=Seg2(2,1);
        aux2=Seg2(2,2);
        Seg2(2,1)=aux2;
        Seg2(2,2)=aux1;
    end
end

% Se segmentos forem proximos da horizontal, redefinir y com a redefinicao
% dos endpoints
if abs(Seg1(2,1)-Seg1(2,2))<dist_min
    y_med = (Seg1(2,1)+Seg1(2,2))/2;
    Seg1(2,:) = [y_med y_med];
    if Seg1(1,1)>Seg1(1,2)
        aux1=Seg1(1,1);
        aux2=Seg1(1,2);
        Seg1(1,1)=aux2;
        Seg1(1,2)=aux1;
    end
end
if abs(Seg2(2,1)-Seg2(2,2))<dist_min
    y_med = (Seg2(2,1)+Seg2(2,2))/2;
    Seg2(2,:) = [y_med y_med];
    if Seg2(1,1)>Seg2(1,2)
        aux1=Seg2(1,1);
        aux2=Seg2(1,2);
        Seg2(1,1)=aux2;
        Seg2(1,2)=aux1;
    end
end

% 1.3 - Garantir que x1 <= x2, x3 <= x4, x1 <= x3
x=[Seg1(1,:) Seg2(1,:)];
if x(2) < x(1); Seg1=Seg1(:,[2 1]); x=[Seg1(1,:) Seg2(1,:)]; end % se x2 < x1
if x(4) < x(3); Seg2=Seg2(:,[2 1]); x=[Seg1(1,:) Seg2(1,:)]; end % se x4 < x3
if x(3) < x(1); Seg=Seg1; Seg1=Seg2; Seg2=Seg; x=[Seg1(1,:) Seg2(1,:)]; end % se x3 < x1

% 2. Escolher ponto P0 como nova origem à esquerda e abaixo dos pontos em Seg1 e Seg2
% Dessa maneira, nesse novo SC, os pontos de Seg1 e Seg2 estarão no 1o quadrante
P0=[-0.5; -0.5]; % melhoria: escolher Pd conforme Seg1 e Seg2
P0(1,1)=min([Seg1(1,:) Seg2(1,:)])+P0(1);
P0(2,1)=min([Seg1(2,:) Seg2(2,:)])+P0(2);
Seg1=Seg1-repmat(P0,1,2); % coordenadas dos segmentos no novo SC
Seg2=Seg2-repmat(P0,1,2); % coordenadas dos segmentos no novo SC

% 3. Verificar se Seg1 e Seg2 pertencem a retas com inclinações próximas
%    e com distâncias próximas da origem.
%    m1, m2 = ângulos das retas que contém os segmentos, sendo que abs(m) <= 90 graus
%    dist1, dist2  = distâncias dessas retas à origem
p1=Seg1(:,2)-Seg1(:,1); teta(1)=atan2(p1(2),p1(1))*180/pi;
dist(1)=abs(p1(1)*Seg1(2,1)-p1(2)*Seg1(1,1))/sqrt(p1'*p1);
p2=Seg2(:,2)-Seg2(:,1); teta(2)=atan2(p2(2),p2(1))*180/pi;
dist(2)=abs(p2(1)*Seg2(2,1)-p2(2)*Seg2(1,1))/sqrt(p2'*p2);
teta=fix_teta(teta*pi/180);
teta=teta*180/pi;
%[Seg1, Seg2], [dist teta]
% 4. Tentar fazer o MERGE dos Segmentos
Seg3=[];
if (abs(teta(2)-teta(1)) < ang_min) && (abs(dist(2)-dist(1)) < dist_min)
    y=[];
    if (abs(sum(teta)/2)) >= 90
      %disp("Caso segmentos verticais.");
      % Garantir que y1 <= y2, y3 <= y4, y1 <= y3
       y=[Seg1(2,:) Seg2(2,:)];
       if y(2) < y(1); Seg1=Seg1(:,[2 1]); end % se y2 < y1
       if y(4) < y(3); Seg2=Seg2(:,[2 1]); end % se y4 < y3
       if y(3) < y(1); Seg=Seg1; Seg1=Seg2; Seg2=Seg; end % se y3 < y1
    end  
%     if isempty(y)
%         %Formula da distancia do ponto em relacao ao segmento
%     %teste dos endpoints do Seg1 em relação ao Seg2
%     %endpoint1 do Seg1
%     d1 = abs((Seg2(1,2)-Seg2(1,1))*(Seg2(2,1)-Seg1(2,1))-(Seg2(2,2)-Seg2(2,1))*(Seg2(1,1)-Seg1(1,1)))/sqrt((Seg2(1,2)-Seg2(1,1))^2+(Seg2(2,2)-Seg2(2,1))^2);
% %     d1 = abs((x(4)-x(3))*(y(3)-y(1))-(x(3)-x(1))*(y(4)-y(3)))/sqrt((x(4)-x(3))^2+(y(4)-y(3))^2);
%     %endpoint2 do Seg1
%     d2 = abs((Seg2(1,2)-Seg2(1,1))*(Seg2(2,1)-Seg1(2,2))-(Seg2(2,2)-Seg2(2,1))*(Seg2(1,1)-Seg1(1,2)))/sqrt((Seg2(1,2)-Seg2(1,1))^2+(Seg2(2,2)-Seg2(2,1))^2);
% %     d2 = abs((x(4)-x(3))*(y(3)-y(2))-(x(3)-x(2))*(y(4)-y(3)))/sqrt((x(4)-x(3))^2+(y(4)-y(3))^2);
%     %teste dos endpoints do Seg2 em relação ao Seg1
%     %endpoint1 do Seg2
%     d3 = abs((Seg1(1,2)-Seg1(1,1))*(Seg1(2,1)-Seg2(2,1))-(Seg1(2,2)-Seg1(2,1))*(Seg1(1,1)-Seg2(1,1)))/sqrt((Seg1(1,2)-Seg1(1,1))^2+(Seg1(2,2)-Seg1(2,1))^2);
% %     d3 = abs((x(2)-x(1))*(y(1)-y(3))-(x(1)-x(3))*(y(2)-y(1)))/sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
%     %endpoint2 do Seg2
%     d4 = abs((Seg1(1,2)-Seg1(1,1))*(Seg1(2,1)-Seg2(2,2))-(Seg1(2,2)-Seg1(2,1))*(Seg1(1,1)-Seg2(1,2)))/sqrt((Seg1(1,2)-Seg1(1,1))^2+(Seg1(2,2)-Seg1(2,1))^2);
% %     d4 = abs((x(2)-x(1))*(y(1)-y(4))-(x(1)-x(4))*(y(2)-y(1)))/sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
%     else
%         %Formula da distancia do ponto em relacao ao segmento
%     %teste dos endpoints do Seg1 em relação ao Seg2
%     %endpoint1 do Seg1
% %     d1 = abs((Seg2(1,2)-Seg2(1,1))*(Seg2(2,1)-Seg1(2,1))-(Seg2(2,2)-Seg2(2,1))*(Seg2(1,1)-Seg1(1,1)))/sqrt((Seg2(1,2)-Seg2(1,1))^2+(Seg2(2,2)-Seg2(2,1))^2);
%     d1 = abs((x(4)-x(3))*(y(3)-y(1))-(x(3)-x(1))*(y(4)-y(3)))/sqrt((x(4)-x(3))^2+(y(4)-y(3))^2);
%     %endpoint2 do Seg1
% %     d2 = abs((Seg2(1,2)-Seg2(1,1))*(Seg2(2,1)-Seg1(2,2))-(Seg2(2,2)-Seg2(2,1))*(Seg2(1,1)-Seg1(1,2)))/sqrt((Seg2(1,2)-Seg2(1,1))^2+(Seg2(2,2)-Seg2(2,1))^2);
%     d2 = abs((x(4)-x(3))*(y(3)-y(2))-(x(3)-x(2))*(y(4)-y(3)))/sqrt((x(4)-x(3))^2+(y(4)-y(3))^2);
%     %teste dos endpoints do Seg2 em relação ao Seg1
%     %endpoint1 do Seg2
% %     d3 = abs((Seg1(1,2)-Seg1(1,1))*(Seg1(2,1)-Seg2(2,1))-(Seg1(2,2)-Seg1(2,1))*(Seg1(1,1)-Seg2(1,1)))/sqrt((Seg1(1,2)-Seg1(1,1))^2+(Seg1(2,2)-Seg1(2,1))^2);
%     d3 = abs((x(2)-x(1))*(y(1)-y(3))-(x(1)-x(3))*(y(2)-y(1)))/sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
%     %endpoint2 do Seg2
% %     d4 = abs((Seg1(1,2)-Seg1(1,1))*(Seg1(2,1)-Seg2(2,2))-(Seg1(2,2)-Seg1(2,1))*(Seg1(1,1)-Seg2(1,2)))/sqrt((Seg1(1,2)-Seg1(1,1))^2+(Seg1(2,2)-Seg1(2,1))^2);
%     d4 = abs((x(2)-x(1))*(y(1)-y(4))-(x(1)-x(4))*(y(2)-y(1)))/sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
%     end

P=[Seg1 Seg2];
%     %Formula da distancia do ponto em relacao ao segmento
%     %teste dos endpoints do Seg1 em relação ao Seg2
%     %endpoint1 do Seg1
%     d1 = abs((P(1,4)-P(1,3))*(P(2,3)-P(2,1))-(P(1,3)-P(1,1))*(P(2,4)-P(2,3)))/sqrt((P(1,4)-P(1,3))^2+(P(2,4)-P(2,3))^2);
%     %endpoint2 do Seg1
%     d2 = abs((P(1,4)-P(1,3))*(P(2,3)-P(2,2))-(P(1,3)-P(1,2))*(P(2,4)-P(2,3)))/sqrt((P(1,4)-P(1,3))^2+(P(2,4)-P(2,3))^2);
%     %teste dos endpoints do Seg2 em relação ao Seg1
%     %endpoint1 do Seg2
%     d3 = abs((P(1,2)-P(1,1))*(P(2,1)-P(2,3))-(P(1,1)-P(1,3))*(P(2,2)-P(2,1)))/sqrt((P(1,2)-P(1,1))^2+(P(2,2)-P(2,1))^2);
%     %endpoint2 do Seg2
%     d4 = abs((P(1,2)-P(1,1))*(P(2,1)-P(2,4))-(P(1,1)-P(1,4))*(P(2,2)-P(2,1)))/sqrt((P(1,2)-P(1,1))^2+(P(2,2)-P(2,1))^2);

   %Formula da distancia entre end points
   d12 = sqrt((P(1,1)-P(1,2))^2+(P(2,1)-P(2,2))^2);
    %teste dos endpoints do Seg1 em relação ao Seg2
    %endpoint1 do Seg1 com endpoint1 do Seg2
    d13 = sqrt((P(1,1)-P(1,3))^2+(P(2,1)-P(2,3))^2);
    %endpoint2 do Seg1 com endpoint1 do Seg2
    d23 = sqrt((P(1,2)-P(1,3))^2+(P(2,2)-P(2,3))^2);
    %teste dos endpoints do Seg2 em relação ao Seg1
    %endpoint1 do Seg1 com endpoint2 do Seg2
    d14 = sqrt((P(1,1)-P(1,4))^2+(P(2,1)-P(2,4))^2);
    %endpoint2 do Seg1 com endpoint2 do Seg2
    d24 = sqrt((P(1,2)-P(1,4))^2+(P(2,2)-P(2,4))^2);

if d23<=dist_min
    %Se os segmentos forem horizontais mantem o y do seg1
    if abs(Seg1(2,1)-Seg1(2,2))<dist_min
        if Seg1(1,2)<=Seg2(1,1) && Seg1(1,1)<=Seg2(1,2)
           Seg3 = [P(:,1) P(:,4)]; 
        elseif Seg1(1,2)>Seg2(1,1) && Seg1(1,1)<=Seg2(1,2)
            Seg3 = [P(:,1) P(:,4)]; 
        elseif Seg1(1,2)>Seg2(1,1) && Seg1(1,2)>Seg2(1,2)
            Seg3 = [P(:,1) P(:,2)]; 
        end

    end  
    %Se os segmentos forem verticais mantem o x do seg1
    if abs(Seg1(1,1)-Seg1(1,2))<dist_min
        if Seg1(2,2)<=Seg2(2,1) && Seg1(2,1)<=Seg2(2,2)
           Seg3 = [P(:,1) P(:,4)]; 
        elseif Seg1(2,2)>Seg2(2,1) && Seg1(2,1)<=Seg2(2,2)
            Seg3 = [P(:,1) P(:,4)]; 
        elseif Seg1(2,2)>Seg2(2,1) && Seg1(2,2)>Seg2(2,2)
            Seg3 = [P(:,1) P(:,2)]; 
        end
    end
elseif d23>dist_min
     %Se os segmentos forem horizontais mantem o y do seg1
    if abs(Seg1(2,1)-Seg1(2,2))<dist_min
        if Seg1(1,2)<=Seg2(1,1) && Seg1(1,1)<=Seg2(1,2)
           Seg3 = [P(:,1) P(:,2)]; 
        elseif Seg1(1,2)>Seg2(1,1) && Seg1(1,2)<=Seg2(1,2) && Seg1(1,1)<=Seg2(1,1) 
            Seg3 = [P(:,1) P(:,4)]; 
        elseif Seg1(1,2)>Seg2(1,1) && Seg1(1,2)>Seg2(1,2) && Seg1(1,1)<=Seg2(1,1)
            Seg3 = [P(:,1) P(:,2)]; 
        end
    end
    %Se os segmentos forem verticais mantem o x do seg1
    if abs(Seg1(1,1)-Seg1(1,2))<dist_min
        if Seg1(2,2)<=Seg2(2,1) && Seg1(2,1)<=Seg2(2,2)
           Seg3 = [P(:,1) P(:,2)]; 
        elseif Seg1(2,2)>Seg2(2,1) && Seg1(2,2)<=Seg2(2,2) && Seg1(2,1)<=Seg2(2,1)
            Seg3 = [P(:,1) P(:,4)]; 
        elseif Seg1(2,2)>Seg2(2,1) && Seg1(2,2)>Seg2(2,2) && Seg1(2,1)<=Seg2(2,1)
            Seg3 = [P(:,1) P(:,2)]; 
        end
    end
%     if (d13<=d12) && (d12<=d14)
%         Seg3 = [P(:,1) P(:,4)]; 
%     end
%     if (d13<=d12) && (d14<=d12)
%         Seg3 = [P(:,1) P(:,2)]; 
%     end
% %     Seg3 = [P(:,1) P(:,2)]; 

end
if ~isempty(Seg3)
           Seg3=Seg3+repmat(P0,1,2);
end
end
%     if d1<dist_min && d2<dist_min && d3<dist_min && d4<dist_min
%         %Se os segmentos forem horizontais mantem o y do seg1
%         if abs(Seg1(2,1)-Seg1(2,2))<dist_min
%             if abs(Seg1(1,2)-Seg2(1,1))<dist_min
% %                 x1 = [Seg1(1,:) Seg2(1,:)];
%                 Seg3 = [P(:,1) P(:,4)];
%             elseif (abs(Seg1(1,1)-Seg2(1,1))<=abs(Seg1(1,2)-Seg1(1,1))) && (abs(Seg1(1,1)-Seg1(1,2))<=abs(Seg1(1,1)-Seg2(1,2)))
%                 Seg3 = [P(:,1) P(:,4)];
%             else
% %                 x1 = [Seg1(1,:)];
%                 Seg3 = [P(:,1) P(:,2)];
%             end
% %             x1 = [Seg1(1,:) Seg2(1,:)];
% %             Seg3(1,:) = [min(x1) max(x1)];
% %             Seg3(2,:) = [Seg1(2,1) Seg1(2,2)];
%         end
%         %Se os segmentos forem verticais mantem o x do seg1
%         if abs(Seg1(1,1)-Seg1(1,2))<dist_min
%             if abs(Seg1(2,2)-Seg2(2,1))<dist_min
% %                 y1 = [Seg1(2,:) Seg2(2,:)];
%                 Seg3 = [P(:,1) P(:,4)];
%             elseif (abs(Seg1(2,1)-Seg2(2,1))<=abs(Seg1(2,2)-Seg1(2,1))) && (abs(Seg1(2,1)-Seg1(2,2))<=abs(Seg1(2,1)-Seg2(2,2)))
%                 Seg3 = [P(:,1) P(:,4)];
%             else
% %                 y1 = [Seg1(2,:)];
%                 Seg3 = [P(:,1) P(:,2)];
%             end
% %             Seg3(2,:) = [min(y1) max(y1)];
% %             Seg3(1,:) = [Seg1(1,1) Seg1(1,2)];
%         end
%     elseif ((d1<dist_min && d2>dist_min) || (d1>dist_min && d2<dist_min)) && ((d3<dist_min && d4>dist_min) || (d3>dist_min && d4<dist_min))
% %          %Se os segmentos forem horizontais mantem o y do seg1
% %         if abs(Seg1(2,1)-Seg1(2,2))<dist_min
% %             x1 = [Seg1(1,:) Seg2(1,:)];
% %             Seg3(1,:) = [min(x1) max(x1)];
% %             Seg3(2,:) = [Seg1(2,1) Seg1(2,2)];
% %         end
% %         %Se os segmentos forem verticais mantem o x do seg1
% %         if abs(Seg1(1,1)-Seg1(1,2))<dist_min
% %             y1 = [Seg1(2,:) Seg2(2,:)];
% %             Seg3(2,:) = [min(y1) max(y1)];
% %             Seg3(1,:) = [Seg1(1,1) Seg1(1,2)];
% %         end
% %     elseif (d1>dist_min && d2>dist_min ) && (d3>dist_min && d4>dist_min )
% %         Seg3=[];
%         if abs(Seg1(2,1)-Seg1(2,2))<dist_min
%             if abs(Seg1(1,2)-Seg2(1,1))<dist_min
% %                 x1 = [Seg1(1,:) Seg2(1,:)];
%                 Seg3 = [P(:,1) P(:,4)];
%             elseif (abs(Seg1(1,1)-Seg2(1,1))<=abs(Seg1(1,2)-Seg1(1,1))) && (abs(Seg1(1,1)-Seg1(1,2))<=abs(Seg1(1,1)-Seg2(1,2)))
%                 Seg3 = [P(:,1) P(:,4)];
%             else
% %                 x1 = [Seg1(1,:)];
%                 Seg3 = [P(:,1) P(:,2)];
%             end
% %             x1 = [Seg1(1,:) Seg2(1,:)];
% %             Seg3(1,:) = [min(x1) max(x1)];
% %             Seg3(2,:) = [Seg1(2,1) Seg1(2,2)];
%         end
%         %Se os segmentos forem verticais mantem o x do seg1
%         if abs(Seg1(1,1)-Seg1(1,2))<dist_min
%             if abs(Seg1(2,2)-Seg2(2,1))<dist_min
% %                 y1 = [Seg1(2,:) Seg2(2,:)];
%                 Seg3 = [P(:,1) P(:,4)];
%             elseif (abs(Seg1(2,1)-Seg2(2,1))<=abs(Seg1(2,2)-Seg1(2,1))) && (abs(Seg1(2,1)-Seg1(2,2))<=abs(Seg1(2,1)-Seg2(2,2)))
%                 Seg3 = [P(:,1) P(:,4)];
%             else
% %                 y1 = [Seg1(2,:)];
%                 Seg3 = [P(:,1) P(:,2)];
%             end
% %             Seg3(2,:) = [min(y1) max(y1)];
% %             Seg3(1,:) = [Seg1(1,1) Seg1(1,2)];
%         end
%     end
%     if ~isempty(Seg3)
%        Seg3=Seg3+repmat(P0,1,2);
%     end
% end
end
function teta=fix_teta(teta)
% Retorna teta no intervalo [-pi,pi]
ind=find(abs(teta)>=pi);
for i=ind
    while teta(i) < -pi; teta(i) = teta(i) + 2*pi; end
    while teta(i) >  pi; teta(i) = teta(i) - 2*pi; end
end
end