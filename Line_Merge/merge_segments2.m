% Função que tenta fazer "merge" de 2 segmentos de reta
% Entradas: coordenadas (x,y) dos 2 segmentos Seg1 e Seg2
%           Seg1 = [x1 x2; y1 y2] onde (x1,y1) e (x2,y2) = "end points" 1 e 2
%           Seg2 = [x3 x4; y3 y4] onde (x3,y3) e (x4,y4) = "end points" 3 e 4
% Saídas: Seg3 =[x5 x6; y5 y6] com as coordenadas do segmento resultante
%         Seg3 = [] se não for possível fazer o "merge" de Seg1 e Seg2
close all;clc;clear all;
dist_min = 0.2; %metros
ang_min = 15; %graus

% Seg1 = [1 0;0 1];
% Seg2 = [0.5 1;1.5 1];

%Segmentos diagonais
% Seg1 = [0.9 0;0 1.1];
% Seg2 = [1 0;0 1];

%Caso1
% Seg1 = [0 1;0 1];     %OK!!!
% Seg2 = [-0.1 1.1;-0.1 1.1];
%Caso2
% Seg1 = [0 1;0 1];     %OK!!!
% Seg2 = [-0.1 0.8;-0.1 0.8];
%Caso3
% Seg1 = [0 1;0 1];     %OK!!!
% Seg2 = [2 3;2 3];

%Segmentos Verticais
% Seg1 = [1.1 1.1;0.9 0.1];
% Seg2 = [1 1;1 0];

%Caso1
% Seg1 = [1 1;0 1];         %OK!!!
% Seg2 = [1.1 1.1;-0.1 1.1];
%Caso2
% Seg1 = [1 1;0 1];         %OK!!!
% Seg2 = [1.1 1.1;0.8 1.8];
%Caso3
% Seg1 = [1 1;0 1];         %OK!!!
% Seg2 = [1.1 1.1;1.8 2.8];

%Segmentos Horizontais
% Seg1 = [0.9 0.1;1.1 1.1];
% Seg2 = [1 0;1 1];

%Caso1
% Seg1 = [0.1 1.1;1 1];     %OK!!!
% Seg2 = [-0.9 1.3;1.1 1.1];
%Caso2
% Seg1 = [0.1 1.1;1 1];     %OK!!!
% Seg2 = [-0.9 0.8;1.1 1.1];
%Caso3
% Seg1 = [0.1 1.1;1 1];     %OK!!!
% Seg2 = [-0.1 -0.5;1.1 1.1];

%TESTES
%segmentos diagonais
% Seg1 = [-5 7;4 4.5];      %OK!!!
% Seg2 = [-5.15 7.15;4.1 4.15];

% segmentos quase verticais
% Seg1 = [5.5 5.505; 10.3 11];  %OK!!!
% Seg2 = [5.51 5.495;10 11.2];

% segmentos quase horizontais
% Seg1 = [-15.1 -13.8;-14.95 -15.1];  %verificar OK!!!
% Seg2 = [-15 -14;-15 -14.9];

% Seg1 =[0.4508 -0.6897;-0.7924 -0.7600]; %NAO FUNC!!! -> FUNCIONOU
% Seg2 = [-0.6924 5.0737;-2.2784 -2.2784];

Seg1 =[0.4508 -0.6897;-0.7924 -0.7600]; %NAO FUNC!!!  -> FUNCIONOU
Seg2 = [-0.6924 5.0737;-0.759 -0.759];

figure
plot(Seg1(1,:),Seg1(2,:),'LineWidth',2);hold on;
plot(Seg2(1,:),Seg2(2,:),'LineWidth',2);hold off;
axis equal;


p1 = Seg1(:,2)-Seg1(:,1);
p2 = Seg2(:,2)-Seg2(:,1);
teta(1)=atan2(p1(2),p1(1))*180/pi;
teta(2)=atan2(p2(2),p2(1))*180/pi;
teta1(1) = fix_teta(teta(1)*pi/180);
teta1(2) = fix_teta(teta(2)*pi/180);
teta1=teta1*180/pi;

dif_ang=calc_dif_ang(teta1(1,1),teta1(1,2));



% if abs(teta(2)-teta(1))<=ang_min
%     teta1(1) = teta(1);
%     teta1(2) = teta(2);
% else
%     teta1(1) = fix_teta(teta(1)*pi/180);
%     teta1(2) = fix_teta(teta(2)*pi/180);
%     teta1=teta1*180/pi;
% end

% abs(teta1(2)-teta1(1))
if dif_ang<=ang_min
    %PODE OCORRER MERGE
    
    V=[Seg1 Seg2];
    %Formula da distancia do ponto em relacao ao segmento
    %Segmentos originais
    dist=distance_point_to_seg(V);

    if min(dist)<=dist_min
        %Segmentos proximos da vertical, torná-los verticais
        x = [Seg1(1,:) Seg2(1,:)];
        if abs(x(1)-x(2)) < dist_min
            x_med=(x(1)+x(2))/2; Seg1(1,:)=[x_med x_med];
            if Seg1(2,2) < Seg1(2,1); Seg1(2,:) = Seg1(2,[2 1]); end
        end
        if abs(x(3)-x(4)) < dist_min
           x_med=(x(3)+x(4))/2; Seg2(1,:)=[x_med x_med];
           if Seg2(2,2) < Seg2(2,1); Seg2(2,:) = Seg2(2,[2 1]); end
           Seg2(1,:) = Seg1(1,:);
        end
    
        %Segmentos proximos da horizontal, torná-los horizontais
        y=[Seg1(2,:) Seg2(2,:)];
        if abs(y(1)-y(2)) < dist_min
            y_med=(y(1)+y(2))/2; Seg1(2,:)=[y_med y_med]; 
            if Seg1(1,2) < Seg1(1,1); Seg1(1,:) = Seg1(1,[2 1]); end
        end
        if abs(y(3)-y(4)) < dist_min
            y_med=(y(3)+y(4))/2; Seg2(2,:)=[y_med y_med]; 
            if Seg2(1,2) < Seg2(1,1); Seg2(1,:) = Seg2(1,[2 1]); end
            Seg2(2,:) = Seg1(2,:);
        end
    
        if abs(Seg1(1,1)-Seg1(1,2))>dist_min && abs(Seg2(1,1)-Seg2(1,2))>dist_min
            % Garantir que x1 <= x2, x3 <= x4
            x1=[Seg1(1,:) Seg2(1,:)];
            if x1(2) < x1(1); Seg1=Seg1(:,[2 1]); x1=[Seg1(1,:) Seg2(1,:)]; end % se x2 < x1
            if x1(4) < x1(3); Seg2=Seg2(:,[2 1]); x1=[Seg1(1,:) Seg2(1,:)]; end % se x4 < x3
        else
            % Garantir que y1 <= y2, y3 <= y4
            y1=[Seg1(2,:) Seg2(2,:)];
            if y1(2) < y1(1); Seg1=Seg1(:,[2 1]); y1=[Seg1(2,:) Seg2(2,:)]; end % se x2 < x1
            if y1(4) < y1(3); Seg2=Seg2(:,[2 1]); y1=[Seg1(2,:) Seg2(2,:)]; end % se x4 < x3
        end
        %-------------------------------------------------------------
        P=[Seg1 Seg2];
        %Formula da distancia do ponto em relacao ao segmento
        %Segmentos modificados
        d_point_to_seg=distance_point_to_seg(P);
    
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
    
        %-------------------------------------------------------------
        if min(d_point_to_seg)<=dist_min
        % Caso 1 - Total Overlap
            if abs(P(1,1)-P(1,2))>dist_min
                if P(1,1)<=P(1,3) && P(1,4)<=P(1,2)
                    Seg3=[P(:,1) P(:,2)];
                    fprintf('Caso 1 - Total Overlap\n');
                elseif P(1,3)<=P(1,1) && P(1,2)<=P(1,4)
                    Seg3=[P(:,3) P(:,4)];
                    fprintf('Caso 1 - Total Overlap\n');
                end
            else
                if P(2,1)<=P(2,3) && P(2,4)<=P(2,2)
                    Seg3=[P(:,1) P(:,2)];
                    fprintf('Caso 1 - Total Overlap\n');
                elseif P(2,3)<=P(2,1) && P(2,2)<=P(2,4)
                    Seg3=[P(:,3) P(:,4)];
                    fprintf('Caso 1 - Total Overlap\n');
                end
            end
        % Caso 2 - Partial Overlap
            if abs(P(1,1)-P(1,2))>dist_min
                if P(1,1)<=P(1,3) && P(1,2)<=P(1,4) && P(1,3)<=P(1,2)
                    Seg3=[P(:,1) P(:,4)];
                    fprintf('Caso 2 - Partial Overlap\n');
                elseif P(1,3)<=P(1,1) && P(1,4)<=P(1,2) && P(1,1)<=P(1,4)
                    Seg3=[P(:,3) P(:,2)];
                    fprintf('Caso 2 - Partial Overlap\n');
                end
            else
                if P(2,1)<=P(2,3) && P(2,2)<=P(2,4) && P(2,3)<=P(2,2)
                    Seg3=[P(:,1) P(:,4)];
                    fprintf('Caso 2 - Partial Overlap\n');
                elseif P(2,3)<=P(2,1) && P(2,4)<=P(2,2) && P(2,1)<=P(2,4)
                    Seg3=[P(:,3) P(:,2)];
                    fprintf('Caso 2 - Partial Overlap\n');
                end
            end
        % Caso 3 - No Overlap
            if abs(P(1,1)-P(1,2))>dist_min
                if d23>=dist_min && d14>=dist_min && d13>=dist_min && d24>=dist_min
                    if P(1,2)<=P(1,3) && P(1,1)<=P(1,4)
                        Seg3=[];
                        fprintf('Caso 3 - No Overlap\n');
                    elseif P(1,4)<=P(1,1) && P(1,3)<=P(1,2)
                        Seg3=[];
                        fprintf('Caso 3 - No Overlap\n');
                    end
                end
            else
                if d23>=dist_min && d14>=dist_min && d13>=dist_min && d24>=dist_min
                    if P(2,2)<=P(2,3) && P(2,1)<=P(2,4)
                        Seg3=[];
                        fprintf('Caso 3 - No Overlap\n');
                    elseif P(2,4)<=P(2,1) && P(2,3)<=P(2,2)
                        Seg3=[];
                        fprintf('Caso 3 - No Overlap\n');
                    end
                end
            end
            hold on;
            plot(Seg3(1,:),Seg3(2,:),'LineWidth',2);
            hold off;
        
        else
            Seg3=[];
        end
    else
        Seg3=[];
    end
else
    %MERGE NAO OCORRE
    Seg3 = [];
end

function d_point_to_seg=distance_point_to_seg(P)
    %     %Formula da distancia do ponto em relacao ao segmento
    %     %teste dos endpoints do Seg1 em relação ao Seg2
    %     %endpoint1 do Seg1
        d1 = abs((P(1,4)-P(1,3))*(P(2,3)-P(2,1))-(P(1,3)-P(1,1))*(P(2,4)-P(2,3)))/sqrt((P(1,4)-P(1,3))^2+(P(2,4)-P(2,3))^2);
    %     %endpoint2 do Seg1
        d2 = abs((P(1,4)-P(1,3))*(P(2,3)-P(2,2))-(P(1,3)-P(1,2))*(P(2,4)-P(2,3)))/sqrt((P(1,4)-P(1,3))^2+(P(2,4)-P(2,3))^2);
    %     %teste dos endpoints do Seg2 em relação ao Seg1
    %     %endpoint1 do Seg2
        d3 = abs((P(1,2)-P(1,1))*(P(2,1)-P(2,3))-(P(1,1)-P(1,3))*(P(2,2)-P(2,1)))/sqrt((P(1,2)-P(1,1))^2+(P(2,2)-P(2,1))^2);
    %     %endpoint2 do Seg2
        d4 = abs((P(1,2)-P(1,1))*(P(2,1)-P(2,4))-(P(1,1)-P(1,4))*(P(2,2)-P(2,1)))/sqrt((P(1,2)-P(1,1))^2+(P(2,2)-P(2,1))^2);
        d_point_to_seg = [d1 d2 d3 d4];
end

function dif_ang=calc_dif_ang(teta1,teta2)
d_teta=teta1-teta2;
d_ang_1=mod(d_teta,180);
d_ang_2=mod(-d_teta,180);
dif_ang=min(abs([d_ang_1 d_ang_2]));
end
% function teta=fix_teta(teta)
% % % Retorna teta no intervalo [-pi,pi]
% if abs(teta)>=pi
%     ind=find(abs(teta)>=pi);
% %     for i=ind
% %         while teta(i) < -pi; teta(i) = teta(i) + 2*pi; end
% %         while teta(i) >  pi; teta(i) = teta(i) - 2*pi; end
% %     end
%     for i=ind
%         while teta(i) <= -pi; teta(i) = teta(i) + pi; end
%         while teta(i) >=  pi; teta(i) = teta(i) - pi; end
%     end
% elseif abs(teta)>=pi/2 && abs(teta)<pi
%     % Retorna teta no intervalo [-pi/2,pi/2]
%     ind=find(abs(teta)>=pi/2);
%     for i=ind
%         while teta(i) <= -pi/2; teta(i) = teta(i) + pi; end
%         while teta(i) >  pi/2; teta(i) = teta(i) - pi; end
%     end
% end
% end
% 
% function teta=fix_teta(teta)
% % Retorna teta no intervalo [0,pi]
%     ind=find(abs(teta)>=pi || abs(teta)>=0);
%     for i=ind
%         while teta(i) < 0; teta(i) = teta(i) + pi; end
%         while teta(i) >= pi; teta(i) = teta(i) - pi; end
%     end
% %     if teta<0
% %         teta = teta+pi;
% %     elseif teta>=pi
% %         teta = teta-pi;
% %     end
% end


% function teta=fix_teta(teta)
% % Retorna teta no intervalo [0,pi/2]
%     ind=find(abs(teta)>=pi/2);
%     for i=ind
%         while teta(i) < 0; teta(i) = teta(i) + pi; end
%         while teta(i) >= pi/2; teta(i) = teta(i) - pi; end
%     end
% %     if teta<0
% %         teta = teta+pi;
% %     elseif teta>=pi
% %         teta = teta-pi;
% %     end
% end

function teta=fix_teta(teta)
% Retorna teta no intervalo [0,pi]
    while teta < 0; teta = teta + pi; end
    while teta > pi; teta = teta - pi; end    
end