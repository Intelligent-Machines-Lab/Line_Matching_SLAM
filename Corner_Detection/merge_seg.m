% Função que tenta fazer "merge" de 2 segmentos de reta
% Entradas: coordenadas (x,y) dos 2 segmentos Seg1 e Seg2
%           Seg1 = [x1 x2; y1 y2] onde (x1,y1) e (x2,y2) = "end points" 1 e 2
%           Seg2 = [x3 x4; y3 y4] onde (x3,y3) e (x4,y4) = "end points" 3 e 4
% Saídas: Seg3 =[x5 x6; y5 y6] com as coordenadas do segmento resultante
%         Seg3 = [] se não for possível fazer o "merge" de Seg1 e Seg2
% CLNJ, 10/11/2022

function Seg3=merge_seg(Seg1,Seg2)
ang_min=2;  % ângulo mínimo em graus
dist_min=0.05; % distância mínima em metros

close all;
[h1,h2]=plot_Ambiente([Seg1 Seg2],1);

% 1. Escolher ponto P0 como nova origem à esquerda e abaixo dos pontos em Seg1 e Seg2
% Dessa maneira, nesse novo SC, os pontos de Seg1 e Seg2 estarão no 1o quadrante
P0=[-0.5; -0.5]; % melhoria: escolher Pd conforme Seg1 e Seg2
P0(1,1)=min([Seg1(1,:) Seg2(1,:)])+P0(1);
P0(2,1)=min([Seg1(2,:) Seg2(2,:)])+P0(2);
Seg1=Seg1-repmat(P0,1,2); % coordenadas dos segmentos no novo SC
Seg2=Seg2-repmat(P0,1,2); % coordenadas dos segmentos no novo SC

% 2 - Garantir que x1 <= x2, x3 <= x4, x1 <= x3
x=[Seg1(1,:) Seg2(1,:)];
if x(2) < x(1); Seg1=Seg1(:,[2 1]); x=[Seg1(1,:) Seg2(1,:)]; end % se x2 < x1
if x(4) < x(3); Seg2=Seg2(:,[2 1]); x=[Seg1(1,:) Seg2(1,:)]; end % se x4 < x3
if x(3) < x(1); Seg=Seg1; Seg1=Seg2; Seg2=Seg; end % se x3 < x1

% 3. Verificar se Seg1 e Seg2 pertencem a retas com inclinações próximas
%    e com distâncias próximas da origem.
%    m1, m2 = ângulos das retas que contém os segmentos, sendo que abs(m) <= 90 graus
%    dist1, dist2  = distâncias dessas retas à origem
p=Seg1(:,2)-Seg1(:,1); teta(1)=atan2(p(2),p(1))*180/pi;
dist(1)=abs(p(1)*Seg1(2,1)-p(2)*Seg1(1,1))/sqrt(p'*p);
p=Seg2(:,2)-Seg2(:,1); teta(2)=atan2(p(2),p(1))*180/pi;
dist(2)=abs(p(1)*Seg2(2,1)-p(2)*Seg2(1,1))/sqrt(p'*p);
for i=1:2
    if teta(i) <= -90; teta(i)=teta(i)+180; end
    if teta(i) > 90;  teta(i)=teta(i)-180; end
end

%[dist teta]
% 3. Tentar fazer o MERGE dos Segmentos
Seg3=[];
if (abs(teta(2)-teta(1)) < ang_min) && (abs(dist(2)-dist(1)) < dist_min)
   % Tentar MERGE dos segmentos
   % Se os segmentos forem verticais, ordenar os pontos pela coordenada Y
   disp("MERGE talvez possível.");
   if (abs(sum(teta)/2)) >= 90
      disp("Caso segmentos verticais.");
      % Garantir que y1 <= y2, y3 <= y4, y1 <= y3
       y=[Seg1(2,:) Seg2(2,:)];
       if y(2) < y(1); Seg1=Seg1(:,[2 1]); end % se y2 < y1
       if y(4) < y(3); Seg2=Seg2(:,[2 1]); end % se y4 < y3
       if y(3) < y(1); Seg=Seg1; Seg1=Seg2; Seg2=Seg; end % se y3 < y1
   end   
   P=[Seg1 Seg2];
   d12=P(:,2)-P(:,1); d12=sqrt(d12'*d12);
   d13=P(:,3)-P(:,1); d13=sqrt(d13'*d13);
   d23=P(:,3)-P(:,2); d23=sqrt(d23'*d23);
   d14=P(:,4)-P(:,1); d14=sqrt(d14'*d14);
   d=[d12 d13 d23 d14]; %[d,teta],
   if (d23 <= dist_min); Seg3=[P(:,1) P(:,4)]; end % P1 P2 P3 P4
   if (d13 <= d12) && (d12 <= d14); Seg3=[P(:,1) P(:,4)]; end % P1 P3 P2 P4
   if (d13 <= d12) && (d12 <= d14); Seg3=[P(:,1) P(:,4)]; end % P1 P3 P2 P4
   if (d13 <= d12) && (d14 <= d12); Seg3=[P(:,1) P(:,2)]; end % P1 P3 P4 P2
   % Retornar as coordendas dos pontos de Seg3 para o SC original
   if ~isempty(Seg3)
       Seg3=Seg3+repmat(P0,1,2);
       disp("MERGE feito!");
       plot(Seg3(1,:),Seg3(2,:),'g+');
   else
       disp("MERGE não feito!");
   end
else
   disp("MERGE NOT POSSIBLE!")
end
end

function [h1,h2]=plot_Ambiente(Ambiente,Nf)
figure(Nf);
Sep=[NaN; NaN];
N1=size(Ambiente,2)/2; % N1 = número de segmentos em Ambiente
if N1==0; error('Ambiente vazio em plot_Ambiente!'); end
for i=1:N1; Ambiente1(:,[3*i-2 3*i-1 3*i])= [Ambiente(:,[2*i-1 2*i]) Sep]; end
h1=plot(Ambiente1(1,:),Ambiente1(2,:),'k-','LineWidth',2); hold on;
plot(Ambiente1(1,:),Ambiente1(2,:),'*');
for i=1:N1; h2(i)   =text(Ambiente1(1,3*i-1)-0.2,Ambiente1(2,3*i-1),num2str(i),'Color','b'); end
for i=1:N1; h2(N1+i)=text(Ambiente1(1,3*i-2)+0.2,Ambiente1(2,3*i-2),num2str(i),'Color','b'); end
grid on; axis equal;
% LimX=get(gca,'XLim'); LimY=get(gca,'YLim');
% if LimX(1) > 0; LimX(1)=0; else LimX(2)=0;  end
% if LimY(1) > 0; LimY(1)=0; else LimY(2)=0;  end
% axis([LimX(1) LimX(2) LimY(1) LimY(2)]);
%title('Ambiente');
xlabel('X [m]'); ylabel('y [m]');
end