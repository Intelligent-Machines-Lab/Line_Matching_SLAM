clear all; close all;
load MG1;
MG1=Mapa_Global;
fprintf('\n MG1: No. de segmentos/cantos = %d',size(Mapa_Global.Segmentos,2));
[h1,h2]=plot_Ambiente(Mapa_Global.Segmentos,1); hold on;
Mapa_Global.Cantos=Detect_Corners(Mapa_Global.Segmentos);
fprintf('\n MG1: No. de cantos = %d',size(Mapa_Global.Cantos,2));
plot(Mapa_Global.Cantos(1,:),Mapa_Global.Cantos(2,:),'ro','MarkerSize',5,'MarkerFaceColor','r');
%
load MG2;
MG2=Mapa_Global;
fprintf('\n MG2: No. de segmentos/cantos = %d',size(Mapa_Global.Segmentos,2));
[h1,h2]=plot_Ambiente(Mapa_Global.Segmentos,2); hold on;
Mapa_Global.Cantos=Detect_Corners(Mapa_Global.Segmentos);
fprintf('\n MG2: No. de cantos = %d',size(Mapa_Global.Cantos,2));
plot(Mapa_Global.Cantos(1,:),Mapa_Global.Cantos(2,:),'ro','MarkerSize',5,'MarkerFaceColor','r');

function corners=Detect_Corners(segments)
% dim(segments) = 2x2*N, onde N = no. de "line segments"
% dim(corners) = 2xNc, onde Nc = número de cantos detectados
N = size(segments,2)/2;
corner1 = zeros(0,2);
% Procura por cantos ("end points" que estão muito próximos de outros "end points")
aux=1;
for i = 1:N
    for j=1:N
        if i~=j
            [xi,yi] = polyxpoly(segments(1,2*i-1:2*i),segments(2,2*i-1:2*i),segments(1,2*j-1:2*j),segments(2,2*j-1:2*j));
            if (~isempty(xi) && ~isempty(yi)) && (length(xi)<2 && length(yi)<2)
                corner1(aux,:)=[xi,yi];
                aux=aux+1;
            end
        end
    end
end
[corners,~,~] = unique(corner1,'rows','stable');
corners=corners';
% Procura por "end points" isolados, ou seja, sem vizinhos
for j=1:2*N
    ind1=setdiff(1:2*N,j);
    dist1=segments(:,j)-segments(:,ind1);
    dist=sqrt(dist1(1,:).^2+dist1(2,:).^2);
    dist_min=min(dist);
    if dist_min > 0.2
       corners=[corners segments(:,j)];
    end
end
% Verificar se algum "end point" identificado como isolado já tinha sido identificado como canto.
% Sejam 2 segmentos de reta S1 e S2 definidos respectivamente pelos seus "end points" (A,B) e (C,D).
% Se o "end points" A de S1 está próximo do meio de S2 mas longe de C e D, o "end point" A será
% identificado como canto e depois também como "end point" isolado. Então retirar a sua repetição
% de "corners".
tol=1e-5;
[corners2,~,~] = uniquetol(corners',tol,'ByRows',true);
corners=corners2';
end

function [h1,h2]=plot_Ambiente(Ambiente,Nf)
figure(Nf);
Sep=[NaN; NaN];
N1=size(Ambiente,2)/2; % N1 = número de segmentos em Ambiente
if N1==0; error('Ambiente vazio em plot_Ambiente!'); end
for i=1:N1; Ambiente1(:,[3*i-2 3*i-1 3*i])= [Ambiente(:,[2*i-1 2*i]) Sep]; end
h1=plot(Ambiente1(1,:),Ambiente1(2,:),'k-','LineWidth',2);
for i=1:N1; h2(i)   =text(Ambiente1(1,3*i-1)-0.2,Ambiente1(2,3*i-1),num2str(i),'Color','b'); end
for i=1:N1; h2(N1+i)=text(Ambiente1(1,3*i-2)+0.2,Ambiente1(2,3*i-2),num2str(i),'Color','b'); end
%axis([Limx(1) Limx(2) Limy(1) Limy(2)]);
grid on; axis equal;
%title('Ambiente');
xlabel('X [m]'); ylabel('y [m]');
end