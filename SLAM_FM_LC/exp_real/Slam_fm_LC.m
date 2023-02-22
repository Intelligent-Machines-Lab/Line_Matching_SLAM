% Programa de SLAM 2D usando Feature Matching (Line Matching) and Loop Closure
% Dados de entrada:
%   1. Segmentos de reta extraídos dos dados capturados em cada Scan
%   2. Poses reais de cada Scan
% Dados de Saída:
%   1. Mapa Global do Ambiente com os todos os segmentos de reta posicionados no mesmo Sistema de
%      Coordenadas
%   2. Poses estimadas de cada Scan no mesmo Sistema de Coordenadas do Mapa Global do Ambiente
% CLNJ e Luciana Lemos, 17/11/2022, 03:55 h
clc; clear all; close all;
fprintf('\n Feature Matching');

% Carregamento dos Scans e das Poses reais
fprintf('\n ==> Passo 1: Carregamento dos Scans');
Scan_vec=[2:82]; % vetor com os nos. dos arquivos de Scan a serem carregados
% Scan_vec=[2:12];
LC=1; % LC > 0 para uso de LOOP CLOSURE (LC)
Scan_LC=[3:81]; % Nos. dos scans (entre 1 e no. máximo de scans) selecionados para execução do procedimento de LC
% Scan_LC=[3:11];
Nf_MG=100;     % No. da janela para plotar o Mapa Global
Nf_LC=Nf_MG+1; % No. da janela inicial do procedimento de LC
Marcos_MG=[];
ic=0;
for iScan=Scan_vec
    str_iScan=num2str(iScan);
    eval(['load scan' str_iScan '.mat;']);
    %Ambiente(iScan).Scan=double(line_segment);
    ic=ic+1;
    Ambiente(ic).Scan=double(line_segment);
    Pose_P1_Real(:,ic)=[0; 0; double(pose_gt)]; % Armazenar leitura da bússola
    Ambiente(ic).LC=[];
end
NScans=ic;
clear line_segment pose_gt;

% Chamada da função de "Feature Matching" que retorna Rotação + Tranlação entre
% Scans consecutivos + correspondencia entre os segmentos (IC13 e IC31)
fprintf('\n ==> Passo 2: Cálculo da Rotação/Translação entre os Scans\n');
% Geração das Poses e do Mapa Global
fprintf('\n ==> Passo 3: Cálculo das Poses e do Mapa Global');
Mapa_Global.Cantos=[];
Marcos_MG=[];
Ambiente(1).Pose_P1=[0;0;0];

% Seg = [ -0.8208 1.6547; % Scan1.mat
%          0.6267 0.8166];
% Seg = [ -0.9372  0.1742; % Scan2.mat
%         -0.4365 -0.8707];
Seg = Ambiente(1).Scan(:,6:-1:5);
Seg=Seg(:,2)-Seg(:,1);
teta0=atan2(Seg(2),Seg(1));
Ambiente(1).Pose_P1=[0;0;-teta0];
   
for iScan=1:NScans
    Ambiente_SC3=Ambiente(iScan).Scan;
    N3=size(Ambiente_SC3,2)/2; % N3 = número de segmentos em SC3
    if iScan == 1
       Ambiente(1).IC31=zeros(3,N3);
       Ambiente_SC31=inv_rot_transl(Ambiente_SC3,-teta0,0);
       Ambiente(1).Scan_P1=Ambiente_SC31;
       Mapa_Global.Segmentos=Ambiente(1).Scan_P1;
       Ambiente(iScan).LC=0;
    else
       Ambiente_SC1=Ambiente(iScan-1).Scan;
       N1=size(Ambiente_SC1,2)/2; % N1 = número de segmentos em SC1
       fprintf('\n\n --> Scan Matching entre Scans %g e %g:',iScan-1,iScan);
       [rotacao,translacao,IC13,IC31]=Calcula_Segmentos_RT(Ambiente_SC1,Ambiente_SC3);
       rotacao=fix_teta(rotacao);
       Ambiente(iScan).RT=[rotacao translacao];
       Ambiente(iScan).IC13=IC13;
       Ambiente(iScan).IC31=IC31;
       % Plotar os 2 Scans na mesma figura
       Ambiente_SC31=inv_rot_transl(Ambiente_SC3,rotacao,translacao);
       figure(iScan-1);
       [h1_SClocal,h2_SClocal]=plot_2_Ambientes(Ambiente_SC1,Ambiente_SC31); hold on;
       plot_Pose([0;0;0]);
       str_title=sprintf('Scans %d e %d no SC local %d',iScan-1,iScan,iScan-1);
       title(str_title);
       % Plotar os corners de Ambiente_SC31
       Corners_SC31=Detect_Corners(Ambiente_SC31);
       plot(Corners_SC31(1,:),Corners_SC31(2,:),'ko','MarkerSize',8,'MarkerFaceColor','r');
       %axis([-8 8 -8 8]);
       % disp('Pause'); pause;
    end
    
    % Atualizar o Mapa_Global com a posição dos Segmentos e com as poses do robô após o
    % processamento dos dados obtidos por cada scan
    if iScan > 1
       % Cálculo de Pose_P1 = Poses no SC_Pose1
       Pose_P1=Ambiente(iScan-1).Pose_P1;
       rotacao_Scan=Ambiente(iScan).RT(1);
       translacao_Scan=Ambiente(iScan).RT(2);
       angulo=Pose_P1(3)+rotacao_Scan;
       Ambiente(iScan).Pose_P1(1:2,1)=Pose_P1(1:2,1)+translacao_Scan*[cos(angulo); sin(angulo)];
       Ambiente(iScan).Pose_P1(3)=angulo;
       %=============================================================
       % Correção das Poses por Loop Closure
       Ambiente(iScan).LC=0;
       if LC > 0
          if max(iScan==Scan_LC) % executar o procedimento de LC
             Ambiente(iScan).LC=1; % indicar que LC foi tentado neste Scan
             if isempty(Marcos_MG); error('Marcos_MG está vazio!'); end
             Ambiente=loop_closure(Ambiente,Marcos_MG,iScan,Nf_LC);
             Nf_LC=Nf_LC+1;
          end
       end
       %=============================================================
       % Cálculo dos Segmentos no SC da Pose P1
       Ambiente(iScan).Scan_P1=transf_coordenadas_Pose(Ambiente(iScan).Scan,Ambiente(iScan).Pose_P1);
       % Atualização do Mapa Global com os segmentos do Scan atual
       [Mapa_Global,Ambiente]=update_Mapa_Global(Mapa_Global,Ambiente,iScan);
       % Detectar os cantos no Mapa Global
       Mapa_Global.Cantos=Detect_Corners(Mapa_Global.Segmentos);
       %============================================================
       % Regras para selecionar os marcos para LOOP CLOSURE
       if iScan == 2
          ind1=find((Mapa_Global.Cantos(1,:)<2) & (Mapa_Global.Cantos(2,:)>-1));
          Marcos_MG=Mapa_Global.Cantos(:,ind1);
       end
       ind2=[];
       if iScan == 7
          ind1=find((Mapa_Global.Cantos(1,:)<0) & (Mapa_Global.Cantos(2,:)<-1));
          %ind2=find(0.4<(Mapa_Global.Cantos(1,:)) & (Mapa_Global.Cantos(1,:)<0.6));
          ind3=find(3<(Mapa_Global.Cantos(1,:)) & (Mapa_Global.Cantos(1,:)<5));
          Marcos_MG1=Mapa_Global.Cantos(:,[ind1 ind2 ind3]);
          Marcos_MG=[Marcos_MG Marcos_MG1];
       end
       if iScan == 12
          Ambiente_Corners_SC_Local=Detect_Corners(Ambiente(iScan).Scan);
          Ambiente_Corners_MG=transf_coordenadas_Pose(Ambiente_Corners_SC_Local,Ambiente(iScan).Pose_P1);
          ind1=find((7<Ambiente_Corners_MG(1,:)) & (Ambiente_Corners_MG(1,:)<9));
          Marcos_MG1=Ambiente_Corners_MG(:,ind1);
          
%           seg7 = [8.36 8.36; 0 1]- [0.72;2.30];
%           seg8 = [8.34 8.34; 2.85 1.85]- [0.72;2.30];
%           Marcos_MG1=[seg7, seg8];
          
          Marcos_MG=[Marcos_MG Marcos_MG1];
       end
       if iScan == 26
          Ambiente_Corners_SC_Local=Detect_Corners(Ambiente(iScan).Scan);
          Ambiente_Corners_MG=transf_coordenadas_Pose(Ambiente_Corners_SC_Local,Ambiente(iScan).Pose_P1);
          ind1=find((10<Ambiente_Corners_MG(1,:)) & (Ambiente_Corners_MG(1,:)<12));
          Marcos_MG1=Ambiente_Corners_MG(:,ind1);
          Marcos_MG=[Marcos_MG Marcos_MG1];
       end
       if iScan == 36
          Ambiente_Corners_SC_Local=Detect_Corners(Ambiente(iScan).Scan);
          Ambiente_Corners_MG=transf_coordenadas_Pose(Ambiente_Corners_SC_Local,Ambiente(iScan).Pose_P1);
          ind1=find(Ambiente_Corners_MG(1,:)>13.5);
          Marcos_MG1=Ambiente_Corners_MG(:,ind1);
          Marcos_MG=[Marcos_MG Marcos_MG1];
       end
       %Marcos_MG=Mapa_Global.Cantos;
       % Verificar se existem marcos selecionados para LC que estão muito próximos entre si        
       pause(0.3);
    end
    
    % Plotar Mapa Global e Pose atual no Mapa Global
    if exist('h1_MG','var'); delete(h1_MG); end
    h_fig_MG=figure(Nf_MG);
    [h1_MG,h2_MG]=plot_Ambiente(Mapa_Global.Segmentos,Nf_MG); hold on;
    delete(h2_MG);
    N_seg_MG=size(Mapa_Global.Segmentos,2)/2;
    N_cantos_MG=size(Mapa_Global.Cantos,2);
    str_LC='NS'; str_LC=str_LC(1+(Ambiente(iScan).LC>0));
    str_t1=sprintf('Mapa Global após Scan %d, LC=%s, No. Seg/Cantos = %d/%d',iScan,str_LC,N_seg_MG,N_cantos_MG);
    title(str_t1);
    % Plotar Pose em P1 no Mapa Global
    if iScan > 1
        Pose_linha=[Ambiente(iScan-1).Pose_P1 Ambiente(iScan).Pose_P1];
        plot(Pose_linha(1,:),Pose_linha(2,:),'r*-','LineWidth',1); hold on;
    end
    if exist('h_pose_MG','var'); delete(h_pose_MG); end
    h_pose_MG=plot_Pose(Ambiente(iScan).Pose_P1);
    % Plotar os marcos selecionados para LOOP CLOSURE
    if (~isempty(Marcos_MG)) %&& (LC > 0)
       if exist('h_marcos','var'); delete(h_marcos); end
       h_marcos=plot(Marcos_MG(1,:),Marcos_MG(2,:),'ko','MarkerSize',8,'MarkerFaceColor','g');
    end
    if (~isempty(Marcos_MG))
       if exist('h_marcos','var'); delete(h_marcos); end
       h_marcos=plot(Marcos_MG(1,:),Marcos_MG(2,:),'ko','MarkerSize',8,'MarkerFaceColor','g');
    end

    % Ajustar posição da figura
    Fig_Pos=get(h_fig_MG,'Position');
    set(h_fig_MG,'Position',[100 100 Fig_Pos(3) Fig_Pos(3)]);
    axis([-2 16 -4 2]); 
    %set(gca,'Xtick',[-2:16]);
    %set(gca,'Ytick',[-4:4]);
    pause(0.3);

    MG_Nseg=size(Mapa_Global.Segmentos,2)/2;
    MG_Ncantos=size(Mapa_Global.Cantos,2);
    fprintf('\n --> No. de segmentos no Mapa Global = %g',MG_Nseg);
    fprintf('\n --> No. de cantos no Mapa Global = %g',MG_Ncantos);
end
fprintf('\n-------------------------');

% Plotar todos as Poses (Pose_P1) na janela do Mapa Global
% Imprimir todas as Poses Pose_P1
figure(Nf_MG);
for iScan=1:NScans
    if iScan > 1
       % Plotar linha entre a pose atual e a pose anterior
       Pose_linha=[Ambiente(iScan-1).Pose_P1 Ambiente(iScan).Pose_P1]; hold on;
       plot(Pose_linha(1,:),Pose_linha(2,:),'r*-','LineWidth',1);
       fprintf('\n          RT: %8.3f graus,  %8.3f m',Ambiente(iScan).RT(1)*180/pi,Ambiente(iScan).RT(2));
    end
    Ambiente(iScan).Pose_P1(3)=fix_teta(Ambiente(iScan).Pose_P1(3)); hold on;
    Pose_P1(:,iScan)=Ambiente(iScan).Pose_P1;
    fprintf('\n Scan %2d: Pose_P1 (x,y,teta) = %8.3f m, %8.3f m, %8.3f graus',iScan,Pose_P1(1:2,iScan),Pose_P1(3,iScan)*180/pi);
    %h=plot_Pose(Ambiente(iScan).Pose_P1);
end
% Mudar a cor da 1a Pose e da última Pose no Mapa Global
hp1=plot_Pose(Ambiente(1).Pose_P1);
set(hp1(1),'MarkerFaceColor','c');
leg1 = hp1(1);
hp1=plot_Pose(Ambiente(NScans).Pose_P1);
set(hp1(1),'MarkerFaceColor','b');
leg2 = hp1(1);
legend([leg1 leg2],{'Postura inicial','Postura final'});
fprintf('\n-------------------------');

% Plotar apenas os mapas estimado e real em uma mesma figura
Nf_MG2=Nf_MG+100;
h_fig_MG2=figure(Nf_MG2);
[h1_MG2,h2_MG2]=plot_Ambiente(Mapa_Global.Segmentos,Nf_MG2); hold on;
delete(h2_MG2);
Plot_mapa_real;
title('Mapas Estimado e Real');
Fig_Pos=get(h_fig_MG2,'Position');
set(h_fig_MG2,'Position',[100 100 Fig_Pos(3) Fig_Pos(3)]);
axis([-2 16 -4 2]); 

% Imprimir Pose_P1 Real
% Pose_P1_Real=Pose_Real-Pose_Real(:,1); % Define ORIGEM do SC do Mapa Global como a 1a Pose Real
% for iScan=1:NScans
%     Pose_P1_Real(3,iScan)=fix_teta(Pose_P1_Real(3,iScan));
%     fprintf('\n Scan %2d: Pose_P1 Real (x,y,teta) = %8.3f m, %8.3f m, %8.3f graus',iScan,Pose_P1_Real(1:2,iScan),Pose_P1_Real(3,iScan)*180/pi);
% end
% fprintf('\n-------------------------');
% % Imprimir Erro_Pose
% for iScan=1:NScans
%     Ambiente(iScan).Erro_Pose=Pose_P1_Real(:,iScan)-Ambiente(iScan).Pose_P1;
%     Erro_Pose=Ambiente(iScan).Erro_Pose;
%     Erro_Pose(3)=fix_teta(Erro_Pose(3));
%     fprintf('\n Scan %2d: Erro Pose_P1 (x,y,teta) = %8.3f m, %8.3f m, %8.3f graus',iScan,Erro_Pose(1:2),Erro_Pose(3)*180/pi);
% end
% 
% % Plotar Poses estimadas e reais
% figure(91);
% plot([1:NScans],Pose_P1(1,:),'r-',[1:NScans],Pose_P1_Real(1,:),'b-'); hold on;
% plot([1:NScans],Pose_P1(1,:),'r*',[1:NScans],Pose_P1_Real(1,:),'bo'); hold on;
% xlabel('No. do Scan'); ylabel('X [m]'); title('Postura: Eixo x');
% legend('Postura calculada','Postura Real','Location','best'); grid on;
% %
% figure(92);
% plot([1:NScans],Pose_P1(2,:),'r-',[1:NScans],Pose_P1_Real(2,:),'b-'); hold on;
% plot([1:NScans],Pose_P1(2,:),'r*',[1:NScans],Pose_P1_Real(2,:),'bo'); hold on;
% xlabel('No. do Scan'); ylabel('Y [m]'); title('Postura: Eixo y');
% legend('Postura calculada','Postura Real','Location','best'); grid on;

figure(93);
Pose_P1(3,:)=Pose_P1(3,:)+(Pose_P1(3,:)>80*pi/180).*(-2*pi);
Pose_P1_Real(3,:)=Pose_P1_Real(3,:)+(Pose_P1_Real(3,:)>80*pi/180).*(-2*pi);
plot([1:NScans],Pose_P1(3,:)*180/pi,'r-',[1:NScans],Pose_P1_Real(3,:)*180/pi,'b-'); hold on;
plot([1:NScans],Pose_P1(3,:)*180/pi,'r*',[1:NScans],Pose_P1_Real(3,:)*180/pi,'bo'); hold on;
xlabel('No. do Scan'); ylabel('Ângulo [graus]'); title('Postura: Ângulo');
legend('Postura calculada','Postura medida','Location','best'); grid on;

if LC > 0
   figure(94);
   for iScan=1:NScans
       N_Marcos_LC(iScan)=numel(Ambiente(iScan).ind_Marcos_LC);
       if Ambiente(iScan).LC==0; N_Marcos_LC(iScan)=NaN; end
   end
   plot(N_Marcos_LC,'b*-'); grid on;
   xlabel('No. do Scan'); title('No. de marcos detectados para Loop Closure');
   axis([1 NScans 0 inf]);
end

MG_Nseg=size(Mapa_Global.Segmentos,2)/2;
MG_Ncantos=size(Mapa_Global.Cantos,2);
fprintf('\n --> No. de segmentos no Mapa Global = %g',MG_Nseg);
fprintf('\n --> No. de cantos no Mapa Global = %g',MG_Ncantos);

fprintf('\n'); disp('Fim.');

% ===================================================================

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
xlabel('Eixo X [m]'); ylabel('Eixo Y [m]');
end

function h=plot_Pose(Pose)
if (size(Pose,1)) ~= 3 || numel(Pose) ~=3; error('Entrada Pose deve ter 3 elementos em 1 coluna!'); end
L=0.3;
tf = ishold; % tf = 0 se hold = off
h(1)=plot(Pose(1),Pose(2),'bs','MarkerFaceColor','b','MarkerSize',12);
teta=Pose(3);
P2=Pose(1:2,1)+L*[cos(teta); sin(teta)];
if tf == 0; hold on; end
h(2)=plot([Pose(1) P2(1)],[Pose(2) P2(2)],'r-','LineWidth',2);
if tf == 0; hold off; end
end

function P3=rot_transl(P1,teta,distance)
% Dadas as coordenadas (X,Y) de pontos no SC1, mapeia essas coordenadas para SC3, onde:
% SC2 = SC1 com rotação teta (em rad),
% SC3 = SC2 com translação distance (em metros).
% Cada coluna de P1 corresponde a um ponto no SC1.
% Cada coluna de P3 corresponde a um ponto no SC3.
% dim(P1) = dim(P3) = 2xN, N = no. de colunas de P1 = no. de pontos em SC1.
N=size(P1,2);
if numel(P1) ~= 2*N; error('P1 com dimensão errada'); end
c1=cos(teta); s1=sin(teta);
Mrot=[c1 -s1 0; s1 c1 0; 0 0 1];
Mtr=[1 0 distance; 0 1 0; 0 0 1];
M1=Mrot*Mtr; M2=inv(M1);
P3=M2*[P1; 1];
P3=P3(1:2,:);
end

function P1=inv_rot_transl(P3,teta,distance)
% Dadas as coordenadas (X,Y) de pontos no SC3, mapeia essas coordenadas para SC1, onde:
% SC2 = SC1 com rotação teta (em rad),
% SC3 = SC2 com translação distance (em metros).
% Cada coluna de P3 corresponde a um ponto no SC3.
% Cada coluna de P1 corresponde a um ponto no SC1.
% dim(P1) = dim(P3) = 2xN, N = no. de colunas de P3 = no. de pontos em SC3.
N=size(P3,2); c1=cos(teta); s1=sin(teta);
Mrot=[c1 -s1; s1 c1];
Mtr=[c1; s1]*distance;
Mtr=repmat(Mtr,1,N);
P1=Mrot*P3+Mtr;
end

function P1=transf_coordenadas_pontos(P3,Ambiente,iScan)
% Transformação de coordenadas de 1 conjunto de pontos no SC do Scan no. iScan
% para as coordenadas do 1o Scan (SC_Pose1) considerando a sequência de RT efetuadas entre
% os Scan 1 e iScan.
% Cálculo de P1 = coordenadas dos pontos no SC_Pose1
if iScan > 1
   for k=iScan:-1:2
       rotacao=Ambiente(k).RT(1); translacao=Ambiente(k).RT(2);
       P1=inv_rot_transl(P3,rotacao,translacao);
       P3=P1;
   end
end
end

function P1=transf_coordenadas_Pose(P3,Pose)
% Transformação de P3 para P1, onde
% P3 = conjunto de pontos com coordenadas definidas no SC3 = SC local do robô,
% Pose = pose do robô (xr,yr,tetar) definida no SC1,
% P1 = conjunto de pontos com coordenadas definidas no SC1.
% SC1 = SC global, SC2 = SC1 com translação (tx,ty), SC3 = SC2 com rotação tetar (SC local no robÔ)
% dim(Pose)= 3x1
% dim(P1) = dim(P3) = 2xN, N = no. de colunas de P3 = no. de pontos em SC3.
if (numel(Pose) ~= 3) || (size(P3,1) ~= 2); error('Erro nas dimensões de Pose e/ou P3!'); end
N=size(P3,2);
tx=Pose(1); ty=Pose(2); teta=Pose(3);
c1=cos(teta); s1=sin(teta);
Mrot=[c1 -s1; s1 c1]; Mtr=repmat([tx;ty],1,N);
P1=Mrot*P3+Mtr;
end

function P3=inv_transf_coordenadas_Pose(P1,Pose)
% Transformação de P1 para P3, onde:
% P1 = conjunto de pontos com coordenadas definidas no SC1 = SC global.
% P3 = conjunto de pontos com coordenadas definidas no SC3 = SC local do robô,
% Pose = pose do robô (xr,yr,tetar) definida no SC1,
% SC1 = SC global, SC2 = SC1 com translação (tx,ty), SC3 = SC2 com rotação tetar (SC local no robô)
% dim(Pose)= 3x1
% dim(P1) = dim(P3) = 2xN, N = no. de colunas de P3 = no. de pontos em SC1.
if (numel(Pose) ~= 3) || (size(P1,1) ~= 2); error('Erro nas dimensões de Pose e/ou P1!'); end
N=size(P1,2);
tx=Pose(1); ty=Pose(2); teta=Pose(3);
c1=cos(teta); s1=sin(teta);
M1=[c1 -s1 tx; s1 c1 ty; 0 0 1];
P3=inv(M1)*[P1; ones(1,N)];
P3=P3(1:2,:);
end

function [Mapa_Global,Ambiente]=update_Mapa_Global(Mapa_Global,Ambiente,iScan)
Ambiente(iScan).New_Seg=find(Ambiente(iScan).IC31(2,:)==0);
Ambiente(iScan).Old_Seg=find(Ambiente(iScan).IC31(2,:)~=0);
N_seg=size(Ambiente(iScan).Scan,2)/2;
N_new_seg=length(Ambiente(iScan).New_Seg);
N_old_seg=length(Ambiente(iScan).Old_Seg);
fprintf('\n Scan %g: No. de segmentos/No. de novos segmentos = %g/%g',iScan,N_seg,N_new_seg);

% Verificar para cada Segmento do Scan atual se é possível fazer o merge dele com algum segmento
% que já está no Mapa_Global.
% Se sim, fazer o "merge" substituindo o Segmento que já está no mapa atual.
% Se não, adicionar o Segmento ao Mapa Global.
N_seg_MG=size(Mapa_Global.Segmentos,2)/2;
fprintf('\n Mapa Global contém %d segmentos',N_seg_MG);
Ambiente(iScan).ind_Seg_MG=[]; % armazena índice dos Segmentos adicionados ao Mapa Global
for k1=1:N_seg
    Seg_Scan=Ambiente(iScan).Scan_P1(:,[2*k1-1 2*k1]);
    %fprintf('\n Seg_Scan(%d) = (%8.3f,%8.3f),(%8.3f,%8.3f)',k1,Seg_Scan(1,1),Seg_Scan(2,1),Seg_Scan(1,2),Seg_Scan(2,2));
    flag_merge=0;
    for k2=1:N_seg_MG
        Seg_MG=Mapa_Global.Segmentos(:,[2*k2-1 2*k2]);
        Seg3=merge_segments_teste(Seg_Scan,Seg_MG);
%         Seg3=merge_segments(Seg_Scan,Seg_MG);
%         Seg3=merge_seg(Seg_Scan,Seg_MG);
        if ~isempty(Seg3)
           fprintf('\n  Merge de Seg %2d (Scan %d) + Seg %2d de MG',k1,iScan,k2);
           Mapa_Global.Segmentos(:,[2*k2-1 2*k2])=Seg3;         
           flag_merge=flag_merge+1;
           %fprintf('\n  Seg_MG   = (%8.3f,%8.3f),(%8.3f,%8.3f)',Seg_MG(1,1),Seg_MG(2,1),Seg_MG(1,2),Seg_MG(2,2));
           %fprintf('\n  Seg3     = (%8.3f,%8.3f),(%8.3f,%8.3f)',Seg3(1,1),Seg3(2,1),Seg3(1,2),Seg3(2,2));
        end
    end
    % Se não foi feito nenhum MERGE, adicionar o segmento de iScan ao Mapa Global
    if flag_merge == 0
       Mapa_Global.Segmentos=[Mapa_Global.Segmentos Seg_Scan];
       % armazena índice dos Segmentos adicionados ao Mapa Global
       Ambiente(iScan).ind_Seg_MG=[Ambiente(iScan).ind_Seg_MG k1];
       fprintf('\n Adicionado Scan %2d, Segmento %2d ao Mapa Global',iScan,k1);
    end
end
end

% Função que tenta fazer "merge" de 2 segmentos de reta
% Entradas: coordenadas (x,y) dos 2 segmentos Seg1 e Seg2
%           Seg1 = [x1 x2; y1 y2] onde (x1,y1) e (x2,y2) = "end points" 1 e 2
%           Seg2 = [x3 x4; y3 y4] onde (x3,y3) e (x4,y4) = "end points" 3 e 4
% Saídas: Seg3 =[x5 x6; y5 y6] com as coordenadas do segmento resultante
%         Seg3 = [] se não for possível fazer o "merge" de Seg1 e Seg2
% CLNJ, 10/11/2022
function Seg3=merge_seg(Seg1,Seg2)
ang_min=5;  % ângulo mínimo em graus
dist_min=0.2; % distância mínima em metros
ang_min=30;  % ângulo mínimo em graus
dist_min=0.5; % distância mínima em metros

% 1.1 - Se segmentos são próximos de verticais, torná-los verticais redefinindo a coordenada x
x=[Seg1(1,:) Seg2(1,:)];
if abs(x(1)-x(2)) < dist_min
    x_med=(x(1)+x(2))/2; Seg1(1,:)=[x_med x_med];
    if Seg1(2,2) < Seg1(2,1); Seg1(2,:) = Seg1(2,[2 1]); end
end
if abs(x(3)-x(4)) < dist_min
   x_med=(x(3)+x(4))/2; Seg2(1,:)=[x_med x_med];
   if Seg2(2,2) < Seg2(2,1); Seg2(2,:) = Seg1(2,[2 1]); end
end
% 1.2 - Se segmentos são próximos de horizontais, torná-los horizontais redefinindo a coordenada y
y=[Seg1(2,:) Seg2(2,:)];
if abs(y(1)-y(2)) < dist_min; y_med=(y(1)+y(2))/2; Seg1(2,:)=[y_med y_med]; end
if abs(y(3)-y(4)) < dist_min; y_med=(y(3)+y(4))/2; Seg2(2,:)=[y_med y_med]; end

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
p=Seg1(:,2)-Seg1(:,1); teta(1)=atan2(p(2),p(1))*180/pi;
dist(1)=abs(p(1)*Seg1(2,1)-p(2)*Seg1(1,1))/sqrt(p'*p);
p=Seg2(:,2)-Seg2(:,1); teta(2)=atan2(p(2),p(1))*180/pi;
dist(2)=abs(p(1)*Seg2(2,1)-p(2)*Seg2(1,1))/sqrt(p'*p);
teta=fix_teta(teta*pi/180);
teta=teta*180/pi;
% for i=1:2
%     if teta(i) <= -90; teta(i)=teta(i)+180; end
%     if teta(i) > 90;  teta(i)=teta(i)-180; end
% end

%[Seg1, Seg2], [dist teta]
% 4. Tentar fazer o MERGE dos Segmentos
Seg3=[];
if (abs(teta(2)-teta(1)) < ang_min) && (abs(dist(2)-dist(1)) < dist_min)
   % Tentar MERGE dos segmentos
   % Se os segmentos forem verticais, ordenar os pontos pela coordenada Y
   %disp("MERGE talvez possível.");
   if (abs(sum(teta)/2)) >= 90
      %disp("Caso segmentos verticais.");
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
       %disp("MERGE feito!");
       %plot(Seg3(1,:),Seg3(2,:),'g+');
   else
       %disp("MERGE não feito!");
   end
else
   %disp("MERGE NOT POSSIBLE!")
end
end

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

% Loop Closure
% Marcos_MG = matriz com os pontos no SC do Mapa Global que serão usados como marcos para
% a execução do procedimento de LC.
function Ambiente=loop_closure(Ambiente,Marcos_MG,iScan,Nf)
figure(Nf);

fprintf('\n --> Fechando LOOP, Scan = %d',iScan);
[h1,h2]=plot_Ambiente(Ambiente(iScan).Scan,Nf); hold on;
h=plot_Pose([0; 0; 0]);
corners_iScan=Detect_Corners(Ambiente(iScan).Scan);
plot(corners_iScan(1,:),corners_iScan(2,:),'ko','MarkerSize',8,'MarkerFaceColor','r');
str=1:size(corners_iScan,2); str=num2cell(str);
text(corners_iScan(1,:),corners_iScan(2,:)+0.2,str,'Color','r');
Fig_Pos=get(gcf,'Position'); set(gcf,'Position',[1000 100 Fig_Pos(3) Fig_Pos(3)]);
%
Marcos_MG_SC3=inv_transf_coordenadas_Pose(Marcos_MG,Ambiente(iScan).Pose_P1);
plot(Marcos_MG_SC3(1,:),Marcos_MG_SC3(2,:),'ko','MarkerSize',8,'MarkerFaceColor','g');
str1_title=[];
ind_Seg=0;
Seg_marcos=[];
ind_Marcos=[];
Marcos_MG_new=[];
for i=1:size(Marcos_MG_SC3,2)
   dist=corners_iScan-repmat(Marcos_MG_SC3(:,i),1,size(corners_iScan,2));
   dist=[dist(1,:).*dist(1,:); dist(2,:).*dist(2,:)];
   dist=sqrt(dist(1,:)+dist(2,:));
   [dist_min(i),ind_dist_min(i)]=min(dist);
   if dist_min(i)<0.5  % Achou match entre Marcos_MG_SC3 e corners_iScan
      Marcos_MG_new=[Marcos_MG_new Marcos_MG(:,i)];
      ind_Seg=ind_Seg+1;
      ind_Marcos=[ind_Marcos ind_dist_min(i)];
      Seg_marcos=[Marcos_MG_SC3(:,i) corners_iScan(:,ind_dist_min(i))];
      plot(Seg_marcos(1,:),Seg_marcos(2,:),'b-');
      str1_title=[str1_title sprintf(' %d',ind_dist_min(i))];
   end
end
if isempty(Seg_marcos)
   warning('LC foi tentado mas não foram encontrados cantos perto dos marcos!')
end
str2_title=sprintf('Scan %d, SC local, Marcos (%d)= ',iScan,numel(ind_Marcos));
title([str2_title str1_title]);
% iScan = 21: ind_Marcos=[10  9  8  6];
% iScan = 23: ind_Marcos=[ 9  8  7  6];
% iScan = 28: ind_Marcos=[ 9 10 11 12];
% iScan = 29: ind_Marcos=[ 7  8  9 10];
fprintf('\n ind_Marcos = '); fprintf(' %d, ',ind_Marcos);
Marcos_iScan=corners_iScan(:,ind_Marcos);
Ambiente(iScan).ind_Marcos_LC=ind_Marcos;
Ambiente(iScan).Marcos_LC=Marcos_iScan;
if numel(ind_Marcos) ~= numel(unique(ind_Marcos))
   error('Erro na correspondência entre Marcos locais e Marcos globais: ind_Marcos com elementos repetidos');
end
for i=1:size(Marcos_iScan,2)
   m=Marcos_iScan(:,i);
   Marcos_Dist(i)=sqrt(m'*m);
   Marcos_Ang(i)=atan2(m(2),m(1));       
end
hold off;   

% [h3,h4]=plot_Ambiente(Mapa_Global.Segmentos,Nf+1); hold on;
% corners_MG=Detect_Corners(Mapa_Global.Segmentos);
% plot(corners_MG(1,:),corners_MG(2,:),'ko','MarkerSize',8,'MarkerFaceColor','r');
% title(['Scan ' num2str(iScan) ', SC global']);
% h=plot_Pose(Ambiente(iScan).Pose_P1);

Pose_old=Ambiente(iScan).Pose_P1;
if size(Marcos_MG_new,2) >= 3
   Pose0=[mean(Marcos_MG_new(1,:)); mean(Marcos_MG_new(2,:)); 0];
   [Pose1,feval]=Loop_Closure_Pose(Pose0,Marcos_MG_new,Marcos_Ang,Marcos_Dist);
   Ambiente(iScan).Pose_P1=Pose1;
   Ambiente(iScan).feval_LC=feval;
   if abs(feval) > 0.1; warning('==> abs(feval) > 0.1!'); end
   fprintf('\n Pose_old: %8.3f m, %8.3f m, %8.3f graus',Pose_old(1:2),Pose_old(3)*180/pi);
   fprintf('\n Pose_new: %8.3f m, %8.3f m, %8.3f graus',Pose1(1:2),Pose1(3)*180/pi);
   % hold on; h=plot_Pose(Ambiente(iScan).Pose_P1);
   % set(h(1),'MarkerFaceColor','k'); set(h(1),'Color','k');
   % Fig_Pos=get(gcf,'Position'); set(gcf,'Position',[1000 100 Fig_Pos(3) Fig_Pos(3)]);
else
   fprintf('\n No. de marcos globais com correspondência local (%d) insuficiente!!',size(Marcos_MG_new,2));
end
end

function teta=fix_teta(teta)
% Retorna teta no intervalo [-pi,pi]
ind=find(abs(teta)>=pi);
for i=ind
    while teta(i) < -pi; teta(i) = teta(i) + 2*pi; end
    while teta(i) >  pi; teta(i) = teta(i) - 2*pi; end
end
end

% Função para plotar 2 ambientes (conjunto de segmentos de reta) no mesmo SC
function [h1,h2]=plot_2_Ambientes(Ambiente1,Ambiente2)
if isempty(Ambiente1) || isempty(Ambiente2); error('Ambiente1 ou Ambiente2 são vazios!'); end
N1=size(Ambiente1,2)/2; % no. de segmentos do Ambiente1
if numel(Ambiente1) ~= 4*N1; error('Ambiente1 com dimensões erradas!'); end
N2=size(Ambiente2,2)/2; % no. de segmentos do Ambiente2
if numel(Ambiente2) ~= 4*N2; error('Ambiente2 com dimensões erradas!'); end

Sep=[NaN; NaN];
% Plotar segmentos de reta da variável Ambiente1
for i=1:N1; Ambiente11(:,[3*i-2 3*i-1 3*i])= [Ambiente1(:,[2*i-1 2*i]) Sep]; end
h1(1)=plot(Ambiente11(1,:),Ambiente11(2,:),'-k','LineWidth',2); hold on;
h1(2)=plot(Ambiente11(1,:),Ambiente11(2,:),'*k','LineWidth',2);
for i=1:N1; h2(i)=text(Ambiente11(1,3*i-1)-0.2,Ambiente11(2,3*i-1),num2str(i)); end
for i=1:N1; h2(i+N1)=text(Ambiente11(1,3*i-2)+0.2,Ambiente11(2,3*i-2),num2str(i)); end
% grid on; axis equal;
% xlabel('X [m]'); ylabel('y [m]');

% Plotar segmentos de reta da variável Ambiente2
for i=1:N2; Ambiente22(:,[3*i-2 3*i-1 3*i])= [Ambiente2(:,[2*i-1 2*i]) Sep]; end
h3(1)=plot(Ambiente22(1,:),Ambiente22(2,:),'r--','LineWidth',2);
h3(2)=plot(Ambiente22(1,:),Ambiente22(2,:),'rd','LineWidth',2);
for i=1:N2; h4(i)=text(Ambiente22(1,3*i-1)-0.4,Ambiente22(2,3*i-1),num2str(i),'Color','red'); end
for i=1:N2; h4(i+N2)=text(Ambiente22(1,3*i-2)+0.4,Ambiente22(2,3*i-2),num2str(i),'Color','red'); end
grid on; axis equal;
xlabel('Eixo X [m]'); ylabel('Eixo Y [m]');

h1=[h1 h3]; h2=[h2 h4];
end

