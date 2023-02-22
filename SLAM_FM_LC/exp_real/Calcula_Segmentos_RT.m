% MATLAB 2017b script Calcula_Segmentos_Rotacao_Translacao.m (versão 5)
% Script MATLAB para:
% 1) Carregar as matrizes Ambiente_SC1 e Ambiente_SC3 com as coordenadas (X,Y) dos "end points"
%    dos N segmentos de reta que compõe o Ambiente em SC1 e em SC3.
% 2) Calcular o melhor par RT (Rotação seguida de Translação) que mapeia Ambiente_SC1 em
%    Ambiente_SC3.
%    SC = Sistema de Coordenadas,
%    SC1 = SC original, SC2 = SC1 com rotação, SC3 = SC2 com translação.
% 3) Indicar a correspondência entre os segmentos de Ambiente_SC1 e Ambiente_SC3 para o melhor
%    par (R,T).
% CLNJ e Luciana Lemos, 30/08-02/09/2022, 23,25-26/09/2022, 23/10/2022
% clc; clearvars; close all;
% warning('off');

function [rotacao,translacao,IC13,IC31]=Calcula_Segmentos_RT(Ambiente_SC1,Ambiente_SC3)
%load Ambiente_Segmentos;
N1=size(Ambiente_SC1,2)/2; % N1 = número de segmentos em SC1
N3=size(Ambiente_SC3,2)/2; % N3 = número de segmentos em SC3
fprintf('\n No. de segmentos em Ambiente SC1 = %g',N1);
fprintf('\n No. de segmentos em Ambiente SC3 = %g',N3);
% Tentar encontrar a transformação de pareamento entre cada segmento de SC1 e cada segmento de SC3
X_sol=NaN(N1,N3,6); 
x0=[0;0];
%options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
%options = optimoptions('fsolve','Display','none');
options = optimset('Display','off');
options = optimset('Display','off','TolFun',1e-8,'TolX',1e-8);
ind_sol=0; % contador de tentativa de solução
ind_sol_val=0; % contador de soluções válidas
X_sol_val=zeros(5,N1*N3*2);
fprintf('\n i =');
for i=1:N1
    fprintf(' %g',i);
    P(:,1:2)=Ambiente_SC1(:,[2*i-1 2*i]);
    for j=1:N3
        % fprintf(' %g',j);
        % Procurar por correspondência entre o segmento i de SC1 (pontos 2*i-1 e 2*i) 
        % - se k = 1: segmento j de SC3 com pontos na ordem direta (pontos 2*j-1 e 2*j)
        % - se k = 2: segmento j de SC3 com pontos na ordem invertida (pontos 2*j e 2*j-1)
        for k=1:2
            P(:,3:4)=Ambiente_SC3(:,[2*j-1 2*j]);
            if k==2; P(:,3:4)=Ambiente_SC3(:,[2*j 2*j-1]); end
            f=@(x) func_rot_transl(x,P);
            %[x,fval,exitflag]=fsolve(f,x0,options);
            [x,fval,exitflag]=fminsearch(f,x0,options);
            X_sol(i,j,3*k-2)=x(1);
            X_sol(i,j,3*k-1)=x(2);
            X_sol(i,j,3*k)=exitflag;
            ind_sol=ind_sol+1;
            if (exitflag > 0) && (fval < 0.1)
               ind_sol_val=ind_sol_val+1;
               %X_sol_val(:,ind_sol_val)=[ind_sol; x(1); x(2); sqrt(fval'*fval); exitflag];
               X_sol_val(:,ind_sol_val)=[ind_sol; x(1); x(2); fval; exitflag];
            end
        end
    end
end
X_sol_val=X_sol_val(:,1:ind_sol_val);
if ind_sol_val==0; fprintf('\n'); error('==> Erro! Não foram encontradas soluções válidas!'); end
fprintf('\n %g Soluções válidas/%g Soluções:',ind_sol_val,ind_sol);
fprintf('\n Índice         : '); fprintf(' %d,',X_sol_val(1,:));
fprintf('\n Rotação [graus]: '); fprintf(' %g,',X_sol_val(2,:)*180/pi);
fprintf('\n Translação  [m]: '); fprintf(' %g,',X_sol_val(3,:));
fprintf('\n Módulo da Saída: '); fprintf(' %.2e,',X_sol_val(4,:));
fprintf('\n Exit Flag      : '); fprintf(' %d,',X_sol_val(5,:));

% Agrupar as soluções próximas por rotação e por translação
X_sol2=uniquetol(X_sol_val(2:3,:)',1e-3,'ByRows',true);
X_sol2=X_sol2';
fprintf('\n\n %g Soluções válidas agrupadas:',size(X_sol2,2));
fprintf('\n Rotação [graus]: '); fprintf(' %g,',X_sol2(1,:)*180/pi);
fprintf('\n Translação  [m]: '); fprintf(' %g,',X_sol2(2,:));

% Para cada proposta de solução (Rotação,Translação), calcular MC (Medida de Correspondência)
% entre o Ambiente_SC1 e o Ambiente_SC3 mapeado para SC1 usando a proposta de solução RT.
% O valor de MC é calculado somando uma função da distância entre os "end points"
% de cada par [Segmento i de SC1, Segmento j de SC3 (ordem direta e ordem invertida)].
Ns=size(X_sol2,2);
M=zeros(N1,N3,Ns);
MS=zeros(N1,Ns);
MC=zeros(Ns,1);
for is=1:Ns % loop da solução
    teta=X_sol2(1,is);
    distance=X_sol2(2,is);
    Ambiente_SC31=inv_rot_transl(Ambiente_SC3,teta,distance);
    for i=1:N1 % loop do segmento do Ambiente SC1
        S1=Ambiente_SC1(:,[2*i-1 2*i]);
        for j=1:N3  % loop do segmento do Ambiente SC3 (ordem direta e invertida dos "end points")
            for k=1:2
                S31=Ambiente_SC31(:,[2*j-1 2*j]);
                if k==2; S31=Ambiente_SC31(:,[2*j 2*j-1]); end
                d1=S1(:,1)-S31(:,1); d11=sqrt(d1'*d1); 
                d2=S1(:,2)-S31(:,2); d22=sqrt(d2'*d2);
                d(k)=d11+d22;
            end
            dmin=min(d);
            Ls1=S1(:,1)-S1(:,2);Ls1=sqrt(Ls1'*Ls1);
            Ls2=S31(:,1)-S31(:,2);Ls2=sqrt(Ls2'*Ls2);
            % Penalizar diferença entre os tamanhos dos segmentos
            Ld=abs(Ls1-Ls2); Lp=1+Ld;
            dmin=Lp*dmin;
            % Md(i,j,is) = correspondência entre Segmento i de SC1 e Segmento j de S3
            % (melhor entre ordem direta e ordem inversa)
            Md(i,j,is)=exp(-(dmin^2));
            % Md(i,j,is)=exp(-dmin);
        end
        MS(i,is)=sum(Md(i,:,is));
        %MS(i,is)=max(Md(i,:,is));
    end
    MC(is)=sum(MS(:,is));

end
fprintf('\n MC = '); fprintf(' %g,',MC);
[max_MC,imax_MC]=max(MC);
fprintf('\n\n Melhor solução = Solução %g',imax_MC);
fprintf('\n Rotação [graus]: '); fprintf(' %g,',X_sol2(1,imax_MC)*180/pi);
fprintf('\n Translação  [m]: '); fprintf(' %g.',X_sol2(2,imax_MC));
disp(' ');

% IC13 = índice da melhor correspondência de cada segmento de SC1 para os segmentos de SC3
% IC13 = matriz 3xN1, N1 = no. de segmentos em SC1
% Para cada coluna de IC13 temos:
% 1a linha: menor distância encontrada entre os "end points" do segmento dessa coluna de SC1 e
%           os "end points" dos segmentos em SC31 (segmentos de SC3 transformados para SC1
%           usando "rotacao" e "translacao" a partir de SC1)
% 2a linha: no. do segmento de SC31 com a distância indicada na 1a linha de IC13
% 3a linha: 0 ou 1, indica a ordem dos "end points" do segmento em SC31 indicado na 2a linha de IC13
%           0: ordem direta, 1: ordem invertida
rotacao=X_sol2(1,imax_MC);
translacao=X_sol2(2,imax_MC);
IC13=Calcula_IC13(Ambiente_SC1,Ambiente_SC3,rotacao,translacao);
% Comparar linhas 2 e 3 de IC13 com as linhas 1 e 2 Ind_Seg
% IC13(2:3,:), Ind_Seg,
% IC13(2:3,:)-Ind_Seg,

% IC31 = índice de correspondência entre os segmentos de SC3 e os segmentos de SC1
% Indica para cada segmento de SC3 o segmento de SC1 com melhor correspondência para ele.
% IC31 = matriz 3xN3, N3 = no. de segmentos em SC1
% Para cada coluna de IC31 temos:
% 1a linha: menor distância encontrada entre os "end points" do segmento dessa coluna de SC3
%           (transformados para SC1 usando "rotacao" e "translacao" a partir de SC1) e
%           os "end points" dos segmentos em SC1
% 2a linha: no. do segmento de SC1 com a distância indicada na 1a linha de IC31
% 3a linha: no. de segmentos de SC1 que indicam correspondência como segmento dessa coluna de SC3
% - Se IC31(2,j) = 0: o segmento j de SC3 não contém correspondente em SC1
IC31=zeros(3,N3);
for j=1:N3
    ind13=find(IC13(2,:)==j);
    IC31(3,j)=length(ind13);
    if IC31(3,j) == 1
       IC31(1,j)=IC13(1,ind13);
       IC31(2,j)=ind13;
    end
    if IC31(3,j) > 1
       fprintf('\n Segmento de SC31 %g corresponde aos Segmentos de SC1:',j); fprintf(' %g,',ind13);
       [IC31(1,j),ind_min]=min(IC13(1,ind13));
       IC31(2,j)=ind13(ind_min);
       fprintf('\n Escolha do melhor correspondente em SC1: %g',IC31(2,j));
    end    
end
ind31=find(IC31(2,:)==0);
if length(ind31) >= 1
   fprintf('\n Segmentos de SC31 sem correspondente em SC1:');
   fprintf(' %g,',ind31);
end
% IC13(2,:),
% IC31(2,:),
fprintf('\n Correspondência dos segmentos de SC1 para SC3:'); fprintf('% g',IC13(2,:));
fprintf('\n Correspondência dos segmentos de SC31 para SC1:'); fprintf('% g',IC31(2,:));

% Limpeza final
% clear X_sol X_sol2 X_sol_val M MC MS Md x x0 options Ambiente_SC31;
% clear S1 S31 d d1 d11 d2 d22 distance dmin exitflag f fval i imax_MC ind13 ind31;
% clear ind_sol ind_sol_val is j k max_MC Ns P teta;
fprintf('\n'); disp('Fim.');
end

function Fd = func_rot_transl(x,P)
% Dados: os 2 "end points" de um segmento de reta em SC1, denominado de SR1,
%        os 2 "end points" de um segmento de reta em SC3, denominado de SR3,
%        ângulo e distância,
% essa função calcula:
% 1) SR31 = "end points' de SR3 projetados no SC1 onde (SR31 + rotação + translação) = SR3,
% 2) F = diferença entre as coordenadas (x,y) dos 2 "end points" de SR31 e
%        dos 2 "end points" de SR1, onde dim(F) = 4x1
% 3) Fd = Fd = sqrt(F'*F), onde dim(Fd) = 1x1.
% Essa função retorna escalar Fd = sqrt(F'*F) onde F = vetor 4x1 com as diferenças entre:
% - as coordenadas (x,y) de 2 pontos em SC1 (colunas 1 e 2 de P), e
% - as coordenadas (x,y) de 2 pontos em SC3 (colunas 3 e 4 de P) mapeados para SC1 usando
%   rotação e translação (x(1) e x(2), respectivamente).
% x(1) = teta [rad]
% x(2) = distance [m]
% P = matriz 2x4 com as coordenadas (x,y) dos 4 pontos dos 2 segmentos de reta
% 1a linha de P: coordenadas x dos 4 pontos
% 2a linha de P: coordenadas y dos 4 pontos
% Colunas 1 e 2 de P: coordenadas dos "end points" do segmento de reta em SC1
% Colunas 3 e 4 de P: coordenadas dos "end points" do segmento de reta em SC3
% Exemplo:
% P(:,1:2)=[0 0; 0 1]; % S1 = segmento vertical de (0,0) até (0,1)
% P(:,3:4)=[1 2; 0 0]; % S2 = segmento horizontal de (1,0) até (2,0)
% x0=[0;0]; f=@(x)func_rot_transl(x,P);
% options = optimoptions('fsolve','Display','none','PlotFcn',@optimplotfirstorderopt);
% x=fsolve(f,x0,options);
c1=cos(x(1)); s1=sin(x(1));
A=[c1, -s1; s1, c1]; B=[c1; s1];
F(1:2,1)=A*P(:,3)+B*x(2)-P(:,1); % Diferença entre pontos armazenados em P(:,1) e P(:,3)
F(3:4,1)=A*P(:,4)+B*x(2)-P(:,2); % Diferença entre pontos armazenados em P(:,2) e P(:,4)
%Fd=sqrt(F'*F);
Fd=F'*F;
% d1=P(:,1)-P(:,2);L1=sqrt(d1'*d1); % L1 = comprimento do segmento 1
%Fd=(F'*F)/L1;
%Fd=max(abs(F));
end

function P1=inv_rot_transl(P3,teta,distance)
% Dadas as coordenadas (X,Y) de pontos no SC3, mapeia essas coordenadas para SC1, onde:
% SC2 = SC1 com rotação teta (em rad),
% SC3 = SC2 com translação distance (em metros).
% Cada coluna de P3 corresponde a um ponto no SC3.
% Cada coluna de P1 corresponde a um ponto no SC1.
% Output: P1 contém 2xN, N = no. de colunas de P3 = no. de pontos em SC3.
N=size(P3,2); c1=cos(teta); s1=sin(teta);
Mrot=[c1 -s1; s1 c1];
Mtr=[c1; s1]*distance;
Mtr=repmat(Mtr,1,N);
P1=Mrot*P3+Mtr;
end

function IC13=Calcula_IC13(Ambiente_SC1,Ambiente_SC3,rotacao,translacao)
% Calcula IC13 = índice de correspondência entre os segmentos em Ambiente_SC1 e Ambiente_SC3
% IC13 = matriz 3xN1, N1 = no. de segmentos em SC1
% Para cada coluna de IC13 temos:
% 1a linha: menor distância encontrada entre os "end points" do segmento dessa coluna de SC1 e
%           os "end points" dos segmentos em SC31 (segmentos de SC3 transformados para SC1
%           usando "rotacao" e "translacao" a partir de SC1)
% 2a linha: no. do segmento de SC31 com a distância indicada na 1a linha de IC13
% 3a linha: 0 ou 1, indica a ordem dos "end points" do segmento em SC31 indicado na 2a linha de IC13
%           0: ordem direta, 1: ordem invertida
% Output: IC13 = matriz 3xN1, N1 = no. de segmentos em SC1
N1=size(Ambiente_SC1,2)/2;
N3=size(Ambiente_SC3,2)/2;
M=zeros(1,N3);
Ambiente_SC31=inv_rot_transl(Ambiente_SC3,rotacao,translacao);
for i=1:N1 % loop do segmento do Ambiente SC1
    S1=Ambiente_SC1(:,[2*i-1 2*i]);
    for j=1:N3  % loop do segmento do Ambiente SC31 (ordem direta e invertida dos "end points")
        for k=1:2
            S31=Ambiente_SC31(:,[2*j-1 2*j]);
            if k==2; S31=Ambiente_SC31(:,[2*j 2*j-1]); end
            d1=S1(:,1)-S31(:,1); d11=sqrt(d1'*d1); 
            d2=S1(:,2)-S31(:,2); d22=sqrt(d2'*d2);
            d(k)=d11+d22;
        end
        [dmin,idmin]=min(d); % idmin = 1 ou 2
        M(1:2,j)=[dmin;idmin-1];
    end
    [v_min,ind_min]=min(M(1,:));
    IC13(1:3,i)=[v_min; ind_min; M(2,ind_min)];
end

% Verificar em IC13 se mais de 1 segmento em SC1 corresponde um mesmo segmento em SC31
% Se isso acontecer, escolher o segmento de SC1 que melhor corresponde ao segmento em SC31
for j=1:N3
    ind13=find(IC13(2,:)==j);
    if length(ind13) > 1
       fprintf('\n Segmento de SC31 %g corresponde aos Segmentos de SC1:',j); fprintf(' %g,',ind13);
       fprintf('\n  Antes da correção: %g %g %g',IC13(:,ind13));
       [~,ind_min]=min(IC13(1,ind13));
       ind_ok=ind13(ind_min);
       for k=1:length(ind13); if ind13(k) ~= ind_ok; IC13(2,ind13(k))=0; end; end
       fprintf('\n  Segmento com melhor correspondência em SC1: %g',ind_ok);
       fprintf('\n  Após correção: %g %g %g',IC13(:,ind13));
    end
end
% Indicar os segmentos em SC1 sem correspondente em SC31
ind13=find(IC13(2,:)==0);
if length(ind13) >= 1
   fprintf('\n Segmentos de SC1 sem correspondente em SC31:');
   fprintf(' %g,',ind13);
end
end

function h=plot_Pose(Pose)
if numel(Pose) ~=3; error('Pose deve ter 3 elementos em 1 coluna!'); end
L=0.3;
tf = ishold; % tf = 0 se hold = off
h(1)=plot(Pose(1),Pose(2),'bs','MarkerFaceColor','b','MarkerSize',12);
teta=Pose(3);
P2=Pose(1:2,1)+L*[cos(teta); sin(teta)];
if tf == 0; hold on; end
h(2)=plot([Pose(1) P2(1)],[Pose(2) P2(2)],'r-','LineWidth',2);
if tf == 0; hold off; end
end

