% Função MATLAB para "Loop Closure" no problema de SLAM de um robô em abiente 2D
% Entradas:
% - Matriz Pm (dim 2xN): a posição (x,y) no SC global dos N marcos do ambiente selecionados
%   para LC,
% - Vetores teta e dist (dim 1xN): os valores medidos no ambiente real de rotação (em rad) e
%   de distância entre o robô e cada marco,
%   essa rotação é a rotação que o robô deve fazer no ambiente real para apontar para o marco,
% - Vetor Pose0 (dim 3x1): estimativa inicial da Pose do robô (x_r, y_r, teta_r) no SC global
% Saídas:
% - Pose (dim 1x3): Pose calculada do robô (x_r, y_r, teta_r) no SC global.
% - feval: cálculo da função de erro a ser minimizada, use o teste "abs(feval)>0.1" como indicativo
%   que a solução "Pose" não deve ser aceita.
% CLNJ, 12/11/2022
function [Pose,feval]=Loop_Closure_Pose(Pose0,Pm,teta,dist)
Pm=[Pm; teta; dist];
teta=fix_teta(teta); % força teta no intervalo [-pi,pi]
options = optimset('Display','off','TolFun',1e-5,'TolX',1e-5);
f=@(Pose) erro_Pose1(Pose,Pm);
[Pose,feval,exitflag,output]=fminsearch(f,Pose0,options);
%fprintf('\n Pose estimada = [%8.3f m, %8.3f m, %8.3f graus], %g m',Pose(1:2),Pose(3)*180/pi);
fprintf('\n F_Eval/exitflag/no. iterações = %8.3f / %d / %d\n',feval,exitflag,output.iterations);
end

function J=erro_Pose1(Pose,Pm)
N=size(Pm,2);
tetam=Pm(3,:); distm=Pm(4,:);
teta=tetam+Pose(3);
xc=Pose(1)+distm.*cos(teta);
yc=Pose(2)+distm.*sin(teta);
Erro_x=Pm(1,:)-xc;
Erro_y=Pm(2,:)-yc;
J=Erro_x*Erro_x'+Erro_y*Erro_y';
end

function J=erro_Pose2(Pose,Pm)
N=size(Pm,2);
dPm=zeros(2,N); tetac=zeros(1,N); distc=zeros(1,N);
dPm=Pm(1:2,:)-repmat(Pose(1:2,1),1,N);
for i=1:N
    tetac(1,i)=atan2(dPm(2,i),dPm(1,i))-Pose(3);
    distc(1,i)=sqrt(dPm(:,i)'*dPm(:,i));
end
Erro_teta=Pm(3,:)-tetac;
% % força Erro_teta no intervalo [-pi,pi]
Erro_teta=fix_teta(Erro_teta);
Erro_dist=Pm(4,:)-distc;
J=(180/(10*pi))*sqrt(Erro_teta*Erro_teta')+sqrt(Erro_dist*Erro_dist');
% J = Erro_teta*Erro_teta' + Erro_dist*Erro_dist';
% J=[Erro_teta Erro_dist];
% J=max(abs(J));
end

function teta=fix_teta(teta)
% Retorna teta no intervalo [-pi,pi]
ind=find(abs(teta)>=pi);
for i=ind
    while teta(i) < -pi; teta(i) = teta(i) + 2*pi; end
    while teta(i) >  pi; teta(i) = teta(i) - 2*pi; end
end
end
