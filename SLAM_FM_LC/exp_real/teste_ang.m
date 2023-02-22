% 0 <= teta <= 180 graus
teta=[  0 179   0   0 175
      179   0 180 175   0];
for i=1:size(teta,2)
    dif_ang(i)=calc_dif_ang(teta(1,i),teta(2,i));
end
dif_ang,

function dif_ang=calc_dif_ang(teta1,teta2)
d_teta=teta1-teta2;
d_ang_1=mod(d_teta,180);
d_ang_2=mod(-d_teta,180);
dif_ang=min(abs([d_ang_1 d_ang_2]));
end