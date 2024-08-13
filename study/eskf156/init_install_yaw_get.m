function bCh = init_install_yaw_get(Vgnss,imu,init_align_angle)
%% input move data :open sky,straight accel
%% Vgnss:tow,gnss_ve,gnss_vn,gnss_vu
%% init_align_angle:vect3= [p,r,0]--rad
bCh = att2Cnb(init_align_angle);%% from v to b no yaw as h frame
len = length(Vgnss);
Vgnss_hori = zeros(len,1);
j=1;
for i = 1:length(Vgnss)
    Vgnss_hori(i) = sqrt(Vgnss(i,2:3));%%
    gnss_ts(i) = Vgnss(i,1:2);
    if i>1
        accel_v(j,:)= (Vgnss_hori(i) - Vgnss_hori(i-1))/(gnss_ts(i)-gnss_ts(i-1));
        yaw_cvn = atan2(Vgnss(i,2),Vgnss(i,2));
    end
end



