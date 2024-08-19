function bCh = init_install_yaw_get(Vgnss,imu,init_align_angle)
%% input move data :open sky,straight accel
%% Vgnss:tow,gnss_ve,gnss_vn,gnss_vu
%% init_align_angle:vect3= [p,r,0]--rad
bCh = att2Cnb(init_align_angle);%% from v to b no yaw as h frame
len = length(Vgnss);%Vgnss =[ve,vn,vu]
Vgnss_hori = zeros(len,1);
fbibx = mean(imu.acc(:,1));
fbiby = mean(imu.acc(:,2));
fbibz = mean(imu.acc(:,3));
fbib = [fbibx,fbiby,fbibz]';
fhib = bCh * fbib;
fhibx = fhib(1);
fhiby = fhib(2);
fhibz = fhib(3);

j=1;
for i = 1:length(Vgnss)
    Vgnss_hori(i) = sqrt(Vgnss(i,2:3));%%
    gnss_ts(i) = Vgnss(i,1);
    if i>1
        accel_v(j,:)= (Vgnss_hori(i,2:3) - Vgnss_hori(i-1,2:3))/(gnss_ts(i)-gnss_ts(i-1));

        yaw_cvn = atan2(Vgnss(i,2),Vgnss(i,2));
        if(yaw_cvn < 0)
           yaw_cvn =  360 + yaw_cvn;
        end
        molecule =  -accel_v(j,1)*fhiby + accel_v(j,2)*fhibx;
        denominator =  fhiby*fhiby + fhibx*fhibx;
        sinpHI_yawbn = sin(molecule/denominator);
        cospHI_yawbn = cos(molecule/denominator);
        pHI_yawbn = atan2(sinpHI_yawbn,cospHI_yawbn);
        if(pHI_yawbn < 0)
           pHI_yawbn =  360 + pHI_yawbn;
        end
        phi_bv(j) = pHI_yawbn - yaw_cvn;
        j = j+1;
    end
end



