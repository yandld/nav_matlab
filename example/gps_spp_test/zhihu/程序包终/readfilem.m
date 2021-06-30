function [P0,T,e0]=readfilem(file,hour,minute,second)
%[P0,T,e0]从m文件中读取对应时刻的气压（mbar）、温度（K）、湿度（%）
if mod(minute,5)>2
    minute=minute-mod(minute,5)+5;
elseif mod(minute,5)==2
       if second>30
           minute=minute-mod(minute,5)+5;
       else
           minute=minute-mod(minute,5);
       end
elseif mod(minute,5)<=2
       minute=minute-mod(minute,5);
end
if minute==60
   hour=hour+1;
   minute=0;
end
data=importdata(file,'\n');
for i=1:length(data(:,:))
    if eval(data{i,:}(11:12))==hour&&eval(data{i,:}(14:15))==minute
        P0=eval(data{i,:}(20:25));
        T=273.16+eval(data{i,:}(29:32));
        e0=eval(data{i,:}(36:37));
    end
end