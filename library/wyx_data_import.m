%% 专门用于导入 MTI201506 的数据
function [Accelerometer  Gyroscope Magnetometer EularAngle time] = wyx_data_import(path )

fidin=fopen(path);     
fidout=fopen('mkmatlab.txt','w');                       % 创建MKMATLAB.txt文件 
while ~feof(fidin)                                      % 判断是否为文件末尾               
    tline=fgetl(fidin);                                 % 从文件读行   
    if double(tline(1))>=48&&double(tline(1))<=57       % 判断首字符是否是数值 
       fprintf(fidout,'%s\n\n',tline);                  % 如果是数字行，把此行数据写入文件MKMATLAB.txt 
       continue                                         % 如果是非数字继续下一次循环 
    end 
end 
fclose(fidout); 
MK=importdata('MKMATLAB.txt');      % 将生成的MKMATLAB.txt文件导入工作空间，变量名为MK，实际上它不显示出来 

Accelerometer = MK(:,3:5);
Gyroscope = MK(:,6:8);
Magnetometer = MK(:,9:11);
EularAngle = MK(:,15:17);
time = 1:length(Accelerometer);
time = time / 100;

end