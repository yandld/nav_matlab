function [time, dt, sats, eof] = fepoch_0(fid)
% FEPOCH_0   Finds the next epoch in an opened RINEX file with
%	          identification fid. From the epoch line is produced
%	          time (in seconds of week), number of sv.s, and a mark
%	          about end of file. Only observations with epoch flag 0
%	          are delt with.

%Kai Borre 09-14-96; revised 03-22-97; revised Sept 4, 2001
%Copyright (c) by Kai Borre
%$Revision: 1.0 $  $Date: 1997/09/22  $
%fide = fopen(fid,'rt');

%　该函数只返回ｏ文件中一个历元的以下参数：

% time：周内秒
% dt：接收机钟差（该参数为可选项，不一定所有的o文件中都有）
% sats：当前历元所观测到的卫星
% eof：是否到文件末尾

global sat_index;
time = 0;
dt = 0;
sats = [];
NoSv = 0;
eof = 0;

%循环读取o文件的每一行
while 1
   lin = fgets(fid); % earlier fgetl  先读一行
   
   if (feof(fid) == 1); % 若到文件末尾，则结束
      eof = 1;
      break
   end;
   
   % 跳过空行
   if length(lin) <= 1
       continue;
   end
   
%    answer = findstr(lin,'COMMENT'); % 判断该行中是否有字符串“COMMENT”
%    
%    if ~isempty(answer);  % 若有“COMMENT“，则继续读下一行
%       lin = fgetl(fid);
%    end;
   
   % 如果该行数据第29个字符不为0（0代表改历元正常），
   % 或者总长度只有29（也就是没有后面的卫星PRN数据），
   % 则结束。
   % if ((strcmp(lin(29),'0') == 0) & (size(deblank(lin),2) == 29)) 
   %    eof = 1; 
   %    break
   % end; % We only want type 0 data
   
   % 如果该行第二个字符是1，或者第29个字符是0，则说明
   % 这一行是某一历元的开始行，接下来就可以从该行中提取时间、PRN等参数
   if ((strcmp(lin(2),'1') == 1)  &  (strcmp(lin(29),'0') == 1))
      ll = length(lin)-2;
      if ll > 60, ll = 60; end;
      linp = lin(1:ll);        
      %fprintf('%60s\n',linp);
      
      %使用strtok函数获得间隔符前面的字符串，
      % 首先是获得当前时间
      [nian, lin] = strtok(lin);
      % year;
      
      [month, lin] = strtok(lin);
      % month;
      
      [day, lin] = strtok(lin);
      % day;
      
      [hour, lin] = strtok(lin);
      % hour
      
      [minute, lin] = strtok(lin);
      % minute
      
      [second, lin] = strtok(lin);
      % second
      
      [OK_flag, lin] = strtok(lin); 
      % OK_flag就是第29个字符，如果是0，则该历元正常
      
      %将时间转成数值型，然后再计算出GPS周和周内秒
      h = str2num(hour)+str2num(minute)/60+str2num(second)/3600;
      jd = julday(str2num(nian)+2000, str2num(month), str2num(day), h);
      [week, sec_of_week] = gps_time(jd);
      time = sec_of_week;
      
      %获得该历元卫星数
      [NoSv, lin] = strtok(lin,'G');
      
      %储存该历元每颗卫星的PRN
      for k = 1:str2num(NoSv)
         [sat, lin] = strtok(lin,'G');
         prn(k) = str2num(sat);
      end
      
      % prn是1行NoSv列的矩阵，sats是其转置
      sats = prn(:);
      
      % 接收及钟差，有的o文件中没有该参数
      dT = strtok(lin);
      if isempty(dT) == 0 %如果dT不为0，则将其记录
         dt = str2num(dT);
      end
      
      break % 跳出while循环
      
   end
   
end; 

% datee=[str2num(nian) str2num(month) str2num(day) str2num(hour) str2num(minute) str2num(second)];

%%%%%%%% end fepoch_0.m %%%%%%%%%
