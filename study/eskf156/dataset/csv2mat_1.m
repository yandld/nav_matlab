function data = csv2mat_1(fullfilename)

    close all;
    % 获取脚本所在的完整路径
    scriptPath = mfilename('fullpath');
    
    % 获取脚本所在的目录路径
    scriptFolder = fileparts(scriptPath);
    
    % 切换当前工作目录到脚本所在的目录
    cd(scriptFolder);

    [pathstr,file_name,ext]=fileparts(fullfilename)
    rawData = csvread(fullfilename, 3, 2);
    
    %获得帧数、项目数
    [numRows, numCols] =size(rawData);
    N =numRows;
    frameNum =floor((N-1)/3);
    
    % 数据空间
    inspvaxbData =zeros(frameNum, numCols);     %inspvaxb数据
    data.ins.time =zeros(frameNum, 2);
    data.ins.att =zeros(frameNum, 3);
    data.ins.vel =zeros(frameNum, 3);
    data.ins.lla =zeros(frameNum, 3);
    data.ins.att_std =zeros(frameNum, 3);
    data.ins.vel_std =zeros(frameNum, 3);
    data.ins.lla_std =zeros(frameNum, 3);
    data.ins.insStatus =0;
    data.ins.posType =0;
    data.ins.evtBit =0;
    rawimuxbData =zeros(frameNum, numCols);     %rawimuxbData数据
    data.imu.time =zeros(frameNum, 2);
    data.imu.gyr =zeros(frameNum, 3);
    data.imu.acc =zeros(frameNum, 3);
    gnssrcvData =zeros(frameNum, numCols);      %gnssrcvData数据
    data.gnss.time =zeros(frameNum, 2);
    data.gnss.lla =zeros(frameNum, 3);
    data.gnss.vel =zeros(frameNum, 3);
    data.gnss.att =zeros(frameNum, 3);
    data.gnss.lla_std =zeros(frameNum, 3);
    data.gnss.vel_std =zeros(frameNum, 3);
    data.gnss.att_std =zeros(frameNum, 3);
    data.gnss.od =0;
    data.gnss.nv =0;
    data.gnss.nv_heading =0;
    data.gnss.solq =0;
    data.gnss.solq_heading =0;
    data.gnss.diff_age =0;
    data.gnss.wb =zeros(frameNum, 3);
    data.gnss.gb =zeros(frameNum, 3);
    % 抽取数据
    for i =0:frameNum-1
        inspvaxbData(i+1, :) = rawData(1+i*3, :);
        rawimuxbData(i+1, :) = rawData(2+i*3, :);
        gnssrcvData(i+1, :) = rawData(3+i*3, :);
    end
    
    % 画帧间隔
    time = inspvaxbData(:, 2);
    diff_time = diff(time);
    plot(diff_time, '.-');
    %细分数据
    data.ins.time =inspvaxbData(: ,1:2);
    data.ins.att =inspvaxbData(: ,3:5);
    data.ins.vel =inspvaxbData(: ,6:8);
    data.ins.lla =inspvaxbData(: ,9:11);
    data.ins.att_std =inspvaxbData(: ,12:14);
    data.ins.vel_std =inspvaxbData(: ,15:17);
    data.ins.lla_std =inspvaxbData(: ,18:20);
    data.ins.insStatus =inspvaxbData(: ,21);
    data.ins.posType =inspvaxbData(: ,22);
    data.ins.evtBit =inspvaxbData(: ,23);
        
    data.imu.time =rawimuxbData(: ,1:2);
    data.imu.gyr =rawimuxbData(: ,3:5);
    data.imu.acc =rawimuxbData(: ,6:8);
        
    data.gnss.time =gnssrcvData(: ,1:2);
    data.gnss.lla =gnssrcvData(: ,3:5);
    data.gnss.vel =gnssrcvData(: ,6:8);
    data.gnss.att =gnssrcvData(: ,9:11);
    data.gnss.lla_std =gnssrcvData(: ,12:14);
    data.gnss.vel_std =gnssrcvData(: ,15:17);
    data.gnss.att_std =gnssrcvData(: ,18:20);
    data.gnss.od =gnssrcvData(: ,21);
    data.gnss.nv =gnssrcvData(: ,22);
    data.gnss.nv_heading =gnssrcvData(: ,23);
    data.gnss.solq =gnssrcvData(: ,24);
    data.gnss.solq_heading =gnssrcvData(: ,25);
    data.gnss.diff_age =gnssrcvData(: ,26);
    data.gnss.wb =gnssrcvData(: ,27:29);
    data.gnss.gb =gnssrcvData(: ,30:32);
    %保存数据
    fprintf("保存数据...\r\n");
    fprintf("保存位置%s/%s\r\n", scriptFolder, fullfile(file_name + ".mat"));
    save(fullfile(file_name + ".mat"), 'data');
    fprintf("保存完成\r\n");
    

end

