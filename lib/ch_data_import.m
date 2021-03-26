function dataset = ch_data_import(path)

tbl = readtable(path);

if sum(ismember(tbl.Properties.VariableNames,'AccX'))
    dataset.imu.acc(1,:) = tbl.AccX';
end

if sum(ismember(tbl.Properties.VariableNames,'AccY'))
    dataset.imu.acc(2,:) = tbl.AccY';
end

if sum(ismember(tbl.Properties.VariableNames,'AccZ'))
    dataset.imu.acc(3,:) = tbl.AccZ';
end

if sum(ismember(tbl.Properties.VariableNames,'GyrX'))
    dataset.imu.gyr(1,:) = tbl.GyrX';
end

if sum(ismember(tbl.Properties.VariableNames,'GyrY'))
    dataset.imu.gyr(2,:) = tbl.GyrY';
end

if sum(ismember(tbl.Properties.VariableNames,'GyrZ'))
    dataset.imu.gyr(3,:) = tbl.GyrZ';
end

if sum(ismember(tbl.Properties.VariableNames,'MagX'))
    dataset.imu.mag(1,:) = tbl.MagX';
end

if sum(ismember(tbl.Properties.VariableNames,'MagY'))
    dataset.imu.mag(2,:) = tbl.MagY';
end

if sum(ismember(tbl.Properties.VariableNames,'MagZ'))
    dataset.imu.mag(3,:) = tbl.MagZ';
end


if sum(ismember(tbl.Properties.VariableNames,'Roll'))
    dataset.eul.roll = tbl.Roll';
end

if sum(ismember(tbl.Properties.VariableNames,'Pitch'))
    dataset.eul.pitch = tbl.Pitch';
end

if sum(ismember(tbl.Properties.VariableNames,'Yaw'))
    dataset.eul.yaw = tbl.Yaw';
end

