clear;
clc;
close all;

% 打开文件
fid = fopen('230831094010.json', 'r');

% 初始化cell数组来存储解析的JSON
jsonDataCell = {};

% 逐行读取并解析
tline = fgetl(fid);
while ischar(tline)
    currentJson = jsondecode(tline);
    
    % 检查当前JSON对象是否包含"type"字段
    if isfield(currentJson, 'type')
        jsonDataCell{end+1} = currentJson;
    end
    
    tline = fgetl(fid);
end

% 关闭文件
fclose(fid);



% 初始化结果结构
resultStruct = struct;

% 遍历解析的JSON数据
for i = 1:length(jsonDataCell)
    currentType = jsonDataCell{i}.type;
    
    % 如果当前type不存在于结果结构中，初始化它
    if ~isfield(resultStruct, currentType)
        resultStruct.(currentType) = struct;
        resultStruct.(currentType).header = jsonDataCell{i}.header;
        resultStruct.(currentType).cnt = [];
        resultStruct.(currentType).data = [];
    end
    
    % 添加当前cnt和data到结果结构中
    resultStruct.(currentType).cnt(end+1) = jsonDataCell{i}.cnt;
    resultStruct.(currentType).data(end+1, :) = jsonDataCell{i}.data;
end



