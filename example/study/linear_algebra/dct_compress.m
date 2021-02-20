% dct_compress.m

% 作用：使用离散余弦变换对输入的 JPG 图片进行压缩
function [z, k] = dct_compress(image, ratio)

    % 对原 JPG 图片在三个颜色上分别做二维离散余弦变换
    z(:,:,1) = dct2(image(:,:,1));
    z(:,:,2) = dct2(image(:,:,2));
    z(:,:,3) = dct2(image(:,:,3));

    % 获取图片的尺寸大小
    [a, b, ~] = size(image);

    % 低通滤波
    for i = 1 : a
        for j = 1 : b
            if (i + j > (a+b) * ratio)
                z(i, j, 1) = 0;
                z(i, j, 2) = 0;
                z(i, j, 3) = 0;
            end
        end
    end

    % 对过滤之后的结果在三个颜色上分别做进行二维反离散余弦变换
    k(:,:,1) = idct2(z(:,:,1));
    k(:,:,2) = idct2(z(:,:,2));
    k(:,:,3) = idct2(z(:,:,3));

    % 类型转换，转换为 0-255 范围内的颜色值
    k = uint8(k);
end