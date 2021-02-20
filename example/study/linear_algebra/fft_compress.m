% fft_compress.m

% 作用：使用离散傅里叶变换对输入的 JPG 图片 image 按照指定的滤波比例 ratio 进行压缩
% 返回值：返回低通滤波之后的频率分布和压缩之后的图片
function [z, k] = fft_compress(image, ratio)

    % 对原 JPG 图片在三个颜色上分别做二维离散傅里叶变换，得到频率分布
    z(:,:,1) = fft2(image(:,:,1));
    z(:,:,2) = fft2(image(:,:,2));
    z(:,:,3) = fft2(image(:,:,3));

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

    % 对过滤之后的结果在三个颜色上分别做进行二维反离散傅里叶变换
    k(:,:,1) = ifft2(z(:,:,1));
    k(:,:,2) = ifft2(z(:,:,2));
    k(:,:,3) = ifft2(z(:,:,3));

    % 类型转换，转换为 0-255 范围内的颜色值
    k = uint8(k);
end