function sqrt_sum = sqrt_sum(mat_in)
[r,c] =  size(mat_in);
sqrt_sum = zeros(r,1);
sum = 0;
for i = 1:r
    for j =1: c
        sum = sum + mat_in(i,j)*mat_in(i,j);
    end
    sqrt_sum(i) = sqrt(sum);
    sum = 0;
end


