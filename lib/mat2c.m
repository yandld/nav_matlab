function mat2c(mat)
    [r, c] = size(mat);
    mn = r*c;

    fprintf('{');
    if r==1||c==1
        for i=1:mn-1
            fprintf('%f,', mat(i));
        end
        fprintf('%f};\n', mat(i+1));
    else
        for i=1:r-1
            for j=1:c
                fprintf('%f,', mat(i,j));
            end
            fprintf('\n');
        end
        for j=1:c-1
            fprintf('%f,', mat(i+1,j));
        end
        fprintf('%f};\n', mat(i+1,j+1));
    end
end
