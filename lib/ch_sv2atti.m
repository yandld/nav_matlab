function [Cb2n] = ch_sv2atti(acc)

acc = acc / norm(acc);

Cb2n(3,:) = acc ;
C3 = Cb2n(3,:);

if Cb2n(3,1) > 0.5
    C2 = [Cb2n(3,2),  -Cb2n(3,1), 0];
else
    C2 = [0, Cb2n(3,3), -Cb2n(3,2)];
end
C2 = C2 / norm(C2);
C1 = cross(C2, C3);

Cb2n = [C1; C2; C3];
end