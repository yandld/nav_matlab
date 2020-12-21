function [Cb2n] = ch_dv2atti(vn1, vn2, vb1, vb2)

    vntmp1 = cross(vn1,vn2); vntmp2 = cross(vntmp1,vn1);
    vbtmp1 = cross(vb1,vb2); vbtmp2 = cross(vbtmp1,vb1);
    Cb2n = [vn1/norm(vn1), vntmp1/norm(vntmp1), vntmp2/norm(vntmp2)]*...
          [vb1/norm(vb1), vbtmp1/norm(vbtmp1), vbtmp2/norm(vbtmp2)]';

      