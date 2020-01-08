function qnb = ch_m2q(R)
    R = R';
    C11 = R(1,1); C12 = R(1,2); C13 = R(1,3); 
    C21 = R(2,1); C22 = R(2,2); C23 = R(2,3); 
    C31 = R(3,1); C32 = R(3,2); C33 = R(3,3); 
    if C11>=C22+C33
        q1 = 0.5*sqrt(1+C11-C22-C33);  qq4 = 4*q1;
        q0 = (C32-C23)/qq4; q2 = (C12+C21)/qq4; q3 = (C13+C31)/qq4;
    elseif C22>=C11+C33
        q2 = 0.5*sqrt(1-C11+C22-C33);  qq4 = 4*q2;
        q0 = (C13-C31)/qq4; q1 = (C12+C21)/qq4; q3 = (C23+C32)/qq4;
    elseif C33>=C11+C22
        q3 = 0.5*sqrt(1-C11-C22+C33);  qq4 = 4*q3;
        q0 = (C21-C12)/qq4; q1 = (C13+C31)/qq4; q2 = (C23+C32)/qq4;
    else
        q0 = 0.5*sqrt(1+C11+C22+C33);  qq4 = 4*q0;
        q1 = (C32-C23)/qq4; q2 = (C13-C31)/qq4; q3 = (C21-C12)/qq4;
    end
    qnb = [q0; q1; q2; q3];
    
