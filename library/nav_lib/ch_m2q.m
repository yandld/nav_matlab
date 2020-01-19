function qnb = ch_m2q(Cnb)
    C11 = Cnb(1,1); C12 = Cnb(1,2); C13 = Cnb(1,3); 
    C21 = Cnb(2,1); C22 = Cnb(2,2); C23 = Cnb(2,3); 
    C31 = Cnb(3,1); C32 = Cnb(3,2); C33 = Cnb(3,3); 
    if C11>=C22+C33
        q1 = 0.5*sqrt(1+C11-C22-C33);
        q0 = (C32-C23)/(4*q1); q2 = (C12+C21)/(4*q1); q3 = (C13+C31)/(4*q1);
    elseif C22>=C11+C33
        q2 = 0.5*sqrt(1-C11+C22-C33);
        q0 = (C13-C31)/(4*q2); q1 = (C12+C21)/(4*q2); q3 = (C23+C32)/(4*q2);
    elseif C33>=C11+C22
        q3 = 0.5*sqrt(1-C11-C22+C33);
        q0 = (C21-C12)/(4*q3); q1 = (C13+C31)/(4*q3); q2 = (C23+C32)/(4*q3);
    else
        q0 = 0.5*sqrt(1+C11+C22+C33);
        q1 = (C32-C23)/(4*q0); q2 = (C13-C31)/(4*q0); q3 = (C21-C12)/(4*q0);
    end
    qnb = [q0; q1; q2; q3];
    
