Version 2.0(2013/05/10): I rewrote the code to use function handle as input parameter instead of symbolic expression as in the previous edition. The old one is in the file -Extended_KF_v1.m.

This toolkit aims at giving an implementation of the Extended Kalman Filter (EKF), as in the file:
	-Extended_KF.m

An example of using the EKF function is given in the file:
	-GPS_EKF.m
which applies Extended_KF to the GPS positioning task. It reads the pseudorange and the satellite position information provided in the 
	-SV_Rho.mat and SV_Pos.mat
and uses the EKF as well as the traditional Least Square(LS) method to compute the position of the receiver. Other .m files are:
	-Rcv_Pos_Compute.m, G_Compute.m, Delta_Rho_Compute.m and
	-PseudorangeEquation.m, ConstantVelocity.m
which contain auxiliary functions for the LS and EKF methods, respectively.

References for the general Kalman Filtering and KF with application to target tracking:
[1] Y. Bar-Shalom, X. R. Li and T. Kirubarajan, Estimation with Applications to Tracking and Navigation, Wiley Interscience, 2001.
[2] R. G. Brown and P. Y. C. Gwang, Introduction to Random Signals and
Applied Kalman Filtering, 3rd ed. New York: Wiley, 1997.
[3] http://www.cs.unc.edu/~welch/kalman/

chong.you1987@gmail.com
