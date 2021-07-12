% Lucas Ward
% ASEN 5090
% Read the RINEX navigation file.  The header is skipped because
% information in it (a0, a1, iono alpha and beta parameters) is not 
% currently needed for orbit computation.  This can be easily amended to
% include navigation header information by adding lines in the 'while' loop
% where the header is currently skipped.
% 
% Input:        - filename - enter the filename to be read.  If filename
%                            exists, the orbit will be calculated.
% 
% Output:       - ephemeris - Output is a matrix with rows for each PRN and
%                             columns as follows:
% 
%                  col  1:    PRN    ....... satellite PRN          
%                  col  2:    M0     ....... mean anomaly at reference time
%                  col  3:    delta_n  ..... mean motion difference
%                  col  4:    e      ....... eccentricity
%                  col  5:    sqrt(A)  ..... where A is semimajor axis
%                  col  6:    OMEGA  ....... LoAN at weekly epoch
%                  col  7:    i0     ....... inclination at reference time
%                  col  8:    omega  ....... argument of perigee
%                  col  9:    OMEGA_dot  ... rate of right ascension 
%                  col 10:    i_dot  ....... rate of inclination angle
%                  col 11:    Cuc    ....... cosine term, arg. of latitude
%                  col 12:    Cus    ....... sine term, arg. of latitude
%                  col 13:    Crc    ....... cosine term, radius
%                  col 14:    Crs    ....... sine term, radius
%                  col 15:    Cic    ....... cosine term, inclination
%                  col 16:    Cis    ....... sine term, inclination
%                  col 17:    toe    ....... time of ephemeris
%                  col 18:    IODE   ....... Issue of Data Ephemeris
%                  col 19:    GPS_wk ....... GPS week
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ephemeris = read_rinex_nav( filename )

fid = fopen(filename);

if fid == -1
    errordlg(['The file ''' filename ''' does not exist.']);
    return;
end

% skip through header
end_of_header = 0;
while end_of_header == 0
    current_line = fgetl(fid);
    if strfind(current_line,'END OF HEADER')
        end_of_header=1;
    end
end

j = 0;
while feof(fid) ~= 1
    j = j+1;
    
    current_line = fgetl(fid);
    % parse epoch line (ignores SV clock bias, drift, and drift rate)
    [PRN, Y, M, D, H, min, sec,af0,af1,af2] = parsef(current_line, {'I2' 'I3' 'I3' 'I3' 'I3' 'I3' ...
                                                  'F5.1','D19.12','D19.12','D19.12'});

    % Broadcast orbit line 1
    current_line = fgetl(fid);
    [IODE Crs delta_n M0] = parsef(current_line, {'D22.12' 'D19.12' 'D19.12' 'D19.12'});

    % Broadcast orbit line 2
    current_line = fgetl(fid);
    [Cuc e Cus sqrtA] = parsef(current_line, {'D22.12' 'D19.12' 'D19.12' 'D19.12'});

    % Broadcast orbit line 3
    current_line = fgetl(fid);
    [toe Cic OMEGA Cis] = parsef(current_line, {'D22.12' 'D19.12' 'D19.12' 'D19.12' 'D19.12'});

    % Broadcast orbit line 4
    current_line = fgetl(fid);
    [i0 Crc omega OMEGA_dot] = parsef(current_line, {'D22.12' 'D19.12' 'D19.12' 'D19.12' 'D19.12'});

    % Broadcast orbit line 5
    current_line = fgetl(fid);
    [i_dot L2_codes GPS_wk L2_dataflag ] = parsef(current_line, {'D22.12' 'D19.12' 'D19.12' 'D19.12' 'D19.12'});

    % Broadcast orbit line 6
    current_line = fgetl(fid);
    [SV_acc SV_health TGD IODC] = parsef(current_line, {'D22.12' 'D19.12' 'D19.12' 'D19.12' 'D19.12'});

    % Broadcast orbit line 7
    current_line = fgetl(fid);
    [msg_trans_t fit_int ] = parsef(current_line, {'D22.12' 'D19.12' 'D19.12'});
    
    varargin=[Y,M,D,H,min,sec];
    [ gps_week, toc ] = cal2gpstime(varargin);

    ephemeris(j,:) = [PRN, M0, delta_n, e, sqrtA, OMEGA, i0, omega, OMEGA_dot, i_dot, Cuc, Cus, Crc, Crs, Cic, Cis, toe, IODE, GPS_wk,toc,af0,af1,af2,TGD];
end

