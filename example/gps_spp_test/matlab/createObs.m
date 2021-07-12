function [broadcast_obs,precise_obs]=createObs(TOW,satOrbits)

precise_obs = 0;

index=find(satOrbits(1).TOW==TOW);
PRNlist=[];

for ii=1:32
    if satOrbits(ii).C1(index)~=0 && satOrbits(ii).L1(index)~=0 ...
            &&satOrbits(ii).L2(index)~=0 &&satOrbits(ii).P2(index)~=0 
        PRNlist=[PRNlist,ii];
    end
end

broadcast_obs.col.XS=1;
broadcast_obs.col.YS=2;
broadcast_obs.col.ZS=3;
broadcast_obs.col.CorrP=4;
broadcast_obs.col.TOW=5;
broadcast_obs.col.PRN=6;

broadcast_obs.data=zeros(length(PRNlist),5);

for ii=1:length(PRNlist)
    broadcast_obs.data(ii,broadcast_obs.col.XS)=satOrbits(PRNlist(ii)).XS(index);
    broadcast_obs.data(ii,broadcast_obs.col.YS)=satOrbits(PRNlist(ii)).YS(index);
    broadcast_obs.data(ii,broadcast_obs.col.ZS)=satOrbits(PRNlist(ii)).ZS(index);
    broadcast_obs.data(ii,broadcast_obs.col.CorrP)=satOrbits(PRNlist(ii)).CorrP1(index);
    broadcast_obs.data(ii,broadcast_obs.col.PRN)=satOrbits(PRNlist(ii)).PRN;
    broadcast_obs.data(ii,broadcast_obs.col.TOW)=TOW;    
end

