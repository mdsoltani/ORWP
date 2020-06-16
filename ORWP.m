
% This Code calculates the Handover Rate of ORWP with pause time

% Papers:
%   [1] M. D. Soltani, A. A. Purwita, Z. Zeng, C. Chen, H. Haas, and M. Safari,
%     “ An Orientation-based Random Waypoint Model for User Mobility in Wireless Networks” 
%     IEEE International Conference on Communications, June 2020. 

%   [2] M. D. Soltani, A. A. Purwita, Z. Zeng, H. Haas, and M. Safari,
%      “Modeling the Random Orientation of Mobile Devices:  Measurement, Analysis and LiFi Use Case,”
%      IEEE Transactions on Communications, vol. 67, no. 3, pp. 2157-2172, March 2019.

%   [3] M. D. Soltani, M. A. Arfaoui, I. Tavakkolnia, A. Ghrayeb, M. Safari, C. Assi, M. Hasna, H. Haas, 
%       “Bidirectional Optical Spatial Modulation for Mobile Users: Towards a Practical Design for LiFi Systems,” 
%        IEEE Journal on Selected Area in Communications, vol. 37, no. 9, pp. 2069–2086, Sep. 2019.


% If you use the code, please make sure that you cite the references [1], [2] and [3].

%  This code is written by Mohammad Dehghani Soltani
%  The University of Edinburgh
%  msoltan@ed.ac.uk

% All rights are reserved


clc;
clear;
%--------------------------------------------------------------------------
theta = 60; % semi-angle at half power
m=-log10(2)/log10(cosd(theta)); %Lambertian order of emission
Adet=1e-4; %detector physical area of a PD
gf=1; %gain of an optical filter; ignore if no filter is used
index=1; %refractive index of a lens at a PD; ignore if no lens is used
FOV=90; %FOV of a receiver
G_Con=(index^2)/(sind(FOV).^2); %gain of an optical concentrator; ignore if no lens is used
h=2.15;
%--------------------------------------------------------------------------

% Rcell=4   %#ok 
Lroom=10  %#ok 4*Rcell/sqrt(2);

TP1=[Lroom/4 Lroom/4 h];
TP2=[Lroom/4 -Lroom/4 h];
TP3=[-Lroom/4 Lroom/4 h];
TP4=[-Lroom/4 -Lroom/4 h];

x=-Lroom/2:0.01:Lroom/2;
y=-Lroom/2:0.01:Lroom/2;

Area=combvec(x,y);

%--------------------------------------------------------------------------
% Pause Time Parameters
CoherenceTime_alpha_pause=342*10^-3;               
sigma_alpha_pause=3.67;

CoherenceTime_beta_pause=377*10^-3;
sigma_beta_pause=2.39;            

CoherenceTime_gamma_pause=331*10^-3;
mu_gamma_pause=-0.84;               
sigma_gamma_pause=2.21;       

CoherenceTime_pause=min([CoherenceTime_alpha_pause,CoherenceTime_beta_pause,CoherenceTime_gamma_pause]);
%--------------------------------------------------------------------------
% Walking Parameters
CoherenceTime_alpha_walk=131*10^-3;               
sigma_alpha_walk=10;

CoherenceTime_beta_walk=176*10^-3;
mu_beta_walk=28.81; 
sigma_beta_walk=3.26;            

CoherenceTime_gamma_walk=142*10^-3;
mu_gamma_walk=-1.35;               
sigma_gamma_walk=5.42;

CoherenceTime=min([CoherenceTime_alpha_walk,CoherenceTime_beta_walk,CoherenceTime_gamma_walk]);
%--------------------------------------------------------------------------
% beta Parameters
beta_min=30;
beta_max=30;

%--------------------------------------------------------------------------

Omega=0;  % Direction angle
t=1;

%Initialization for waypoint
waypoint_index=randi(length(Area));
waypoint=Area(:,waypoint_index);
x1(1)=waypoint(1);
y1(1)=waypoint(2);
RP=[x1(1) y1(1) 0];

N_iter=10000;
alpha=Omega-90;beta=0;gamma=0;
[index(1)]=APSlction(alpha,beta,gamma,TP1,TP2,TP3,TP4,RP,m,Adet,h,gf,G_Con,FOV);
velocity=1 %#ok %constant velocity
%-------------------------------------------
t_feedback=0.001; %Feedback time to check wehther handover should be done or not

%--------------------------------------------------------------------------
T_pause_mean=10 %#ok  % mean of pause time in second


alpha_RP=[];

for t=1:N_iter
    
    waypoint_index=randi(length(Area));
    waypoint=Area(:,waypoint_index);
    x1(t+1)=waypoint(1); %#ok
    y1(t+1)=waypoint(2); %#ok
    
    RP=[x1(t+1) y1(t+1) 0];
    transition_length(t)=sqrt(dot([x1(t+1), y1(t+1)]-[x1(t), y1(t)],[x1(t+1), y1(t+1)]-[x1(t), y1(t)])); %#ok
    
    Time(t)=transition_length(t)/velocity; %#ok
    
    Omega(t)=atan2d((y1(t+1)-y1(t)),(x1(t+1)-x1(t))); %#ok
           
    if Time(t)<=t_feedback
        
        %theta_elv=AR1Gaussian(Time(t),t_feedback,CoherenceTime,mu_theta,sigma_theta);
        
        mu_alpha=Omega(t)-90;
        alpha=AR1Gaussian(Time(t),t_feedback,CoherenceTime_alpha_walk,mu_alpha,sigma_alpha_walk);
        beta=AR1Gaussian(Time(t),t_feedback,CoherenceTime_beta_walk,mu_beta_walk,sigma_beta_walk);
        gamma=AR1Gaussian(Time(t),t_feedback,CoherenceTime_gamma_walk,mu_gamma_walk,sigma_gamma_walk);
        
        [index(t+1)]=APSlction(alpha,beta,gamma,TP1,TP2,TP3,TP4,RP,m,Adet,h,gf,G_Con,FOV); %#ok
        
        
        %% Calculate SNR or whatever you want
        
        
    else
        index_temp=[];
        RP_temp=[x1(t) y1(t) 0];
        %theta_elv=AR1Gaussian(Time(t),t_feedback,CoherenceTime,mu_theta,sigma_theta);
        mu_alpha=Omega(t)-90;
        alpha=AR1Gaussian(Time(t),t_feedback,CoherenceTime_alpha_walk,mu_alpha,sigma_alpha_walk);
        beta=AR1Gaussian(Time(t),t_feedback,CoherenceTime_beta_walk,mu_beta_walk,sigma_beta_walk);
        gamma=AR1Gaussian(Time(t),t_feedback,CoherenceTime_gamma_walk,mu_gamma_walk,sigma_gamma_walk);
        
        
        
        index_temp(1)=APSlction(alpha(1),beta(1),gamma(1),TP1,TP2,TP3,TP4,RP_temp,m,Adet,h,gf,G_Con,FOV);
%         checking_Time=0:CoherenceTime:Time(t);
        checking_Time=0:10*t_feedback:Time(t);
        for a1=1:length(checking_Time)
            x1_temp=x1(t)+velocity*cosd(Omega(t))*checking_Time(a1);
            y1_temp=y1(t)+velocity*sind(Omega(t))*checking_Time(a1);
            RP_temp=[x1_temp y1_temp 0];
            [index_temp(a1+1)]=APSlction(alpha(a1),beta(a1),gamma(a1),TP1,TP2,TP3,TP4,RP_temp,m,Adet,h,gf,G_Con,FOV); %#ok
            
            
            %% Calculate SNR or whatever you want
            
            
        end
    end
    
    %pause time
    
    T_pause(t)=exprnd(T_pause_mean); %#ok % random pause time
    
    mu_alpha=Omega(t)-90;
    alpha=CorrLaplace(CoherenceTime_alpha_pause,mu_alpha,sigma_alpha_pause,T_pause(t));
    
    T_mean=10;  % mean time that beta is stationary
    sumT_Tbeta=0;
    beta=[];
    while (sumT_Tbeta<T_pause(t))
    T_beta=exprnd(T_mean); % random pause time
    mu_beta=rand*(beta_max-beta_min)+beta_min;
    beta1=CorrLaplace(CoherenceTime_beta_pause,mu_beta,sigma_beta_pause,T_beta);
    beta=[beta,beta1]; %#ok
    sumT_Tbeta=sumT_Tbeta+T_beta;
    end
    beta=beta(1:length(alpha));
    
    gamma=CorrLaplace(CoherenceTime_gamma_pause,mu_gamma_pause,sigma_gamma_pause,T_pause(t));
    
    index_last=index_temp(end);
    index_temp=[];
    index_temp(1)=index_last;
%     checking_Time=0:CoherenceTime_pause:T_pause(t);
    checking_Time=0:10*t_feedback:T_pause(t);
    for a2=1:length(checking_Time)
        [index_temp(a2+1)]=APSlction(alpha(a2),beta(a2),gamma(a2),TP1,TP2,TP3,TP4,RP_temp,m,Adet,h,gf,G_Con,FOV); %#ok
        
        %% Calculate SNR or whatever you want
        
    end
    
end



