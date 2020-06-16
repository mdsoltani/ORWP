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


function[Theta_elv]=AR1Gaussian(Time,t_feedback,CoherenceTime,mu_theta,sigma_theta)

%mu_theta=29.67;
%sigma_theta=7.78;
%--------------------------------------------------------------------
%AR(1) coefficient:
Ts=10^-3;     % Sampling Time
%CoherenceTime=130*10^-3; % Coherence Time
threshod=0.05;
phi=threshod^(Ts/CoherenceTime);
%--------------------------------------------------------------------
mu=(1-phi)*mu_theta;
zigma=sqrt(1-phi^2)*sigma_theta;
%--------------------------------------------------------------------
xn_1=mu_theta;  % If you put this equal to zero, it needs some time to converge

n_thTerm=floor(Time/Ts);

for i=1:n_thTerm+10
    
    xn(i)=mu+phi*xn_1+zigma*randn;%#ok
    xn_1=xn(i);
    
end

if Time<=t_feedback
    Theta_elv=xn(1);
else
    %Cheching_Time=t_feedback:t_feedback:Time;
    
    %Theta_elv=xn(floor(Cheching_Time/Ts));
    Theta_elv=xn;
end