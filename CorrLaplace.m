% Correlated Laplace


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


function[x]=CorrLaplace(Tc,mu,sigma,Time)

Ts=0.001;
% Tc=0.377;
% mu=20;
% sigma=2.39;

c_1=0.05^(Ts/Tc/2);
c_0=0; 
sigma=sqrt((c_0^2*(c_1+1)-sqrt((c_1+1)^2*(2*sigma^2*(c_1-1)^4+c_0^4)))/(c_1-1));

x1=sqrt(mu);
x2=sqrt(mu);

n_thTerm=floor(Time/Ts);
for i=1:n_thTerm
   
    x1(i+1)=c_0+c_1*x1(i)+sigma/sqrt(2)*(randn+1i*randn);
    
    x2(i+1)=c_0+c_1*x2(i)+sigma/sqrt(2)*(randn+1i*randn);

    
end


x=real(x1.*x2)+mu;
%figure;plot(x);
% figure;histogram(x,'Normalization','pdf');
% figure;autocorr(x,500);

end