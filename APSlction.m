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


function[index]=APSlction(alpha,beta,gamma,TP1,TP2,TP3,TP4,RP,ml,Adet,h,gf,G_Con,FOV)

normalvect=[sind(alpha)*sind(beta)*cosd(gamma)+cosd(alpha)*sind(gamma);...
            sind(alpha)*sind(gamma)-cosd(alpha)*sind(beta)*cosd(gamma);cosd(beta)*cosd(gamma)];  %Normal vector of UE 

D1=sqrt(dot(TP1-RP,TP1-RP));
D2=sqrt(dot(TP2-RP,TP2-RP));
D3=sqrt(dot(TP3-RP,TP3-RP));
D4=sqrt(dot(TP4-RP,TP4-RP));


cosphi_A1=dot(TP1-RP,normalvect)/D1;
cosphi_A1(cosphi_A1<cosd(FOV))=0;
MatH_LoS1=(ml+1)*Adet.*cosphi_A1.*(h/D1).^(ml)./(2*pi.*D1.^2)*gf.*G_Con;


cosphi_A2=dot(TP2-RP,normalvect)/D2;
cosphi_A2(cosphi_A2<cosd(FOV))=0;
MatH_LoS2=(ml+1)*Adet.*cosphi_A2.*(h/D2).^(ml)./(2*pi.*D2.^2)*gf.*G_Con;


cosphi_A3=dot(TP3-RP,normalvect)/D3;
cosphi_A3(cosphi_A3<cosd(FOV))=0;
MatH_LoS3=(ml+1)*Adet.*cosphi_A3.*(h/D3).^(ml)./(2*pi.*D3.^2)*gf.*G_Con;


cosphi_A4=dot(TP4-RP,normalvect)/D4;
cosphi_A4(cosphi_A4<cosd(FOV))=0;
MatH_LoS4=(ml+1)*Adet.*cosphi_A4.*(h/D4).^(ml)./(2*pi.*D4.^2)*gf.*G_Con;

MatH_LoS=[MatH_LoS1,MatH_LoS2,MatH_LoS3,MatH_LoS4];

[HLoS_val,AP_ind]=max(MatH_LoS);

if (HLoS_val~=0)
    index=AP_ind;
else
    index=0;
end

end
