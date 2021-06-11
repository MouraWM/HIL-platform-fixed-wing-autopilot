%US Standard Atmosphere in SI units
%Input : h altitute (m)
%Output : T temperature (K), p pressure (N/mˆ2), rho density (kg/mˆ3)

function [rho,T,p]=ISA(h)
    h=h/1000; % m -> km
    h1=11; h2=20; h3=32; a0=-6.5e-3; a2=1e-3; g=9.80665; mol=28.9644;
    R0=8.31432; R=R0/mol*1e3;
    T0=288.15; p0=1.01325e5; rho0=1.2250; T1=T0+a0*h1*1e3;
    p1=p0*(T1/T0)^(-g/a0/R); rho1=rho0*(T1/T0)^(-g/a0/R-1); T2=T1;
    p2=p1*exp(-g/R/T2*(h2-h1)*1e3); rho2=rho1*exp(-g/R/T2*(h2-h1)*1e3);
    if h <= h1
        %disp('troposphere');
        T=T0+a0*h*1e3;
        p=p0*(T/T0)^(-g/a0/R);
        rho=rho0*(T/T0)^(-g/a0/R-1);
    elseif h <= h2
        %disp('low stratosphere');
        T=T1;
        p=p1*exp(-g/R/T*(h-h1)*1e3);
        rho=rho1*exp(-g/R/T*(h-h1)*1e3);
    elseif h <= h3
        %disp('stratosphere');
        T=T2+a2*(h-h2)*1e3;
        p=p2*(T/T2)^(-g/a2/R);
        rho=rho2*(T/T2)^(-g/a2/R-1);
    end
end