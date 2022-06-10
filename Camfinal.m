%Code for Analytically synthesising Cam Profile

%Initialising parameters based on given information
thetai = 180;
thetad = 0;
thetar = 180;
Rp = 50;
Rr = 10;
Rb = Rp - Rr;
L = 40;

%Creating matrices
%Each index holds the value of the variable for a particular angle
%The angles are in increments of 0.5, starting from 0 to 360

i=1;    %index of every array

for theta=0:0.5:360     %First, fill the values for follower motion as a function of theta     
    
    x(i) = theta;
    
    %y stands for lift / displacement
    %y1 stands for dy/d(theta)
    %y2 stands for 2nd derivative of y w.r.t theta
    
    if theta<thetai
        y(i) = L*((theta/thetai) - sin(2*pi*theta/thetai)/(2*pi));
        y1(i) = (L/thetai)*(1-cos(2*pi*theta/thetai))*(180/pi);
        y2(i) = (2*pi*L*sin(2*pi*theta/thetai)/thetai^2)*(180/pi)^2;
    
    elseif theta<thetai+thetad
        y(i) = L;
        y1(i) = 0;
        y2(i) = 0;
    
    elseif theta<thetai+thetad+thetar
        y(i) = L*(1-((theta-thetai-thetad)/thetar)+sin(2*pi*(theta-thetai-thetad)/thetar)/(2*pi));
        y1(i) = -(L/thetar)*(1-cos(2*pi*(theta-thetai-thetad)/thetar))*(180/pi);
        y2(i) = (-2*pi*L*sin(2*pi*(theta-thetai-thetad)/thetar)/thetar^2)*(180/pi)^2;
    
    else
        y(i) = 0;
        y1(i) = 0;
        y2(i) = 0;
    end
    
    %Now, find the co-ordinates of the Pitch curve and cam profile
    Xp(i) = (Rp + y(i))*sin(pi*theta/180); 
    Yp(i) = (Rp + y(i))*cos(pi*theta/180);
    %the above 2 coordinates are for the pitch curve
    
    phi(i) = atan(y1(i)/(Rp+y(i)));    %Pressure angle
    
    Xc(i) = (Rr*sin(phi(i)))*cos(theta*pi/180) + (Rp+y(i)-Rr*cos(phi(i)))*sin(theta*pi/180);
    Yc(i) = -Rr*sin(phi(i))*sin(theta*pi/180) + (Rp+y(i)-Rr*cos(phi(i)))*cos(theta*pi/180);

    %The above 2 coordinates are for the cam profile
    
    i = i+1;
    
end

%Animation
figure
hold on
xlim([-150 150])
ylim([-150 150])
j=1;    %Counter used for determining centre of roller for each position

for ang=0:pi/360:2*pi   %ang is the ngle by which it has rotated CCW w.r.t +ve y axis
    
    for i=1:1:721
        [Xr(i),Yr(i)]=rotate(Xc(i),Yc(i),0,0,-ang);     %rotates the curve CCW by ang about (0,0) 
    end
    clf;
    plot(Xr,Yr);    %Plotting the cam profile
    circle(0,Rp+y(j),Rr);   %Plotting the roller
    grid on
    xlim([-175 175])
    ylim([-150 150])
    pause(0.000000001);
    j=j+1;
    
end
hold off

%Now plotting the relations
%Pitch curve and Cam profile
figure
hold on
plot(Xp,Yp,'b')
plot(Xc,Yc,'r')
xlim([-80 80])
ylim([-100 50])
legend('Pitch Curve','Cam profile')
title('Cam Profile and Pitch Curve')
hold off

%Cam profile
figure
title('Cam Profile')
hold on
xlim([-150 150])
ylim([-150 150])
grid on
plot(Xc,Yc)
hold off

%y''(theta) vs theta
figure
plot(x,y2,'g')
title('y''''(\theta) vs \theta')
xlabel('\theta (degrees)')
ylabel('y''''(\theta)')

%y'(theta) vs theta
figure
plot(x,y1,'b')
title('y''(\theta) vs \theta')
xlabel('\theta (degrees)')
ylabel('y''(\theta)')

%Lift vs theta
figure
plot(x,y,'r')
title('Displacement vs \theta')
xlabel('\theta (degrees)')
ylabel('y (mm)')


function h = circle(x,y,r)      %Function to plot a circle given x and y coordinates of centre and the radius
hold on
th = 0:pi/50:2*pi;
xunit = r*cos(th)+x;
yunit = r*sin(th)+y;
h=plot(xunit,yunit);
hold off
end

function [xr,yr] = rotate(x,y,xc,yc,ang)    %Function to rotate a point (x,y) about (xc,yc) by an angle ang CW
xr = (x-xc)*cos(ang) + (y-yc)*sin(ang) + xc;
yr = (y-yc)*cos(ang) - (x-xc)*sin(ang) + yc;
end