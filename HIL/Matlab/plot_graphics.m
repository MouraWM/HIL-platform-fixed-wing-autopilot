%% Plotagem do resultado

close all;

R = 1000;
c1 = R*cos(2*pi/5);
c2 = R*cos(pi/5);
s1 = R*sin(2*pi/5);
s2 = R*sin(4*pi/5);

WP = [ 0    0    400  20; 
	   R    0    400  20; 
	   -c2  s2   420  20; 
	   c1   -s1  380  20; 
	   c1   s1   420  20; 
	   -c2  -s2  380  20; 
	   R    0    400  20]';

%% Trajectory   
f = figure (1); hold on; grid on;
f.Position = [285 39 1281 818];
plot (Y.Data(:,10), Y.Data(:,11), ":b", "lineWidth", 4);
plot (WP(1,:), WP(2,:), "r", "lineWidth", 2);
t = plot (WP(1,:), WP(2,:), "xk", "lineWidth", 5);
t.MarkerSize = 16;
t = xlabel('North (m)');
t(1).FontSize = 16; t(1).FontName = 'Times'; % t(1).FontWeight = 'bold';
t = ylabel('East (m)');
t(1).FontSize = 16; t(1).FontName = 'Times'; % t(1).FontWeight = 'bold';
t = text(WP(1,:)+10, WP(2,:)+30,['  WP1       '; '   WP2 / WP7'; '  WP3       '; '  WP4       '; '  WP5       '; '  WP6       '; '   WP2 / WP7']);
t(1).FontSize = 16; %t(1).FontWeight = 'bold';
t(2).FontSize = 16; %t(2).FontWeight = 'bold';
t(3).FontSize = 16; %t(3).FontWeight = 'bold';
t(4).FontSize = 16; %t(4).FontWeight = 'bold';
t(5).FontSize = 16; %t(5).FontWeight = 'bold';
t(6).FontSize = 16; %t(6).FontWeight = 'bold';
t(7).FontSize = 16; %t(7).FontWeight = 'bold';
t=legend('   Actual Trajectory   ','   Ideal Trajectory   ');
t(1).FontSize = 16; t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
t(1).Position = [0.1621 0.8390 0.1764 0.0580];

%% Velocity   
VR = ones(1, length(Y.Time(:)))*20;
f = figure (2); hold on; grid on;
f.Position = [285 39 1281 818];
plot (Y.Time(:), Y.Data(:,1), ":b", "lineWidth", 4);
plot (Y.Time(:), VR, "r", "lineWidth", 2);
t = xlabel('Time (s)');
t(1).FontSize = 16; t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
t = ylabel('Aerodynamic Velocity  (m/s)');
t(1).FontSize = 16;  t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
t=legend('   Speed obtained','   Reference Speed');
t(1).FontSize = 16; t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
t(1).Position = [0.1621 0.8390 0.1764 0.0580];
ylim([16 24]);

%% Altitude
f = figure (3); hold on; grid on;
f.Position = [285 39 1281 818];
plot (Y.Time(:), Y.Data(:,12), ":b", "lineWidth", 4);
plot (Y.Time(2:end), He_Psie.Data(2:end,1), "r", "lineWidth", 2);
t = xlabel('Time (s)');
t(1).FontSize = 16; t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
t = ylabel('Altitude (m)');
t(1).FontSize = 16; t(1).FontName = 'Times';  %t(1).FontWeight = 'bold';
t=legend('   Altitude obtained','   Reference Altitude');
t(1).FontSize = 16; t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
%t(1).Position = [0.1509 0.8521 0.1356 0.0525];
ylim([370 430]);

%% Guinada
f = figure (4); hold on; grid on;
f.Position = [285 39 1281 818];
plot (Y.Time(:), Y.Data(:,9)*180/pi, ":b", "lineWidth", 4);
plot (Y.Time(1:55915), He_Psie.Data(1:55915,2)*180/pi, "r", "lineWidth", 2);
t = xlabel('Time (s)');
t(1).FontSize = 16; t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
t = ylabel('Yaw angle (degrees)');
t(1).FontSize = 16; t(1).FontName = 'Times';  %t(1).FontWeight = 'bold';
t=legend('   Angle obtained','   Reference Angle');
t(1).FontSize = 16; t(1).FontName = 'Times'; %t(1).FontWeight = 'bold';
t(1).Position = [0.1621 0.8390 0.1764 0.0580];
ylim([0 800]);