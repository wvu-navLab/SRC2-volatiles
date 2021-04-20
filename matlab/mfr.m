%% Mat File Reader :: Volatile Locations

close all;
clear all;
clc;

k = 1;      % Get the  k-th sample from bagfile.
files = dir;

nbags = 0;
for m=1:length(files)
    if(count(files(m).name,'.mat'))
        nbags = nbags + 1;
    end
end
col=hsv(nbags);

Reg = []; Ice = []; Eth = []; Met = []; Mtl = []; 
Cdx = []; Amm = []; Hyd = []; Sul = [];  
    
nmats = 1;
for n=1:length(files)
    if(~contains(files(n).name,'.mat'))
        % Do nothing
    else
        filename_mat = files(n).name;
        load(filename_mat);

        % Volatiles
        Reg = [Reg posReg];
        Ice = [Ice posIce];
        Eth = [Eth posEth];
        Met = [Met posMet];
        Mtl = [Mtl posMtl];
        Cdx = [Cdx posCdx];
        Amm = [Amm posAmm];
        Hyd = [Hyd posHyd];
        Sul = [Sul posSul];        
        
        nmats = nmats + 1;
   
    end % if
end % form

posVols = [Reg Ice Eth Met Mtl Cdx Amm Hyd Sul];
        
radius_from_origin = vecnorm(posVols,2,1);
radius_xy = sqrt(radius_from_origin.^2 - mean(posVols(3,:))^2);
angle_xy = atan2(posVols(2,:),posVols(1,:));
rmin = min(radius_xy);      rmax = max(radius_xy);

% save('posVolatiles','Reg','Ice','Eth','Met','Mtl','Cdx','Amm','Hyd','Sul','posVols');

% Since the height (z-axis) is constant, only xy-axis plane is considered
figure
subplot(2,2,[1 3])
plot(Reg(1,:), Reg(2,:),'o'); hold on;
plot(Ice(1,:), Ice(2,:),'*');
plot(Eth(1,:), Eth(2,:),'+');
plot(Met(1,:), Met(2,:),'.');
plot(Mtl(1,:), Mtl(2,:),'s');
plot(Cdx(1,:), Cdx(2,:),'p');
plot(Amm(1,:), Amm(2,:),'v');
plot(Hyd(1,:), Hyd(2,:),'^');
plot(Sul(1,:), Sul(2,:),'h'); grid on;
viscircles([0 0],rmin);
viscircles([0 0],rmax);
axis([-100 100 -100 100]);
% axis equal
% Set up plot and filepath to save graph
set(gcf, 'Position', [100, 100, 1200, 500]); % [left bottom width height]
title(sprintf('R_{min}= %2.1f m, R_{max}=%2.1f m (%d vols)',...
      rmin,rmax,length(radius_xy)));
% legend('Regolith','Ice','Ethane','Methane','Methanol','C-dioxide','Ammonia','H-sulfite', 'S-dioxide')

% radius_xy = [7 5 12 1 3  9  8  3  5 12  1  8  8]; % For test (DEBUG)
% angle_xy = [1 3  5  7 9 11 23 15  3 19 21 23 23];

[c1,d1] =sort(radius_xy);
radius_sorted = radius_xy(d1);
angle_sorted = angle_xy(d1); % Same index as radius (to track)


idx_equal = [];
for p=2:length(radius_sorted)
    if ( (radius_sorted(p) == radius_sorted(p-1)) && ...
         (angle_sorted(p) == angle_sorted(p-1)))
     
        idx_equal = [idx_equal p];
    end
end

radius_sorted(idx_equal) = [];
angle_sorted(idx_equal) = [];

nUniqueVols = length(radius_sorted);

subplot(222)
plot((radius_sorted),'.'); grid on;
yline(rmin, 'r--', 'LineWidth', 2); yline(rmax, 'r--', 'LineWidth', 2);
title('Radius from origin (m)'); % ylim([-.1 0.5])
subplot(224)
plot((rad2deg(angle_sorted)),'.'); grid on
title('Angle (deg)'), xlabel('Volatile ID (unique location)')

% Check if there is some volatile in a repeated location
find(diff(radius_sorted) == 0)
find(diff(angle_sorted) == 0)


% Origin of the map
x0 = 0;
y0 = 0;

% Generate a synthetic volatile map for comparision agaisnt the original
rng('default'); % Initialize the rng to make the results repeatable.
rng(23);         % Initialize the generator using a seed of 23.

% Generate a random radius between the inner and outer radius
r = (rmax-rmin).*rand(nUniqueVols,1) + rmin;
% Generate a random radius between 0 ~ 2pi
alpha = 2*pi*rand(nUniqueVols,1);
volatile_loc = [r.*cos(alpha)+x0 r.*sin(alpha)+y0]';


figure
subplot(121)
plot(Reg(1,:), Reg(2,:),'b.'); hold on;
plot(Ice(1,:), Ice(2,:),'b.');
plot(Eth(1,:), Eth(2,:),'b.');
plot(Met(1,:), Met(2,:),'b.');
plot(Mtl(1,:), Mtl(2,:),'b.');
plot(Cdx(1,:), Cdx(2,:),'b.');
plot(Amm(1,:), Amm(2,:),'b.');
plot(Hyd(1,:), Hyd(2,:),'b.');
plot(Sul(1,:), Sul(2,:),'b.'); grid on;
viscircles([0 0],rmin);
viscircles([0 0],rmax);
title('Simulation')
axis([-100 100 -100 100]);

subplot(122)
plot(volatile_loc(1,:),volatile_loc(2,:),'k.')
viscircles([0 0],rmin);
viscircles([0 0],rmax);
title('MATLAB')
axis([-100 100 -100 100]);
grid on;
set(gcf, 'Position', [100, 100, 1200, 500]); % [left bottom width height]

radius(1) = 0
for i=1:20
    radius(i+1) = i*5;
    n_vols(i) = sum((radius(i)<radius_from_origin) .* (radius(i+1)>radius_from_origin));
    area(i) = pi*(radius(i+1)^2-radius(i)^2);
    concentration(i)=n_vols(i)/area(i);
end

figure
subplot(221)
histogram(radius_from_origin,15,'Normalization','probability')
xlabel('Radius [m]')
ylabel('Probability')
subplot(222)
histogram(angle_xy+pi,15,'Normalization','probability')
xlabel('Angle [rad]')
ylabel('Probability')
subplot(2,2,[3,4]);
plot(radius(2:end),concentration)
xlabel('Radius [m]')
ylabel('Concentration [vols/m^2]')

