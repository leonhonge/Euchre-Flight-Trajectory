%script to find max L/D of empircal CL CD formula
clear
close all
clc

alpha = linspace(0, 40, 1000);

Mach = 8;

CD0 = .05;
gamma = 1.4;
Cp_max = (2/(gamma*Mach^2))*((((gamma+1)/2)*Mach^2)^((gamma)/(gamma-1))*((gamma + 1)/(2*gamma*Mach^2 - (gamma - 1)))^(1/(gamma-1))-1);
CL = Cp_max.*sind(alpha).^2.*cosd(alpha) + .1;
CD = Cp_max.*sind(alpha).^3 + CD0;

figure()
plot(CD, CL, 'LineWidth', 1)
xlabel('CD')
ylabel('CL')
title ('CL vs. CD')

L_D = CL ./ CD; % Calculate lift-to-drag ratio
[maxL_D, index] = max(L_D); % Find maximum L/D and its index
optimalAlpha = alpha(index); % Get the angle of attack for maximum L/D

figure()
plot(alpha, L_D, 'LineWidth', 1)
xlabel('Alpha')
ylabel('L/D')
title('L/D vs. Alpha')

disp('Max L/D:')
disp(maxL_D)
disp('Optimal alpha:')
disp(optimalAlpha)