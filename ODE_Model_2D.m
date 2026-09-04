%2D ODE Model for simple trajectory analysis

clear
clc
close all

function state_dot = ODE_2D(time, state)
    %state = [V, alpha, h, x]'
    %V = veloctiy
    %alpha = flight path angle
    %h = height
    %x = distance
    m = 120; %kg
    S = 1/2*(.4^2)*pi;
    g = 9.8; %m/s^2
    
    [~, a, ~, rho] = atmoscoesa(state(3));

    Mach = state(1)/a;

    %empircal formula for CL/CD for testing purposes
    CD0 = .05;
    gamma = 1.4;
    alpha = [0, 40, 1000];
    Cp_max = (2/(gamma*Mach^2))*((((gamma+1)/2)*Mach^2)^((gamma)/(gamma-1))*((gamma + 1)/(2*gamma*Mach^2 - (gamma - 1)))^(1/(gamma-1))-1);
    CL_test = Cp_max.*sind(alpha).^2.*cosd(alpha) + .1;
    CD_test = Cp_max.*sind(alpha).^3 + CD0;

    L_D = CL_test ./ CD_test; % Calculate lift-to-drag ratio
    [maxL_D, index] = max(L_D); % Find maximum L/D and its index
    optimalAlpha = alpha(index); % Get the angle of attack for maximum L/D

    %if projectile is moving downwards then hold at best L/D
    %if (state(3) > 10000)
     %   state(2) = deg2rad(optimalAlpha);
    %end

    %logic for trimming projectile at max height
    if (state(3) > 28000)
        state(2) = 0;
    end    

    CL = Cp_max.*sin(state(2)).^2.*cos(state(2)) + .1;
    CD = Cp_max.*sin(state(2)).^3 + CD0;

    D = 1/2*rho*state(1)^2*S*CD;
    L = 1/2*rho*state(1)^2*S*CL;

    state_dot = [-D/m - g*sin(state(2));
                 L/(m*state(1)) - (g/state(1))*cos(state(2));
                 state(1)*sin(state(2));
                 state(1)*cos(state(2))];
end

%event to stop integration when the ground is hit
function [value, isterminal, direction] = ground(t, state)
    value = (state(3) <= 0);
    isterminal = 1;
    direction = 0;
end

Mach_init = 8;
a = 340; %m/s
V0 = Mach_init*a;

state_0 = [V0; 20; 1; 0];
time_range = [0 240];
options = odeset('Events', @ground);

prev_height = state_0(3);

[t, state] = ode45(@ODE_2D, time_range, state_0, options);

figure()
plot(t, state(:,1), 'LineWidth', 1, 'Color', 'Blue')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('2D Trajectory Velocity')

figure()
plot(t, state(:,2), 'LineWidth', 1, 'Color', 'Blue')
xlabel('Time (s)')
ylabel('Flight Path Angle (rad)')
title('2D Trajectory Flight Path Angle')

figure()
plot(t, state(:,3), 'LineWidth', 1, 'Color', 'Blue')
xlabel('Time (s)')
ylabel('Height (m)')
title('2D Trajectory Height')

figure()
plot(t, state(:,4), 'LineWidth', 1, 'Color', 'Blue')
xlabel('Time (s)')
ylabel('Position (m)')
title('2D Trajectory Position')

