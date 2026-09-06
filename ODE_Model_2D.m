%2D ODE Model for simple trajectory analysis

clear
clc
close all

function state_dot = ODE_2D(time, state)
    %state = [V, alpha, h, x]'
    %V = veloctiy
    %gamma = flight path angle (not specific heat ratio ik it gets
    %confusing cause specific heat ratio is used later but deal with it,
    %you can figure it out
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
    alpha = deg2rad(linspace(0, 40, 1000));
    Cp_max = (2/(gamma*Mach^2))*((((gamma+1)/2)*Mach^2)^((gamma)/(gamma-1))*((gamma + 1)/(2*gamma*Mach^2 - (gamma - 1)))^(1/(gamma-1))-1);
    CL_test = Cp_max.*sin(alpha).^2.*cos(alpha);
    CD_test = Cp_max.*sin(alpha).^3 + CD0;

    L_D = CL_test ./ CD_test; % Calculate lift-to-drag ratio
    [~, index] = max(L_D); % Find maximum L/D and its index
    optimalAlpha = alpha(index); % Get the angle of attack for maximum L/D

    %logic for trimming projectile at max height
    if (state(3) > 28000)
        state(2) = 0;
    end

    %reset alpha to 0 as default;
    alpha = 0;

    %logic for trimming vehicle
    h_dot = state(1)*sin(state(2));
    if(h_dot < 0)
       alpha = optimalAlpha;
    end

    
    %logic for keeping speed above Mach 3
    %{
    [~, a_ground, ~, ~] = atmoscoesa(0);
    impact_speed = 3*a_ground;
    if (state(1) < impact_speed)
        num = -2*m*g*sin(state(2));
        den = rho*state(1)^2*S;
        alpha = asin(nthroot((((num/den) - CD0)/Cp_max), 3));
    end
    %}
    
    
    %CL and CD estimations from Estimated Aerodynaics of All-Body
    %Hypersonic Aircraft Configurations, Note: CD is ignoring friction
    %drag, only accounting for estimation of induced drag plus a .05 estimation of friction drag, to get friction
    %drag need to integrate over body surface, will want to do later but
    %hoping that CFD results can be gotten and validated by the point where
    %that would be necessary

    %CL = Cp_max*sin(alpha)^2*cos(alpha);
    beta = (abs(Mach^2 - 1))^(1/2);
    C1 = 4.17/beta - .13;
    C2 = exp(.955 - (4.35/Mach));

    CL = C1*sin(alpha) + C2*sin(alpha)^2;
    %CD = Cp_max*sin(alpha)^3 + CD0;
    Km = 1;
    CD = Km*CL*tan(alpha) + .05;

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
launch_angle = deg2rad(20);
init_height = 1;
[~, a, ~, ~] = atmoscoesa(init_height); %m/s
V0 = Mach_init*a;


state_0 = [V0; launch_angle; init_height; 0];
time_range = [0 240];
options = odeset('Events', @ground);

[t, state] = ode45(@ODE_2D, time_range, state_0, options);

figure()
plot(t, state(:,1), 'LineWidth', 1, 'Color', 'Blue')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('2D Trajectory Velocity')

figure()
plot(t, rad2deg(state(:,2)), 'LineWidth', 1, 'Color', 'Blue')
xlabel('Time (s)')
ylabel('Flight Path Angle (deg)')
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

figure()
plot(state(:,4), state(:,3), 'LineWidth', 1, 'Color', 'Blue')
xlabel('Position (m)')
ylabel('Height (m)')
title('Trajectory Simulation')