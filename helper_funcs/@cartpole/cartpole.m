classdef cartpole < matlab.mixin.Copyable 
    properties
        animate = true;     % enable or disable animation
        animationFig;    % handle to the animation figure
        pauseTime = 0;      % [s] pause time between animation frames

        g = 9.81;   % [m/s^2] gravitational acceleration.
        l = 0.5;    % [m] the distance from the pole-cart attachment to the pole's center of mass (half length).
        m = 0.1;    % [kg] mass of the pole.
        m_cart = 1; % [kg] mass of the cart.
        mu_cart = 0;% [kg/s] friction force on the cart (default 0.01)
        mu_pole = 0;% [kg*m^2/s] friction force on the pole (default 0.001)
        k = 1/3;    % [-] constant to compute moment of inertia of the pole as I=k*m*l^2. For a pendulum (with mass only at the top), k=1, for a solid pole with uniform mass, k=1/3.

        x = zeros(4,1); % state vector
        % x(1) = s [m] cart displacement with respect to its initial position along the x-direction
        % x(2) = diff(s,t) [m/s] 
        % x(3) = theta [rad] angle that expresses the counterclockwise rotation of the pole with respect to the y-axis (theta = 0 represents the pole standing upright on the cart).
        % x(4) = diff(theta,t) [rad/s]

        u = zeros(2,1); % input vector
        % u(1) = F [N] external force acting on the cart.
        % u(2) = Fd [N] external force acting on the center of mass of the pole.
    end

    % properties needed only for plotting
    properties (Access = private)
        cart_width = 0.4;      % [m]
        cart_heigh = 0.2;       % [m]
        wheel_radius = 0.05;    % [m]
        pole_width = 0.02;      % [m]
        color = 0.7*[1 1 1];    

        hgt_cart = [];          %[hgtransform] hgtransform object of the cart.
        hgt_pole = [];          %[hgtransform] hgtransform object of the pole.
        hgt_wheels = [];        %[hgtransform] hgtransform object of the wheels.
    end

    methods
        % constructor
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = cartpole()  
        
        end

        function x_next = simulate(obj, F, dt, inputArgs)
            % Simulate the plant for a time interval dt, using control F.
            % x_next is the final plant state after time dt. 
            arguments
                obj cartpole;
                F (1,1) double {mustBeReal, mustBeFinite} = 0; % propelling force
                dt (1,1) double {mustBeNonzero, mustBeFinite} = 0.1; % sampling interval
                inputArgs.disturbance (1, 1) double {mustBeReal, mustBeFinite} = 0; % number of state-like variables
            end
            Fd = inputArgs.disturbance;
            x_next = obj.rk4(obj.x, [F; Fd], dt);
            obj.x = x_next;

            if obj.animate && ~isempty(obj.animationFig) && isvalid(obj.animationFig)
                obj.plot();
                drawnow;
                pause(obj.pauseTime);
            end
        end

        function plot(obj)
            if isempty(obj.hgt_cart) || ~ishghandle(obj.hgt_cart,'hgtransform')
                ax = axes(obj.animationFig, Title='Cart-pole system');

                % create transformation objects
                obj.hgt_cart = hgtransform(ax);
                                 
                % plot cart
                rectangle('Position',[-obj.cart_width/2,obj.wheel_radius,obj.cart_width,obj.cart_heigh], ...
                    'Curvature',0.2,'FaceColor',obj.color,'Parent',obj.hgt_cart);

                % plot wheels
                obj.hgt_wheels = [hgtransform('Parent',obj.hgt_cart); hgtransform('Parent',obj.hgt_cart)]; 

                % left wheel
                angle = linspace(0, 2*pi, 50);
                xval = -obj.cart_width/2 + 2*obj.wheel_radius + obj.wheel_radius*cos(angle);
                yval = obj.wheel_radius + obj.wheel_radius*sin(angle);
                fill(xval, yval, obj.color, 'Parent', obj.hgt_wheels(1));

                % plot spokes
                n = 8; % number fo spokes
                for j = 1:n
                    a = j*2*pi/n;
                    xval = -obj.cart_width/2 + 2*obj.wheel_radius + [0, obj.wheel_radius*cos(a)];
                    yval = obj.wheel_radius + [0, obj.wheel_radius*sin(a)];
                    line(xval, yval, 'Parent', obj.hgt_wheels(1));
                end

                % right wheel
                xval = obj.cart_width/2 - 2*obj.wheel_radius + obj.wheel_radius*cos(angle);
                yval = obj.wheel_radius + obj.wheel_radius*sin(angle);
                fill(xval, yval, obj.color, 'Parent', obj.hgt_wheels(2));

                % plot spokes
                for j = 1:n
                    a = j*2*pi/n;
                    xval = obj.cart_width/2 - 2*obj.wheel_radius + [0, obj.wheel_radius*cos(a)];
                    yval = obj.wheel_radius + [0, obj.wheel_radius*sin(a)];
                    line(xval, yval, 'Parent', obj.hgt_wheels(2));
                end

                obj.hgt_pole = hgtransform('Parent',obj.hgt_cart);  
                % plot pole
                height_pole = obj.cart_heigh + obj.wheel_radius;                
                xval = [-obj.pole_width, obj.pole_width, obj.pole_width, -obj.pole_width, -obj.pole_width];
                yval = height_pole + [0, 0, 2*obj.l, 2*obj.l, 0];
                fill(xval, yval, obj.color, 'Parent', obj.hgt_pole);

                % plot pole joint
                xval = 1.5*obj.pole_width*cos(angle);
                yval = height_pole + 1.5*obj.pole_width*sin(angle);
                fill(xval, yval, ax.XColor, 'Parent', obj.hgt_cart);

                ax.DataAspectRatioMode="manual";
                ax.Box = "off";
                ax.YAxis.Visible = "off";
                ax.XAxisLocation = "origin";
                ax.Layer = "bottom";
                ax.XLim = [-3, 3];
                ax.YLim = [obj.cart_heigh - 2*obj.l + obj.wheel_radius - 0.05, obj.cart_heigh + 2*obj.l + obj.wheel_radius + 0.05];
            end

            % translate the cart
            obj.hgt_cart.Matrix = makehgtform('translate',obj.x(1),0,0);

            % rotate the pole (move to origin, rotate, then move back)
            height_pole = obj.cart_heigh + obj.wheel_radius; 
            obj.hgt_pole.Matrix = makehgtform('translate',0,height_pole,0)*makehgtform('zrotate',obj.x(3))*makehgtform('translate',0,-height_pole,0);

            % rotate wheels (move to origin, rotate, then move back)
            p0 = [-obj.cart_width/2 + 2*obj.wheel_radius, obj.wheel_radius];
            obj.hgt_wheels(1).Matrix = makehgtform('translate',p0(1),p0(2),0)*makehgtform('zrotate',-obj.x(1)/obj.wheel_radius)*makehgtform('translate',-p0(1),-p0(2),0);

            p0 = [obj.cart_width/2 - 2*obj.wheel_radius, obj.wheel_radius];
            obj.hgt_wheels(2).Matrix = makehgtform('translate',p0(1),p0(2),0)*makehgtform('zrotate',-obj.x(1)/obj.wheel_radius)*makehgtform('translate',-p0(1),-p0(2),0);
        end
    end

    methods (Access=protected)
        function x_next = rk4(obj, x, u, dt)
            subintervals = 4;
            h = dt/subintervals;
            x_next = x;
            for j = 1:subintervals
                k1 = obj.xdot(x_next, u);
                k2 = obj.xdot(x_next + 0.5*h*k1, u);
                k3 = obj.xdot(x_next + 0.5*h*k2, u);
                k4 = obj.xdot(x_next + h*k3, u);
                x_next = x_next + h*(k1 + 2*k2 + 2*k3 + k4)/6;
            end
        end

        function f = xdot(obj, x, u)
            Ds = x(2);
            theta = x(3);
            Dtheta = x(4);
            F = u(1);
            Fd = u(2);  % disturbance
            M = obj.m_cart + obj.m; % total mass
            denom = (1 + obj.k)*M - obj.m*cos(theta)^2;
            f = [ Ds;
                (  (obj.k + 1)*(F + Fd - obj.mu_cart*Ds - obj.l*obj.m*sin(theta)*Dtheta^2) ...
                 + cos(theta)*(-Fd*cos(theta) + obj.g*obj.m*sin(theta) + obj.mu_pole*Dtheta/obj.l)  )/denom;
                Dtheta; 
                (  obj.g*M*sin(theta) + cos(theta)*(F - obj.m_cart*Fd/obj.m ...
                 - obj.l*obj.m*sin(theta)*Dtheta^2 - obj.mu_cart*Ds) ...
                 - obj.mu_pole*M*Dtheta/(obj.m*obj.l)  )/(obj.l*denom)];
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2025-12.