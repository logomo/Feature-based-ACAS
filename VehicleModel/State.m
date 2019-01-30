classdef State < LoggableObject
    %STATE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        figure_angles = 1;      % orientation figure id
        figure_position = 2;    % position evoulution figure id
        x_pos = [];             % X-position GCF
        y_pos = [];             % Y-position GCF
        z_pos = [];             % Z-position GCF
        x_der = [];             % dx/dt GFC
        y_der = [];             % dy/dt GFC
        z_der = [];             % dz/dt GFC
        alpha = [];             % Roll [rad]
        beta = [];              % Pitch [rad]
        gamma = [];             % Yaw [rad]
        omega_alpha = [];       % dRoll/dt [rad/s]
        omega_beta = [];        % dPitch/dt [rad/s]
        omega_gamma = [];       % dYaw/dt [rad/s]
        velocity = [];          % velocity [m/s]
        time = [];              % global simulaiton time [s]
    end
    
    methods
        function obj = State(omega_alpha_0,omega_beta_0,omega_gamma_0,x_0,y_0,z_0, velocity)
            % State initializaiton with parameters - refer to param section
            if (nargin ~=0)
                obj.x_pos = [x_0];
                obj.y_pos = [y_0];
                obj.z_pos = [z_0];
                obj.x_der = [0];
                obj.y_der = [0];
                obj.z_der = [0];
                obj.alpha = [omega_alpha_0];
                obj.beta = [omega_beta_0];
                obj.gamma = [omega_gamma_0];
                obj.omega_alpha = [0];
                obj.omega_beta = [0];
                obj.omega_gamma = [0];
                obj.velocity = [velocity];
                obj.time = [0];
                
                %Log function
                Cmnf.logc(obj,['omega_alpha_0',' : ',mat2str(obj.omega_alpha)]);
                Cmnf.logc(obj,['omega_beta_0',' : ',mat2str(obj.omega_beta)]);
                Cmnf.logc(obj,['omega_gamma_0',' : ',mat2str(obj.omega_gamma)]);
                Cmnf.logc(obj,['x_0',' : ',mat2str(obj.x_pos)]);
                Cmnf.logc(obj,['y_0',' : ',mat2str(obj.y_pos)]);
                Cmnf.logc(obj,['z_0',' : ',mat2str(obj.z_pos)]);
                Cmnf.logc(obj,['velocity',' : ',mat2str(obj.velocity)]);
            end
        end
        
        
        function r = append(obj,inputState)
            %Append the state by inputState
            inputState= inputState';
            obj.velocity = [obj.velocity,inputState(1,:)];
            obj.x_der = [obj.x_der, inputState(2,:)];
            obj.y_der = [obj.y_der, inputState(3,:)];
            obj.z_der = [obj.z_der, inputState(4,:)];
            obj.x_pos = [obj.x_pos, inputState(5,:)];
            obj.y_pos = [obj.y_pos, inputState(6,:)];
            obj.z_pos = [obj.z_pos, inputState(7,:)];
            obj.omega_alpha = [obj.omega_alpha, inputState(8,:)];
            obj.omega_beta = [obj.omega_beta, inputState(9,:)];
            obj.omega_gamma = [obj.omega_gamma, inputState(10,:)];
            obj.alpha = [obj.alpha,inputState(11,:)];
            obj.beta = [obj.beta,inputState(12,:)];
            obj.gamma = [obj.gamma,inputState(13,:)];
            t_length = length(obj.time);
            t_last = obj.time(t_length);
            obj.time = [obj.time, inputState(14,:)+t_last];
            r=obj;
        end
        
        function r = appendLinearModel(obj,state)
            %Append by predictor state contains only position orientation and time
            obj.x_pos = [obj.x_pos state(1)];
            obj.y_pos = [obj.y_pos state(2)];
            obj.z_pos = [obj.z_pos state(3)];
            obj.x_der = [obj.x_der 0];
            obj.y_der = [obj.y_der 0];
            obj.z_der = [obj.z_der 0];
            obj.alpha = [obj.alpha state(4)];
            obj.beta = [obj.beta state(5)];
            obj.gamma = [obj.gamma state(6)];
            obj.omega_alpha = [obj.omega_alpha 0];
            obj.omega_beta = [obj.omega_beta   0];
            obj.omega_gamma = [obj.omega_gamma 0];
            obj.velocity = [obj.velocity 0];
            obj.time = [obj.time state(7)];
            r= obj;
        end
        
        function r=plot(obj)
            %Plot figures for position/orientaiton evolution over time
            figure(obj.figure_angles);
            subplot(3,2,1);
            hold on
            title('Roll derivative \omega_\alpha')
            xlabel('Time [s]');
            ylabel('[rad/s]')
            grid on;
            plot(obj.time,obj.omega_alpha);
            hold off;
            
            subplot(3,2,2);
            hold on;
            grid on;
            title('Roll \alpha')
            xlabel('Time [s]');
            ylabel('[rad]');
            plot(obj.time,obj.alpha);
            hold off;
            
            
            subplot(3,2,3);
            hold on;
            grid on;
            title('Pitch derivate \omega_\beta');
            xlabel('Time [s]');
            ylabel('[rad/s]')
            plot(obj.time,obj.omega_beta);
            hold off;
            
            subplot(3,2,4);
            hold on;
            grid on;
            title('Pitch \beta');
            xlabel('Time [s]');
            ylabel('[rad]')
            plot(obj.time,obj.beta);
            hold off;
             
            subplot(3,2,5);
            hold on;
            grid on;
            title('Yaw derivate \omega_\gamma');
            xlabel('Time [s]');
            ylabel('[rad/s]');
            plot(obj.time,obj.omega_gamma);
            hold off;
            
            subplot(3,2,6);
            hold on;
            grid on;
            title('Yaw \gamma');
            xlabel('Time [s]');
            ylabel('[rad]');
            plot(obj.time,obj.gamma);
            hold off;
            
            figure(obj.figure_position);
            subplot(3,2,1);
            hold on;
            grid on;
            title('Velocity x_{der}');
            xlabel('Time [s]');
            ylabel('[m/s]');
            plot(obj.time, obj.x_der);
            hold off;
            
            subplot(3,2,2);
            hold on;
            grid on;
            title('Position x');
            xlabel('Time [s]');
            ylabel('[m]');
            plot(obj.time,obj.x_pos);
            hold off;
            
            subplot(3,2,3);
            hold on;
            grid on;
            title('Velocity y_{der}');
            xlabel('Time [s]');
            ylabel('[m/s]');
            plot(obj.time, obj.y_der);
            hold off;
            
            subplot(3,2,4);
            hold on;
            grid on;
            title('Position y');
            xlabel('Time [s]');
            ylabel('[m]');
            plot(obj.time,obj.y_pos);
            hold off;
            
            subplot(3,2,5);
            hold on;
            grid on;
            title('Velocity z_{der}');
            xlabel('Time [s]');
            ylabel('[m/s]');
            plot(obj.time, obj.z_der);
            hold off;
            
            subplot(3,2,6);
            hold on;
            grid on;
            title('Position z');
            xlabel('Time [s]');
            ylabel('[m]');
            plot(obj.time,obj.z_pos);
            hold off;
        end
        
        function r = getLastState(obj)
            %Get last state as vector
            p=length(obj.x_pos);
            r = [obj.x_pos(p);
                 obj.y_pos(p);
                 obj.z_pos(p);
                 obj.x_der(p);
                 obj.y_der(p);
                 obj.z_der(p);
                 obj.alpha(p);
                 obj.beta(p);
                 obj.gamma(p);
                 obj.omega_alpha(p);
                 obj.omega_beta(p);
                 obj.omega_gamma(p);
                 obj.velocity(p);
                 obj.time(p)];
        end
        function r = getLastStatePredictor(obj)
            %Get last state for predictor
            p=length(obj.x_pos);
            r = [obj.x_pos(p);
                 obj.y_pos(p);
                 obj.z_pos(p);
                 obj.alpha(p);
                 obj.beta(p);
                 obj.gamma(p);
                 obj.time(p)];
        end
        function new = copy(this)
            % Instantiate new object of the same class.
            new = feval(class(this));
 
            % Copy all non-hidden properties.
            p = properties(this);
            for i = 1:length(p)
                new.(p{i}) = this.(p{i});
            end
        end
    end
end

