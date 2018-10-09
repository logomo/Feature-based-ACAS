classdef LinearizedModel
    %LINEARIZEDMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
      table
      stepTime=1;
    end
    
    methods
        function obj=LinearizedModel()
            a=load('LinearModelData.mat','linearizedStates');
            obj.table = a.linearizedStates;
        end
        
        function state=predict(obj,initState,mt)
            movement = obj.table(:,mt+1);
            [m,n]=size(initState);
            s_x = initState(1,n);
            s_y = initState(2,n);
            s_z = initState(3,n);
            s_alpha = initState(4,n);
            s_beta = initState(5,n);
            s_gamma = initState(6,n);
            s_time = initState(7,n);
            m_x = movement(5);
            m_y = movement(6);
            m_z =  movement(7);
            m_alpha = movement(11);
            m_beta = movement(12);
            m_gamma = movement(13);
            m_time = movement(14);
            %calculate new time offset
            n_time = s_time + m_time;
            %give proper orientation based on vehicle position
            pos = Cmnf.rot3D(s_alpha,s_beta,s_gamma,[m_x;m_y;m_z]);
            %give proger positional offset
            pos = pos + [s_x;s_y;s_z]; 
            %calculate proper rotation aftermovement
            rot=[s_alpha+m_alpha;s_beta+m_beta;s_gamma+m_gamma];
            %append existing state 
            state = [initState, [pos;rot;n_time]];
        end
        
        function state=predictBuffer(obj,initState,buffer)
            state = initState;
            for k=1:length(buffer)
                state=obj.predict(state,buffer(k));
            end
        end
        
        function r=getVelocity(obj)
            movement = obj.table(:,1);
            r=movement(1);
        end
        
        function state=flyLinearized(obj,previousState,mt)
            p_x_pos = previousState(1);
            p_y_pos = previousState(2);
            p_z_pos = previousState(3);
            %p_x_der = previousState(4);
            %p_y_der = previousState(5);
            %p_z_der = previousState(6);
            p_alpha = previousState(7);
            p_beta = previousState(8);
            p_gamma = previousState(9);
            %p_omega_alpha = previousState(10);
            %p_omega_beta = previousState(11);
            %p_omega_gamma = previousState(12);
            %p_velocity = previousState(13);
            p_time = previousState(14);
            %movement = obj.table(:,mt+1);
            
            previousPosition        =   [p_x_pos;p_y_pos;p_z_pos];
            previousOrientation     =   [p_alpha;p_beta;p_gamma];
            previousTimeStamp       =   [p_time];
            previousLinearState     =   [previousPosition;previousOrientation;previousTimeStamp];
            predicted = obj.predict(previousLinearState,mt);
            state=predicted(:,2);
            
            % calculate new position
            %n_pos=Cmnf.rot3Dvec([movement(1);movement(2);movement(3)],[p_alpha,p_beta,p_gamma]);
            %n_pos=[p_x_pos;p_y_pos;p_z_pos]+n_pos;
            
            %calculate new position derivatives
            %n_der=Cmnf.rot3Dvec([movement(4);movement(5);movement(6)],[p_alpha,p_beta,p_gamma]);
            
            %calculate new angles
            %n_angle = [p_alpha + movement(7);p_beta+ movement(8);p_gamma + movement(9)];
            %n_angle = [atan2(sin(n_angle(1)),cos(n_angle(1)));
            %           atan2(sin(n_angle(2)),cos(n_angle(2)));
            %           atan2(sin(n_angle(3)),cos(n_angle(3)))];
            %calculate new derivatives 
            %n_omega = [movement(10);movement(11);movement(12)];
            
            %calculate new velocity
            %n_velocity = movement(13);
            
            % calculate new time
            %n_time = p_time + movement(14);
            
            %put newstate
            %newState = [n_pos;n_der;n_angle;n_omega;n_velocity;n_time];
            %state= [previousState,newState];
        end
    end
    
end

