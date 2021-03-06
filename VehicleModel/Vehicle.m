classdef Vehicle < LoggableObject
    %VEHICLE UAS implementaiton class
    
    properties        
        linearModel;                    %UAS Linearized model reference     
        state;                          %State evolution
        decisions;                      %Decision points in form of State snapshots
        movements;                      %Executed movements trace
        position;                       %actual position
        radius=2.5;                     %[HELPER] radius for intersection models - usually overriden to 0.6 m
        positionOrientation=[];         %Actual position and orientaiton extracted from STATE LCF
        actualPositionOrientation=[];   %Actual position and orientaiton extracted from STATE GCF
        forceSimulinkModel=false        %[HELPER]Indication if Simulink model or movement automaton should be forced
    end
    
    methods
        function obj = Vehicle(s)
            % Create UAS,
            %   s - state object with initial state at time 0.
            if (nargin ~=0)
                Cmnf.logc(obj,'Loading state')
                obj.state =s;
                Cmnf.logc(obj,'Putting state')
                obj.linearModel = LinearizedModel;
                Cmnf.logc(obj,'Creating first decision point')
                obj.decisions=[s.getLastState];
                obj.setActualPositionOrientation;
            end
        end
        
        function r=fly(obj,mt)
            %Fly one movement
            %   mt - movement type from enumeration
            obj.movements = [obj.movements,mt];
            movementCommand = Cmnf.getMovement(mt);
            lastState = obj.state.getLastState;
            if Cmnf.enableNonlinearVehicle || obj.forceSimulinkModel
                omega_alpha=movementCommand(1);
                omega_beta=movementCommand(2);
                omega_gamma=movementCommand(3); 
                velocity=movementCommand(4);
                t_sim=movementCommand(5);
                lastState = obj.state.getLastState;
                % load variables into simulation workspace
                assignin('base', 'x_0', lastState(1));
                assignin('base', 'y_0', lastState(2));
                assignin('base', 'z_0', lastState(3));
                assignin('base', 'velocity', velocity);
                assignin('base', 't_sim', t_sim);
                assignin('base', 'omega_alpha_0', lastState(7));
                assignin('base', 'omega_beta_0', lastState(8));
                assignin('base', 'omega_gamma_0', lastState(9));
                assignin('base', 'omega_alpha', omega_alpha);
                assignin('base', 'omega_beta', omega_beta);
                assignin('base', 'omega_gamma', omega_gamma);
                simOut = sim('plane','SrcWorkspace','base');
                newState = obj.state.append(simOut.get('inputState'));
            else
                linearstateState=obj.linearModel.flyLinearized(lastState,mt);
                newState=obj.state.appendLinearModel(linearstateState);
            end
            obj.decisions = [obj.decisions, newState.getLastState];
            obj.setActualPositionOrientation;
            r= newState;
        end
        
        function r=flyBuffer(obj,buffer)
            %Fly multiple movements 
            %   buffer - series of Movement Types to be applied
            for k=1:length(buffer)
                r=obj.fly(buffer(k));
            end
        end
           
        function new = copy(this)
            % Instantiate new object of the same class and copies data
            new = feval(class(this));
    
            % Copy all non-hidden properties.
            new.state = this.state.copy;
            new.linearModel = this.linearModel;
            new.movements= this.movements;
        end
        
        function r=getPositionalData(obj)
            %Gets XYZ GCF position
            s = obj.state;
            r = [s.x_pos;s.y_pos;s.z_pos;];
        end
        
        function r=setActualPositionOrientation(obj)
            %[HELPER][Internal]Sets Position(1:3) and Orientation(4:6) in GCF for actual time-frame
            x=obj.state.x_pos;
            i=length(x);
            vehicleState = obj.state.getLastState();
            x = vehicleState(1);
            y = vehicleState(2);
            z = vehicleState(3);
            alpha = vehicleState(7);
            beta = vehicleState(8);
            gamma = vehicleState(9);
            time = vehicleState(14);
            r=[x;y;z;alpha;beta;gamma;time];
            obj.positionOrientation = [obj.positionOrientation,r];
            obj.actualPositionOrientation = r;
        end
        
        function r=getActualPositionOrientation(obj)
            %Gets Position(1:3) and Orientation(4:6) in GCF for actual time-frame
            r=obj.actualPositionOrientation;
        end
        
        function r=getDecisionData(obj)
            %Gets decision points in form of column list of states
            r = [obj.decisions(1,:);obj.decisions(2,:);obj.decisions(3,:)];
        end
        
        function handles=plotTrajectory(obj,color)
            %Plots trajectory flew by UAS
            if nargin <=1
                color='b';
            end
            traj = obj.getPositionalData;
            dec = obj.getDecisionData;
            if Cmnf.enableTrajectoryTracePlot
                handles=plot3(traj(1,:),traj(2,:),traj(3,:),'Color',color);
            end
            if Cmnf.enableTracePlot
                %h1=plot3(traj(1,:),traj(2,:),traj(3,:),color);
                h2=plot3(dec(1,:),dec(2,:),dec(3,:),'Marker','o',...
                    'MarkerFaceColor',color,'Color',color);
                handles=[handles,h2];
            else
                [m,n]=size(traj);
                %lastPosition=traj(:,n);
                h4 = obj.plotPlaneModel(obj.radius,color);
                %lastPosition=traj(:,n);
                %h1=plot3(lastPosition(1,:),lastPosition(2,:),lastPosition(3,:),...
                %    'Marker','o','MarkerFaceColor',color,'Color',color);
                handles=[handles,h4];
            end
        end
        
        function handles=plotTrajectoryWide(obj,color)
            %[Helper] plots wide trajectory flew by UAS
            traj = obj.getPositionalData;
             
            if nargin ==1
                color = 'c';
            end
            
            handles=plot3(traj(1,:),traj(2,:),traj(3,:),'Linewidth',4,'Color',color)
        end
        
        function handles = plotPlaneModel(vehicle,radius,farben)
            %Plots plane model on actual position/orientation in GFC
            posOr=vehicle.actualPositionOrientation;
            position=posOr(1:3);
            orientation=rad2deg(posOr(4:6));
            scale =radius/40;
            handles=...
            c130(position(1),position(2),position(3),...
                'roll',orientation(1),...
                'pitch',-orientation(2),...
                'yaw',orientation(3)-90,...
                'color',farben,...
                'linecolor',farben,...
                'scale',scale);
                %  c130(...,'wing',WingColor)
                %  c130(...,'tailwing',TailwingColor)
                %  c130(...,'fin',FinColor)
                %  c130(...,'prop',PropellerColor)
                %  c130(...,'scale',SizeScaleFactor)
                %  c130(...,'z',ZScaleFactor)
                %  c130(...,'linestyle','LineStyle')
                %  c130(...,'linecolor',LineColor)
        end
    end
    
end

