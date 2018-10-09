classdef ReachSetCalculation<uint32

    enumeration
        Harmonic(1),             %Default navigation
        CellSpread(2),           %Maximum coverage
        LimitedPassing(3),       %Maximum smoothness
        ACASLike(4),             %ACAS like
    end
    
    methods(Static)
       
        function r=toString(member)
            r=char(member);
        end
    
        function r=createReachSet(avoidanceGrid,type,parameters) 
            %% default parameters
            % Avoidance grid
            defDs=0;defDf=10; defTheta=pi/4; defPhi= pi/6;
            defLayers=10; defHCell=7; defVCell=5;
            % Cellspread
            defFarCount =  8;
            defNearCount = 1;
            % LimitedPassing
            defPassRatio = 7;
            % ACAS-XU reach set
            defseparations = [MovementGroup.Horizontal,MovementGroup.Vertical];
            %% Nargin checks
            if nargin <= 0
                avoidanceGrid=AvoidanceGrid(defDs,defDf,-defTheta,defTheta,-defPhi,defPhi,defLayers,defHCell,defVCell);
                Cmnf.logc(avoidanceGrid,'-- Warning -- No Avoidance grid detected, creating dumy avoidance grid');
                Cmnf.logc(avoidanceGrid,'-- Warning -- Range 10m, step 1m, Hrange pi/4, Vrange pi/6, Hcell 7, vCell 6');
            end
            if nargin <= 1
                type = ReachSetCalculation.Harmonic;
                Cmnf.logc(avoidanceGrid,'-- Warning -- No Type selected selecting Harmonic as default');
            end
            
            %% Parameter sanity check
            % Avoidance Grid
            try 
                avoidanceGrid.cellMap; % just call some specific property, avoidance grid can not be created via empty constructor anyway!
                Cmnf.logc(avoidanceGrid,'-- PASS -- Proper avoidance grid');
            catch ME                 
                avoidanceGrid=AvoidanceGrid(defDs,defDf,-defTheta,defTheta,-defPhi,defPhi,defLayers,defHCell,defVCell);                Cmnf.logc(avoidanceGrid,'-- Warning -- Malformed avoidance Grid creating default with parameters');
                Cmnf.logc(avoidanceGrid,'-- Warning -- Range 10m, step 1m, Hrange pi/4, Vrange pi/6, Hcell 7, vCell 6');
            end
            % Type
            if type == ReachSetCalculation.Harmonic ...
                    || type == ReachSetCalculation.LimitedPassing ...
                    || type == ReachSetCalculation.CellSpread ...
                    || type == ReachSetCalculation.ACASLike
                Cmnf.logc(avoidanceGrid,['-- PASS -- Proper type selected ',ReachSetCalculation.toString(type)]);
            else
                Cmnf.logc(avoidanceGrid,'-- Warning -- Malformed type of calculation setting default value, Harmonic');
                type = ReachSetCalculation.Harmonic;
            end
            
            % Map
            if nargin <= 2 && (type == ReachSetCalculation.LimitedPassing ...
                    || type == ReachSetCalculation.CellSpread ...
                    || type == ReachSetCalculation.ACASLike)
                parameters=containers.Map;
                Cmnf.logc(avoidanceGrid,'-- Warning -- No Parameters provided, creating empty map');
            else
                Cmnf.logc(avoidanceGrid,'-- PASS -- proper object MAP passed');
            end
            %% Harmonic Reach set calculation
            if type == ReachSetCalculation.Harmonic
                Cmnf.logc(avoidanceGrid,['-- Calculation -- START ',ReachSetCalculation.toString(type)]);
                avoidanceGrid.precalculateHarmonicReachSet(LinearizedModel());
                Cmnf.logc(avoidanceGrid,['-- Calculation -- END ',ReachSetCalculation.toString(type)]);
            end
            %% CellSpread
            if type == ReachSetCalculation.CellSpread
                %load farCount
                try
                    farCount=parameters('farCount');
                    Cmnf.logc(avoidanceGrid,['-- PASS -- Far trajectories count:',mat2str(farCount)]);
                catch ME
                    farCount=defFarCount;
                    Cmnf.logc(avoidanceGrid,['-- Warning --  loading default Far trajectories count:',mat2str(farCount)]);
                end
                
                try 
                    nearCount=parameters('nearCount');
                    Cmnf.logc(avoidanceGrid,['-- PASS -- near trajectories count:',mat2str(nearCount)]);
                catch ME
                    nearCount=defNearCount;
                    Cmnf.logc(avoidanceGrid,['-- Warning -- loading near trajectories count:',mat2str(nearCount)]);
                end
                Cmnf.logc(avoidanceGrid,['-- Calculation -- START ',ReachSetCalculation.toString(type)]);
                avoidanceGrid.precalculateCellSpreadReachSet(LinearizedModel,farCount,nearCount);
                Cmnf.logc(avoidanceGrid,['-- Calculation -- END ',ReachSetCalculation.toString(type)]);
            end
            %% LimitedPassing
            if type == ReachSetCalculation.LimitedPassing
                try 
                    passRatio = parameters('passRatio');
                    Cmnf.logc(avoidanceGrid,['-- PASS -- pass ratio:',mat2str(passRatio)]);
                catch
                    passRatio = defPassRatio;
                    Cmnf.logc(avoidanceGrid,['-- Warning -- loaded default pass ratio:',mat2str(passRatio)]);
                end
                Cmnf.logc(avoidanceGrid,['-- Calculation -- START ',ReachSetCalculation.toString(type)]);
                avoidanceGrid.precalculateLimitedPassingReachSet(LinearizedModel,passRatio);
                Cmnf.logc(avoidanceGrid,['-- Calculation -- END ',ReachSetCalculation.toString(type)]);
            end
            %% ACAS
            if type == ReachSetCalculation.ACASLike
                try
                    separations=parameters('separations');
                    for sep=separations
                        isenum(sep);
                    end
                    Cmnf.logc(avoidanceGrid,['-- PASS -- Separations:',mat2str(separations)]);
                catch ME
                    separations=defseparations;
                    Cmnf.logc(avoidanceGrid,'-- Warning -- No separations provided, loading defautls Horizontal/Vertical');
                end
                Cmnf.logc(avoidanceGrid,['-- Calculation -- START ',ReachSetCalculation.toString(type)]);
                avoidanceGrid.precalculateACASReachSet(LinearizedModel,separations);
                Cmnf.logc(avoidanceGrid,['-- Calculation -- END ',ReachSetCalculation.toString(type)]);
            end
            %% Return the avoidance grid
            r=avoidanceGrid;
        end
    end
end

