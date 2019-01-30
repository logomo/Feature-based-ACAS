classdef GridCellPlotData<handle
    %GRIDCELLPLOTDATA Edge plot data for cell [HELPER]
    
    
    properties
        horizontalCell; %2D cell boundary for horizontal
        verticalCell;   %2D cell boundary for vertical 
        probabilities;  %Vector of ratings related to the cells
    end
    
    methods
        function obj=GridCellPlotData(hc,vc,prob)
            % Create data object
            %   hc - horizontal cell points
            %   vc - vertical cell points
            %   prob - rating distribution
            obj.horizontalCell=hc;
            obj.verticalCell=vc;
            obj.probabilities=prob;
        end
        
        function r=getColorVectorGS(obj,probId)
            %Creates a color vector gray scale
            %0-black,1-white
            sca=1-obj.probabilities(probId);
            r=[sca,sca,sca];
        end
        
        function r=getcolorVectorRGB(obj,probId)
            %Creates color vector based on rating
           if probId==6
               sca=obj.probabilities(3);
           else
               sca=obj.probabilities(probId);
           end
           %biased rating overflow fix
           if sca > 1
               sca=1;
           end
           if sca < 0
               sca=0;
           end
           %red Green Blue
           r = [0,0,0];
           %pReachability
           if (probId==1)
               r=[1-sca,sca,0];
           end
           %pVisibility
           if (probId==2)
               r=[0,sca,sca];
           end
           %pObstacle
           if (probId==3)
               r=[1,1-sca,1-sca];
           end
           %pDecision
           if (probId==4)
               r=[sca,0,sca];
           end
           %pFeasibility
           if (probId==5)
               r=[sca,sca,sca];
           end
           %pObstacleGraph
           if (probId==6)
               r=[sca,1-sca,1-sca];
           end
        end
    end
    
end

