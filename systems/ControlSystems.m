classdef ControlSystems < Systems
    
    properties
        u;
        f;
    end
    
    methods
        function obj = ControlSystems()
        end
        
        function obj = f(x,u,t)
        end
        
        function obj = dx(t,x)
            dx = obj.f(x,obj.u,t);
        end
        
        
    end
end

