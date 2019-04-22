   classdef Systems
       
       properties
           y;
           t;
       end
       
       methods
           
           function obj = Systems()
           end
           
           function obj = dx(t,x)
           end
           
           function obj = simulate(x_0,t_eval)
               t_span = [t_eval(1), t_eval(end)];
               opts = odeset('RelTol',1e-6, 'AbsTol',1e-6);
               [obj.t,obj.y] = ode45(obj.dx, t_span,x_0, opts);
           end
           
       end
   end