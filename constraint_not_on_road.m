function [cst,ceq] = constraint_not_on_road(params,x_true,x)
         if(size(x,1)==1 & size(x,2)==2)
             x = [x];
         end
         cst = [];
         px_true = x_true(1);
         py_true = x_true(2);
         for i=1:size(x,1)
             x_lm = x(i,:);
             px_lm = x_lm(1);
             py_lm = x_lm(2);
             
             dx = px_lm - px_true;
             dy = py_lm - py_true;
                       
         end
end