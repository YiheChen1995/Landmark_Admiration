
function cst = constraint_in_detection_range(params,x_true,x)
         cst = [];
         px_true = x_true(1);
         py_true = x_true(2);
         for i=1:size(x,1)
             x_lm = x(i,:);
             px_lm = x_lm(1);
             py_lm = x_lm(2);
             tmp_cst = sqrt((px_lm - px_true)^2+(py_lm-py_true)^2);
             cst = [cst;tmp_cst-params.detection_range];
         end
end
