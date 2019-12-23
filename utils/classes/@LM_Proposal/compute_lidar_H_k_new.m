
function compute_lidar_H_k_new(obj,etimator, params,new_landmarks)
% this funcion builds the Jacobian H for the factor graphs case without
% mesaurements. It uses all the landmarks in the field of view of the lidar

spsi= sin(estimator.x_true(3));
cpsi= cos(estimator.x_true(3));

% number of expected extracted landmarks
obj.LMP_n_L_k= length(new_landmarks);

% number of expected measurements
obj.LMP_n_k= obj.LMP_n_L_k * params.m_F;

% build Jacobian
obj.LMP_H_k_new= inf * ones( obj.LMP_n_k , params.m );
for i= 1:obj.n_L_k
    % Indexes
    indz= 2*i + (-1:0);
    
    dx= new_landmarks(new_landmarks(i), 1) - obj.x_true(1);
    dy= new_landmarks(new_landmarks(i), 2) - obj.x_true(2);
    
    % Jacobian -- H
    obj.H_k_new(indz,1)= [-cpsi; spsi];
    obj.H_k_new(indz,2)= [-spsi; -cpsi];
    obj.H_k_new(indz,params.ind_yaw)= [-dx * spsi + dy * cpsi;
                                   -dx * cpsi - dy * spsi];
                               
end

end