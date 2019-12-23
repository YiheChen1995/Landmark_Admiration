classdef LM_Proposal < handle
         
         properties  
             
          %------------/module_&_debugging_parameters-------------------------
           DBG_new_landmarks  %[Debug Only] a structure that contains new landmarks  
           CTRL_MAX_Landmarks= 5  %[Module Parameter] controls the max number of landmarks to be added at each epoch, 5 by default
           
           
           LMP_n_L_k % number of expected extracted landmarks
           LMP_n_k   % number of expected measurements
           LMP_H_k   % Jacobian
           LMP_H_k_new   % Jacobian with new lms added
           LMP_B_j   % non-faulted msmts extraction_matrix
           LMP_V     % Just a normal-looking measurement cov matrix
           LMP_P_F_M
           LMP_n_M
           LMP_n_L_M  
           M
           LMP_n_total % number of msmts
           LMP_m_M %states to estimate
           lmp_alpha % the updated_extract state of interest vector
           LMP_abs_msmt_ind
           LMP_PX_M
           LMP_inds_H
           LMP_sigma_hat
           LMP_Lambda
           LMP_p_hmi
           LMP_n_H
           LMP_P_H
           A_aug
           C_req
           LM_abs_msmt_ind
           m
           B_j

         end
         
         methods
             function obj = LM_Proposal(im,params,estimator,debug)

              if(debug == 1)
                 p_phmi = calculate_phmi_pseudo(obj,im,params,estimator,obj.DBG_new_landmarks)
              else
                %%  ---------under costruction------------
                 cstn_num = 1
                 CTRL_MAX_Landmarks = cstn_num
                 for lm_i = 1:CTRL_MAX_Landmarks % try different landmark number until it reaches the max
                    % construct the initial condition
                     new_landmarks_0 = [];
                     for x0_i = 1:lm_i
                         new_landmarks_0 = [new_landmarks_0; estimator.x_true(1),estimator.x_true(2)]
                     end
                    % setup constraints
                     %tmp_loc = [0,0];
                     %tmp_lm = [0,0;1,1;5,5];
                     %cst = constraint_in_detection_range(tmp_loc,tmp_lm)
                     
                    %opt
                     [new_landmark_return,ps_phmi_out] = fmincon(@(new_landmarks) calculate_phmi_pseudo(obj,im,params,estimator,new_landmarks),new_landmarks_0);

                 end
                 a=0;
                 
              end
                 
             end
          %================================================================
          %================================================================
          function lmp_p_hmi_op = calculate_phmi_pseudo(obj,im,params,estimator,new_landmarks)
                 obj.M= im.M; 
                 obj.C_req= params.continuity_requirement;
                 obj.LMP_abs_msmt_ind = im.abs_msmt_ind;
                 obj.m= im.m;
                 obj.LMP_n_M= estimator.n_k + size(new_landmarks,1)*params.m_F + sum( im.n_ph(1:obj.M - 1) );
                  % number of landmarks over the horizon
                 obj.LMP_n_L_M= obj.LMP_n_M / params.m_F; 
                  % compute extraction vector
                 lmp_alpha= obj.build_state_of_interest_extraction_matrix(params, estimator.x_true);
                  % total number of msmts (prior + relative + abs)
                 obj.LMP_n_total= obj.LMP_n_M + (obj.M + 1) * (params.m);
                  % number of states to estimate
                 % obj.m_M= (obj.M + 1) * params.m;
                 obj.A_aug = build_augmented_whiten_jacobian_A(obj,im,params,estimator,new_landmarks);
                 ind= size(im.A,1);
                 for i=1:size(new_landmarks,1)
                     tmp= [ind+1;ind+2];
                     ind= ind+2;
                     obj.LMP_abs_msmt_ind= [obj.LMP_abs_msmt_ind,tmp];
                 end
                 obj.LMP_Lambda= obj.A_aug'*obj.A_aug; % provides the lambda
                 obj.LMP_PX_M= inv(obj.LMP_Lambda);
                 obj.LMP_P_F_M= ones(obj.LMP_n_L_M, 1) * params.P_UA;
                 obj.LMP_n_H= obj.LMP_n_L_M;
                 % initialization of p_hmi
                 lmp_p_hmi_op=0;
                 if obj.LMP_n_M < params.m + params.m_F
                      % if we don't have enough landmarks --> P(HMI)= 1
                     lmp_p_hmi_op= 1;
                 else
                      % standard deviation in the state of interest
                     obj.LMP_sigma_hat= sqrt( (lmp_alpha' / obj.LMP_Lambda) * lmp_alpha );
                     
                     for i= 0:obj.LMP_n_H   
                         % compute P(HMI | H) for the worst-case fault
                         LMP_p_hmi_H= obj.compute_p_hmi_H(lmp_alpha, i, params);
 
                         % Add P(HMI | H) to the integrity risk
                         if i == 0
                             lmp_p_hmi_op= lmp_p_hmi_op + LMP_p_hmi_H * prod( 1 - obj.LMP_P_F_M );
                         else
                             lmp_p_hmi_op= lmp_p_hmi_op + LMP_p_hmi_H * params.P_UA;
                         end
                     end
                 end 
          
          end
          %----------------------------------------------------------------
          function A_aug = build_augmented_whiten_jacobian_A(obj,im,params,estimator,new_landmarks)
                 num_lm_prp = size(new_landmarks,1);
                 compute_lidar_H_k_new(obj,estimator, params, new_landmarks);
                 A_old = im.A;
                 obj.LMP_V = kron( eye(num_lm_prp) , params.sqrt_inv_R_lidar );
                 A_new = [zeros(size(new_landmarks,1)*params.m_F,size(A_old,2)-obj.m),obj.LMP_V*obj.LMP_H_k_new];
                 A_aug = [A_old;A_new];
          end
         end
        
end