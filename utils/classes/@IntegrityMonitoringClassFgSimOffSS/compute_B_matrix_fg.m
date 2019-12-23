
function compute_B_matrix_fg(obj, i, m_F)

%initialize faulted msmts indices vector
ind_faulted_msmts= zeros( m_F , 1 );

ind_faulted_msmts( 1: m_F ) = obj.LM_abs_msmt_ind(:,i);


% build the fault-free msmts extraction matrix
obj.B_j = zeros( obj.LM_n_total-obj.m-m_F , obj.LM_n_total );

tmp=1;

for j = (obj.m+1):obj.LM_n_total

    if sum(ind_faulted_msmts==j) == 1

        continue;

    else

        obj.B_j(tmp,j) = 1;

        tmp=tmp+1;

    end

end
    
end
