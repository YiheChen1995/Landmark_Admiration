
function compute_B_matrix_fg(obj,im, i, m_F)

%initialize faulted msmts indices vector
ind_faulted_msmts= zeros( length(i)*m_F , 1 );

for j = 1 : length( i )
    
    ind_faulted_msmts( m_F*(j-1)+1: m_F*j ) = im.abs_msmt_ind(:,i(j));
    
end


% build the fault-free msmts extraction matrix
obj.LMP_B_j = zeros( im.n_total-im.m-(m_F*length(i)) , im.n_total );

tmp=1;

for j = (im.m+1):im.n_total

    if sum(ind_faulted_msmts==j) == 1

        continue;

    else

        obj.LMP_B_j(tmp,j) = 1;

        tmp=tmp+1;

    end

end
    
end
