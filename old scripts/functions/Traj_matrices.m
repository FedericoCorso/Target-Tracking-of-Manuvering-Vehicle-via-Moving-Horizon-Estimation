function [Lambda_y,Gamma_y,Lambda_z,Gamma_z]=Traj_matrices(M,A,B,C,D)
% this function builds the matrices needed to implement the MHE algorithm
% it takes as input the system matrices and gives as output the matrices
% associated to the linear constraints of the system.

% REMARK: this formulation is valid only if we have linear constraints

nz      =   size(A,1);
nu      =   size(B,2);
ny      =   size(C,1);


Lambda_y    =     zeros(ny*(M+1),nz);
Gamma_y     =     zeros(ny*(M+1),nu*M);
Lambda_z    =     zeros(nz*(M+1),nz);
Gamma_z     =     zeros(nz*(M+1),nu*M);


for ind = 1:M+1
   Lambda_y((ind-1)*ny+1:ind*ny,:)          =   C*A^(ind-1);
   Lambda_z((ind-1)*nz+1:ind*nz,:)          =   A^(ind-1);
   for ind2 =   1:ind-1
       Gamma_z((ind-1)*nz+1:ind*nz,(ind2-1)*nu+1:ind2*nu)=A^(ind-ind2-1)*B;
       if ind2==ind-1
           Gamma_y((ind-1)*ny+1:ind*ny,(ind2-1)*nu+1:ind2*nu)=C*B+D;
       else
           Gamma_y((ind-1)*ny+1:ind*ny,(ind2-1)*nu+1:ind2*nu)=C*A^(ind-ind2-1)*B;
       end
   end
end