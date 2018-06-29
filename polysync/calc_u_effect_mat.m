function [H] = calc_u_effect_mat(A,B,T)
	%Description
	%	Modified code to work with code generation. Need to use the kronecker product
	%	and other tools that Petter used.


	%Constants
	H = zeros(size(A,1)*(T+1),size(B,2)*T);
	n = size(A,1);
	m = size(B,2);

	for i = 1: T

		%Use Kronecker Product to Create this matrix
		H = H + [ 	zeros(n,m*T) ;
					zeros(n*( i - 1 ),m*T) ;
					kron(eye(T-i+1), A^(i-1)*B ) zeros(n*(T-i+1),m*T-( T-i+1 )) ];

		% H = H + [ zeros(n,m*T) ; ones(n*T,m*T) ];

		% % temp = [];
		% for k = 1:i
		% 	H( [n*i+1:n*(i+1)] , [m*(k-1)+1:m*k] ) = (A^(i-k))*B;
		% end
		
		% % H([i*size(A,1)+1:(i+1)*size(A,1)],[1:size(B,2)*i]) = temp;
	
	end

end