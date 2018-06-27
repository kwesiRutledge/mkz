function [G] = calc_w_effect_mat(A,T)
	%Description:
	%	Function changed slightly.
	%	Matrix Equations Modified to Allow for Code Generation

	%Constants
	G = zeros(size(A,1)*(T+1),size(A,2)*T);
	n = size(A,1);

	for i = 1 : T

		% temp = [];
		for k = 1:i
			G( n*(i-1)+1:n*i , n*(k-1)+1:n*k ) = A^(i-k);
		end
		
		% G([i*size(A,1)+1:(i+1)*size(A,1)],[1:size(A,2)*i]) = temp;
	
	end

end
