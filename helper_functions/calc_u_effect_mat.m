function [H] = calc_u_effect_mat(A,B,T)

	%Constants
	H = zeros(size(A,1)*(T+1),size(B,2)*T);

	for i = 1: T

		temp = zeros(size(A,1),size(B,2)*T);
		for k = 1:i
			temp(:,(k-1)*size(B,2)+[1:size(B,2)]) = [A^(i-k)*B];
		end
		
		H([i*size(A,1)+1:(i+1)*size(A,1)],:) = temp;
	
	end

end