function [G] = calc_w_effect_mat(A,T)

	%Constants
	G = zeros(size(A,1)*(T+1),size(A,2)*T);

	for i = 1 : T

		temp = zeros(size(A,1),size(A,2)*T);
		for k = 1:i
			temp(:,(k-1)*size(A,2)+[1:size(A,2)]) = [A^(i-k)];
		end
		
		G([i*size(A,1)+1:(i+1)*size(A,1)],:) = temp;
	
	end

end