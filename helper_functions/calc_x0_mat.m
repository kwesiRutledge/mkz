function [x0_mat] = calc_x0_mat(A,x0,T)

	%Constants

	%Creating matrix.
	x0_mat = zeros(size(x0,1)*(T+1),1);

	for i = 0 : T
		x0_mat(i*size(x0,1)+[1:size(x0,1)],1) = [(A^i)* x0];
	end
end