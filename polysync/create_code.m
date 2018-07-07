clear('conf');

disp(' ')
disp('==========================================')
disp('Select the code that you wish to generate:')
disp('1: polysync_controller.m')
disp('   test_turnsignal.m')
disp('   test_shift.m')
disp('2: ')

disp(' ')
disp('Input the number that corresponds to the code you wish to compile:')

compile_set = input('')

conf = coder.config('exe');
conf.TargetLang = 'c++';
conf.GenerateExampleMain = 'GenerateCodeAndCompile';

switch compile_set
	case 1
		codegen -config conf polysync_controller.m
		codegen -config conf test_turnsignal.m
		codegen -config conf test_shift.m

	case 2
		%cd map_validation1
		addpath(genpath('../'))
		codegen -config conf map_validation1/test1_controller_acc_step_input.m
		codegen -config conf map_validation1/test1_controller_from_standing.m

	otherwise
		error(['Unexpected input: ' num2str(compile_set) '.' ])

end

