function tests = test_project_setup
%TEST_PROJECT_SETUP Check project setup without running a robot simulation.
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    testCase.TestData.projectRoot = projectRoot;
    testCase.TestData.originalPath = path;
    addpath(projectRoot);
    addpath(fullfile(projectRoot, 'src'));
end

function teardownOnce(testCase)
    path(testCase.TestData.originalPath);
end

function testEntryPointFromDifferentFolder(testCase)
    previousFolder = pwd;
    restoreFolder = onCleanup(@() cd(previousFolder)); %#ok<NASGU>
    cd(fullfile(testCase.TestData.projectRoot, 'docs'));

    project = main();
    verifyEqual(testCase, project.executionEnvironment, 'MATLAB desktop / MATLAB Online');
    verifyEqual(testCase, project.status, 'visualization_analysis_implemented');
    verifyEqual(testCase, project.executionValidation, 'pending_matlab_online');
    verifyFalse(testCase, project.simulationExecuted);
    verifyEqual(testCase, project.config.robot.r, 0.05);
    verifyEqual(testCase, project.config.robot.d, 0.30);
end

function testRequiredProjectFilesExist(testCase)
    requiredFiles = {'AGENTS.md', 'README.md', '.gitignore', 'main.m', ...
        'run_tests.m', 'TRACCIA_Robot_Mobile_Differenziale.md', ...
        fullfile('docs', '02_Modello_Cinematico.md'), ...
        fullfile('docs', 'Checklist_Requisiti.md')};

    for index = 1:numel(requiredFiles)
        verifyTrue(testCase, isfile(fullfile( ...
            testCase.TestData.projectRoot, requiredFiles{index})), ...
            ['Missing required project file: ', requiredFiles{index}]);
    end
end

function testInvalidTestSelection(testCase)
    verifyError(testCase, @() run_tests('unknown'), ...
        'RobotProject:InvalidTestSelection');
end
