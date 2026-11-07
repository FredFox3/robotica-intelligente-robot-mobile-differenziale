function tests = test_project_setup
%TEST_PROJECT_SETUP Check required files, paths and the setup entry point.
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
    cd(fullfile(testCase.TestData.projectRoot, 'tests'));

    project = main();
    verifyEqual(testCase, project.executionEnvironment, 'MATLAB desktop / MATLAB Online');
    verifyFalse(testCase, project.simulationExecuted);
    verifyEqual(testCase, project.config.robot.r, 0.05);
    verifyEqual(testCase, project.config.robot.d, 0.30);
end

function testRequiredProjectFilesExist(testCase)
    requiredFiles = {'README.md', '.gitignore', 'main.m', 'run_tests.m'};

    for index = 1:numel(requiredFiles)
        verifyTrue(testCase, isfile(fullfile( ...
            testCase.TestData.projectRoot, requiredFiles{index})), ...
            ['Missing required project file: ', requiredFiles{index}]);
    end

    requiredFolders = {fullfile('src', '+robot'), ...
        fullfile('tests', 'unit'), fullfile('tests', 'smoke'), 'results'};
    for index = 1:numel(requiredFolders)
        verifyTrue(testCase, isfolder(fullfile( ...
            testCase.TestData.projectRoot, requiredFolders{index})), ...
            ['Missing required project folder: ', requiredFolders{index}]);
    end
end

function testInvalidTestSelection(testCase)
    verifyError(testCase, @() run_tests('unknown'), ...
        'RobotProject:InvalidTestSelection');
end
