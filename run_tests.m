function results = run_tests(selection)
%RUN_TESTS Run the selected suite in MATLAB Online, returning actual results.
%   RUN_TESTS() or RUN_TESTS('all') runs all currently available tests.
%   RUN_TESTS('unit') and RUN_TESTS('smoke') select a test folder.
%   Paths are restored on success or failure. A failing/incomplete suite
%   raises an error after displaying results; output assignment then aborts.

    if nargin == 0
        selection = 'all';
    end
    if ~ischar(selection) || ~isrow(selection) || ...
            ~any(strcmp(selection, {'all', 'unit', 'smoke'}))
        error('RobotProject:InvalidTestSelection', ...
            'Test selection must be ''all'', ''unit'' or ''smoke''.');
    end

    projectRoot = fileparts(mfilename('fullpath'));
    originalPath = path;
    restorePath = onCleanup(@() path(originalPath)); %#ok<NASGU>
    addpath(projectRoot);
    addpath(fullfile(projectRoot, 'src'));
    testFolder = fullfile(projectRoot, 'tests');
    if ~strcmp(selection, 'all')
        testFolder = fullfile(testFolder, selection);
    end
    suite = testsuite(testFolder, 'IncludeSubfolders', true);
    if isempty(suite)
        error('RobotProject:NoTests', 'No MATLAB tests were discovered.');
    end

    results = run(suite);
    disp(results);
    if any([results.Failed]) || any([results.Incomplete])
        error('RobotProject:TestsNotPassed', ...
            'One or more tests failed or were incomplete.');
    end
end
