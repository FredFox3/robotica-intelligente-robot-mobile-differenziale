function results = run_tests(selection)
%RUN_TESTS Run all tests or select 'unit' or 'smoke'.
%   Failed or incomplete tests raise an error after displaying the results.

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
