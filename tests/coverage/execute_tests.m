function [results, coverage] = execute_tests(test_folder, coverage_folder)
    % As in https://de.mathworks.com/help/matlab/ref/matlab.unittest.plugins.codecoverageplugin-class.html.
    import matlab.unittest.TestRunner
    import matlab.unittest.Verbosity
    import matlab.unittest.plugins.CodeCoveragePlugin
    import matlab.unittest.plugins.codecoverage.CoverageResult
    import matlab.unittest.plugins.XMLPlugin

    % Define output files.
    cobertura_output_file = 'tests/coverage/coverageReport.xml';
    html_output_folder = 'tests/coverage/htmlReport';
    junit_output_file = 'tests/coverage/junitReport.xml';

    % Create a test runner running the specified tests and creating a cobertura code coverage for the specified source
    % folders.
    runner = TestRunner.withTextOutput('OutputDetail', Verbosity.Detailed);

    format = CoverageResult;
    runner.addPlugin( ...
        CodeCoveragePlugin.forFolder( ...
        coverage_folder, ...
        IncludingSubfolders = true, ...
        Producing = format ...
    ) ...
    );

    runner.addPlugin(XMLPlugin.producingJUnitFormat(junit_output_file));

    % Run the tests generating code coverage information.
    suite = testsuite(test_folder, 'IncludeSubfolders', true);
    results = runner.run(suite);

    % Return code coverage percentage.
    test_results = format.Result;
    generateCoberturaReport(test_results, cobertura_output_file);
    generateHTMLReport(test_results, html_output_folder);
    coverageReport = fileread(cobertura_output_file);
    coverage = regexp(coverageReport, '<coverage[^<]* line-rate="([^"])*" [^<]*>', 'tokens');
    coverage = num2str(str2double(coverage{1}) * 100);

end
