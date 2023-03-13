function [results, coverage] = execute_tests(test_folder, compute_coverage, coverage_folder)
    % As in https://de.mathworks.com/help/matlab/ref/matlab.unittest.plugins.codecoverageplugin-class.html.
    import matlab.unittest.TestRunner
    import matlab.unittest.Verbosity
    import matlab.unittest.plugins.CodeCoveragePlugin
    import matlab.unittest.plugins.XMLPlugin
    import matlab.unittest.plugins.codecoverage.CoberturaFormat

    % Define output files.
    cobertura_output_file = 'tests/coverage/coverageReport.xml';
    junit_output_file = 'tests/coverage/junitReport.xml';

    % Create a test runner running the specified tests and creating a cobertura code coverage for the specified source
    % folders.
    runner = TestRunner.withTextOutput('OutputDetail',Verbosity.Detailed);
    if compute_coverage
        runner.addPlugin(CodeCoveragePlugin.forFolder(coverage_folder, "IncludingSubfolders", true, "Producing", CoberturaFormat(cobertura_output_file)));
    end
    runner.addPlugin(XMLPlugin.producingJUnitFormat(junit_output_file));

    % Run the tests generating code coverage information.
    suite = testsuite(test_folder, 'IncludeSubfolders', true);
    results = runner.run(suite);

    % Return code coverage percentage.
    if compute_coverage
        coverageReport = fileread(cobertura_output_file);
        coverage = regexp(coverageReport, '<coverage[^<]* line-rate="([^"])*" [^<]*>', 'tokens');
        coverage = num2str(str2double(coverage{1}) * 100);
    end
end