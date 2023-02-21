function [results, coverage] = test_with_code_coverage(test_files, source_folders)
    % As in https://de.mathworks.com/help/matlab/ref/matlab.unittest.plugins.codecoverageplugin-class.html.
    import matlab.unittest.plugins.CodeCoveragePlugin
    import matlab.unittest.plugins.codecoverage.CoberturaFormat

    % Define folders.
    coverage_output_file = 'tests/coverage/coverageReport.xml';

    % Create a test runner running the specified tests and creating a cobertura code coverage for the specified source
    % folders.
    runner = testrunner("textoutput");
    reportFormat = CoberturaFormat(coverage_output_file);
    p = CodeCoveragePlugin.forFolder(source_folders, "IncludingSubfolders", true, "Producing", reportFormat);
    runner.addPlugin(p);

    % Run the tests generating code coverage information.
    suite = testsuite(test_files);
    results = runner.run(suite);

    % Return code coverage percentage.
    coverageReport = fileread(coverage_output_file);
    coverage = regexp(coverageReport, '<coverage[^<]* line-rate="([^"])*" [^<]*>', 'tokens');
    coverage = num2str(str2double(coverage{1}) * 100);
end