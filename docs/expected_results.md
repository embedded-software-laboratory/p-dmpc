# How to replace expected results

1. Execute `run(systemtests) ` and ensure that the ExperimentResult objects are saved
2. Execute `replace_expected_results()` to copy all of the `.mat` files in `results/*` to `tests/systemtests/expected_results`
3. Add new files to matlab project path
4. Commit new files; if the new files cannot be added to git, remove `*.mat` flag from gitignore to add them and add it back in before committing
5. Make sure that the committed files are listed as lfs files using `git lfs ls-files --all`, otherwise adjust the `.gitattributes` file accordingly ([lfs reference](https://docs.gitlab.com/ee/topics/git/lfs/#using-git-lfs))
