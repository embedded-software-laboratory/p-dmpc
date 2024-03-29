# Gitlab Badges
Badges are small boxes at the the main page of a Gitlab project providing short information on the project. Further information can be found in the [Gitlab Documentation](https://docs.gitlab.com/ee/user/project/badges.html).

The badges could also be included in the README of the project or similar. The needed phrases can be found as described in the [Gitlab Documentation](https://docs.gitlab.com/ee/user/project/badges.html#view-the-url-of-pipeline-badges).


## How to create/modify a badge
The badges of a Gitlab project are maintained via ```Settings -> General -> Badges```. Here, badges can be added and a list of the already existing badges is available with the possibility to modify or delete each badge.

Three fields have to be filled to create a badge:
- ```Name```: Each badge can be named to identify it.
- ```Link```: Here, a link can be entered which is followed if the respective badge is clicked on.
- ```Badge image URL```: The badges shown are images which are part of the git repository. Default badges are generated by Gitlab, including pipeline status and code coverage of each branch and the project's latest release. Other images, possibly generated by the CI pipeline can be used, too. Look at the [Gitlab Documentation](https://docs.gitlab.com/ee/ci/pipelines/job_artifacts.html#access-the-latest-job-artifacts) to learn about the links to job artifacts.

Both the link and the image URL can contain placeholders, see the [Gitlab Documentation](https://docs.gitlab.com/ee/user/project/badges.html#placeholders).

Further, the appearance of each badge can be modified, including the name, the badge style, its width and the image. Look at the [Gitlab Documentation](https://docs.gitlab.com/ee/user/project/badges.html#customize-badges). The ```Badge image preview``` shows how the badge would look like with the current settings.


## Available Badges

### Pipeline status
The pipeline status of a specific branch (currently ```dev```) can be displayed. The status is defined by a keyword (```pending```, ```running```, ```passed```, ```failed```, ```skipped```, ```manual```, ```canceled```, ```unknown```) and a color. Further information can be found in the [Gitlab Documentation](https://docs.gitlab.com/ee/user/project/badges.html#pipeline-status-badges).

## Test coverage
A test coverage badge shows the percentage of code tested in the CI process of a specific branch (currently ```code_coverage```). Further, it is visualized by a color. To obtain the value, a coverage report has to be uploaded during the CI process. Currently, the coverage values of different tests are averaged to obtain the overall code coverage. That's why all tests for one folder have to be executed in the same CI job. Check for any changes in this field in [this Gitlab issue](https://gitlab.com/gitlab-org/gitlab/-/issues/15399). Further information can be found in the [Gitlab Documentation](https://docs.gitlab.com/ee/user/project/badges.html#test-coverage-report-badges).

## Latest release
The name of the latest release can be shown (currently not). Sadly, the width of the badge's value is not computed correctly at the moment. Check [this Gitlab merge request](https://gitlab.com/gitlab-org/gitlab/-/merge_requests/86442) if something in this field has changed. Further information on the latest release badge can be found in the [Gitlab Documentation](https://docs.gitlab.com/ee/user/project/badges.html#latest-release-badges).

## Custom Badges
Custom badges are also possible, e.g. for code quality. Check [this article](https://medium.com/@iffi33/adding-custom-badges-to-gitlab-a9af8e3f3569) with a description how to create custom badges. This could be for example used in combination with [Matlab's checkcode function](https://de.mathworks.com/help/matlab/ref/checkcode.html) to generate a custom badge for code coverage.