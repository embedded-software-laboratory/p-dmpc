# Credentials to log into the Gitlab container registry can be set in the variables GITLAB_USERNAME and GITLAB_ACCESS_TOKEN.
# To generate such a personal access token see https://docs.gitlab.com/ee/user/profile/personal_access_tokens.html#create-a-personal-access-token.
# The access token needs to have the scopes read_registry and write_registry enabled.
#
# After filling in the values for username and access_token, please run
#   $ git update-index --assume-unchanged <path>/login_credentials.sh
# such that your private data is not (by accident) added to the git history.
GITLAB_USERNAME=<username>
GITLAB_ACCESS_TOKEN=<access_token>