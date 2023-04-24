# Reduce Gitlab Repository Size
reference: https://docs.gitlab.com/ee/user/project/repository/reducing_the_repo_size_using_git.html

**In general it is always a good idea to first test the process on a duplicated repository (Export project and import as a new one).**

## Pre-Requisites
- Install one of the following tools to filter large files from the history of a git repository:
  - [BFG Repo-Cleaner](https://rtyley.github.io/bfg-repo-cleaner/): just one java file to download, needs Git and Java installed, **recommended** as faster and more reliable than git-filter-repo
  - [git-filter-repo](https://github.com/newren/git-filter-repo/blob/main/INSTALL.md): only a text file to download, needs Git and Python 3 installed, recommended by the Gitlab documentation
  - [git-sizer](https://github.com/github/git-sizer#getting-started): needs only Git installed, but not used in Gitlab documentation
- You must have a password set for the Gitlab instance.


## 1. Export the Gitlab Project
reference: https://docs.gitlab.com/ee/user/project/settings/import_export.html#export-a-project-and-its-data
1. In the project on the left sidebar, select ```Settings > General```, expand ```Advanced``` and select ```Export project```.
2. After, the export is generated (can take some minutes), you receive an email with a link to download the file.
3. Download the export from the link in the email.
4. Decompress the export:

    ```tar xzf project-backup.tar.gz```


## 2. Mirror the git repository
including the full history

1. Clone a fresh copy of the repository from the ```project.bundle``` file in the [backup folder](#1-export-the-gitlab-project) with all refs (```--mirror```) and everything copied as is (```--bare```):
    
    ```git clone --bare --mirror /path/to/project.bundle```
2. Move into the newly cloned repository:
   
   ```cd project.git```


## 3. Optional: Analyze the git repository's history for large files

* Using **git-filter-repo**:

    ```(python3) path/to/git-filter-repo --analyze```

    ```head filter-repo/analysis/*-{all,deleted}-sizes.txt```


## 4. Purge large files from history
to remove references to large commits, merge requests, pipelines and environments

* Using **git-filter-repo**:

  reference: https://docs.gitlab.com/ee/user/project/repository/reducing_the_repo_size_using_git.html#purge-files-from-repository-history

   - purging specific files: ```(python3) path/to/git-filter-repo --path path/to/file --invert-paths```
   - purging files larger than a specific size: ```(python3) path/to/git-filter-repo --strip-blobs-bigger-than 10M```

  Afterwards, store the commit map:

    ```cp filter-repo/commit-map ./_filter_repo_commit_map_$(date +%s)```

* Using **BFG Repo-Cleaner**:

  reference: https://rtyley.github.io/bfg-repo-cleaner/

  - purging specific file: ```java -jar path/to/bfg.jar --delete-files path/to/file```
  - purging files larger than a specific size: ```java -jar path/to/bfg.jar --strip-blobs-bigger-than 10M```
  - purging the N largest files: ```java -jar path/to/bfg.jar --strip-biggest-blobs N```

  Afterwards, really delete the unwanted data with:

  ```git reflog expire --expire=now --all && git gc --prune=now --aggressive```

  and store the commit map:

  ```cp ../project.git.bfg-report/<data>/<time>/object-id-map.old-new.txt ./bfg_commit_map_$(data +%s)```


## 5. Push the changes to the remote repository

1. Add the URL of the remote repository:

    ```git remote set-url origin https://git-ce.rwth-aachen.de/cpm/coincar/software/graph_based_planning.git```
2. Unprotect [protected branches](https://docs.gitlab.com/ee/user/project/protected_tags.html) and [tags](https://docs.gitlab.com/ee/user/project/protected_tags.html) (remember which ones to [restore](#4-reprotect-protected-branches-and-tags) them later):

    In the project on the left sidebar, select ```Settings > Repository```, expand ```Protected branches/tags``` and unprotect all branches/tags.
3. Push all changes overwriting the remote repository:
    - ```git push origin --force 'refs/heads/*'```
    - ```git push origin --force 'refs/tags/*'```
    - ```git push origin --force 'refs/replace/*'```

    If this fails with the error ```fatal: --mirror can’t be combined with refspecs```, remove the git mirror option:

    ```git config --unset remote.origin.mirror```

4. Wait at least 30 minutes. Then proceed with [cleaning up](#3-cleanup-repository).


## 6. Cleanup Repository
reference: https://docs.gitlab.com/ee/user/project/repository/reducing_the_repo_size_using_git.html#repository-cleanup

1. In the project on the left sidebar, select ```Settings > Repository``` and expand ```Repository cleanup```.
2. Upload the ```commit-map``` file created by git-filter-repo or bgf.
3. Select ```Start cleanup.```
4. When completed, you get an email with the recalculated repository size.
5. To see the reduction in storage utilization in the project statics, you might have to wait 5-10 minutes, as those are cached.


## 7. Reprotect protected branches and tags
references: https://docs.gitlab.com/ee/user/project/protected_branches.html and https://docs.gitlab.com/ee/user/project/protected_tags.html

1. In the project on the left sidebar, select ```Settings > Repository```, expand ```Protected branches/tags``` and protect all branches/tags with the setting they were protected before.
