# me553-RD
homework repo for ME553 robot dynamics - 2024 spring (Hyungho Chris Choi)

Basic manipulation references this [gist](https://gist.github.com/injoonH/84f05d64b847cc18b9aeb597362fb512) by injoonH:
(and this [repo](https://github.com/iamchoking/me491-LBC/tree/main) (previous homework for learning based control))

upstream (skeleton code repo): [GitHub](https://github.com/jhwangbo/ME553_2024)

Create / Cloning repo (+with upstream)
```sh
# (create a repo (named <me553-RD> in web)
git clone git@github.com:iamchoking/me553-RD
git remote add upstream git@github.com:jhwangbo/ME553_2024.git
```
Manipulating Repo
```sh
# basic groundwork for your new repo
git remote add origin git@github.com:<github-id>/<repo>.git
git checkout main
git push -u origin main

# get a branch from upstream
git fetch upstream
git checkout <branch>
git push -u origin <branch>

# sync an (existing) branch from upstream
git checkout <branch>
# (do NOT do "origin/<branch>")
git merge upstream/<branch>
git push -u origin <branch>
# may need to add options like -f

# if you have your "homework" branch that needs to incorporate this as a rabase, do:
git checkout <branch2>
git rebase <branch>
git push -u origin <branch2>

# adding your homework branch
# (in web, create your homework branch from skeleton branch)
git fetch origin
git checkout <mybranch>
git push origin <mybranch>

# Check remotes
git remote -v

# (?) Get updates from the upstream
git fetch upstream
git merge upstream/main # Or other branches
```
