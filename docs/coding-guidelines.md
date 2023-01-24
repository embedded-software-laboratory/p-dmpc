# General
- Code Style
    - never abbreviate (parallel, not: parl)
    - don't put types in names (not: int_houses, i for Interface e.g. IHouse)
    - put units in variable names (delay_seconds)
    - avoid nesting, e.g. with guard clauses
- Variable naming conventions
    - Classnames are `CamelCase`
    - Everything else is `snake_case`
- Branch naming conventions
    - use slashes to separate folders
    - use hyphens to separate words, no underscores
    - format: author/group/issue-nr/topic, e.g. dg/fix/24/code-structure
        - author: initials
        - group:
            - wip: Works in progress; stuff I know won't be finished soon
            - feat: Feature I'm adding or expanding
            - fix: Bug fix
        - issue-nr: every branch should be referring to an issue
        - topic: short description
- Merging
    - rebase a branch that you alone work on to the target branch before a merge
    - use keywords in merge requests, e.g. "closes #issue-number", "relates to #issue-number"

# MATLAB
- Use Visual Studio Code, team's workspace settings in "/.vscode"
- Use formatter and spell checker, see recommendations
- End each `function` with an `end`
- use `arguments` block for function input checks and easier-to-read code without documentation

# C++
- private members start with underscore, i.e., _member
