# General
- Code Style
    - don't abbreviate (parallel, not: parl)
    - don't put types in names (not: int_houses, i for Interface e.g. IHouse)
    - put units in variable names (delay_seconds)
    - avoid nesting, e.g., with guard clauses (`if (~necessary) continue; end`)
- Code naming conventions
    - Classnames are `CamelCase`
    - Everything else is `snake_case`
    - Interfaces and implementations have a unique name (no `IClass`, no `InterfaceClass`, no `ClassImpl` ...)
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
- Concepts
    - use object-oriented programming when reasonable
    - aim for visible input-output behavior of functions rather than obscure object state changes
    - write functions with similar granularity
    (low-level ([example](https://git-ce.rwth-aachen.de/cpm/coincar/software/graph_based_planning/-/blob/3d051fdc54efd4d3980ee5a86120796544bcd73b/hlc/controller/@HighLevelController/HighLevelController.m#L135)) or
    high-level ([example](https://git-ce.rwth-aachen.de/cpm/coincar/software/graph_based_planning/-/blob/3d051fdc54efd4d3980ee5a86120796544bcd73b/hlc/controller/@HighLevelController/HighLevelController.m#L161)),
    not a mix of the two ([example](https://git-ce.rwth-aachen.de/cpm/coincar/software/graph_based_planning/-/blob/3d051fdc54efd4d3980ee5a86120796544bcd73b/hlc/controller/@HighLevelController/HighLevelController.m#L183)))

# MATLAB
- Use Visual Studio Code, team's workspace settings in "/.vscode"
- Use formatter and spell checker, see recommendations
- End each `function` with an `end`
- use `arguments` block for function input checks and easier-to-read code without documentation
- use `Name = Value` style instead of `'Name', Value` for name-value pairs
- Optional function parameter struct is named `optional`

# C++
- private members start with underscore, i.e., _member
