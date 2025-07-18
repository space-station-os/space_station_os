## Github Branch and Pull-request/Merge Policy

### Overview
We follow a structured Git workflow inspired by Gitflow, adapted for modular, iterative development. This policy outlines how contributors interact with the repository, how release cycles are managed, and how hotfixes are handled.

---

### 1. `main` Branch: The Stable Entry Point
- The `main` branch always reflects the latest stable release.
- Tags (e.g., `v0.8.3`) are attached to commits in `main`.
- New users are encouraged to start from `main` to clone, build, and run the software without trouble.

---

### 2. Hotfixes: Critical Patches Applied Directly to `main`
- If a bug is discovered **after tagging** a release (e.g. `v0.8.3`), create a `hotfix/` branch directly from `main`.
 - Example: `hotfix/fix-config-bug`
- Apply the patch and open a PR to `main`.
- After merging, tag a new patch version:
 - Example: `v0.8.3.1`
- (Optional) Cherry-pick the fix into `v0.8.4-dev` if it also applies to future development.

---

### 3. Iterative Development: `vX.Y.Z-dev` Branches
- For each development cycle (typically 2 to 3 months), a new branch is created from `main`:
 - Example: `v0.8.4-dev`
- All new features, improvements, or refactors are contributed to this development branch via pull requests (PRs).

---

### 4. Issue-Driven Feature Branches
- Each new task must be defined in an issue (bug, feature, or refactor).
- Contributors create a new branch from `v0.8.4-dev`:
 - Branch name example: `feature/42-thermal-model`
- One PR should correspond to **one issue**.
- If a contributor needs to begin work on a second issue while the previous PR is still under review:
 - **Preferred**: Create the new feature branch from `v0.8.4-dev`.
 - **Alternative**: If the new issue depends on code in the previous PR, base the new branch on the previous feature branch.
 - (Caution) In that case, note that changes during the review may require rebasing.

---

### 5. Reviewing and Merging
- Reviewers examine PRs targeting `v0.8.4-dev`.
- Each PR should be reviewed and merged individually.
- When the iteration ends:
 - Merge `v0.8.4-dev` into `main`.
 - Create a release tag: `v0.8.4`.

---

### References
- [GitHub Flow](https://docs.github.com/en/get-started/quickstart/github-flow)
- [GitLab Flow](https://docs.gitlab.com/ee/topics/gitlab_flow.html)
- [Atlassian Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)