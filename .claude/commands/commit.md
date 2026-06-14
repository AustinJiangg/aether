---
description: Stage changes, create a Conventional Commits message, and push
---

Create a git commit following the project's commit conventions (see CLAUDE.md),
then push it to the personal remote.

Steps:
1. Run `git status` and `git diff` (staged and unstaged) to review what changed.
2. Group the changes into one logical commit. If unrelated changes are mixed,
   suggest splitting them into separate commits.
3. Write a commit message in **Conventional Commits** format:
   - `<type>: <short imperative summary>` on the first line.
   - type is one of: feat, fix, docs, refactor, chore, test, perf.
   - summary: lowercase, imperative mood, no trailing period, max 50 chars.
   - Add a body (wrapped at ~72 chars) only if the change needs explanation of
     what/why; separate it from the summary with a blank line.
4. Stage the relevant files and create the commit.
5. Push to the remote with `git push`. This repo is a personal project, so
   pushing on every commit is intended.
   - If there is no upstream set yet, use `git push -u origin <current-branch>`.
   - If the push fails (no network, no remote, auth error), report the error
     clearly but keep the local commit — do NOT amend or reset it.
6. Show the resulting `git log -1` and confirm the push succeeded.

All commit text must be in English.
