# Continuous integration

Status: COMPLETED 2026-10-02 (Tier 11b of the 2026-10-02 roadmap; the author asked for the
review's gaps to be broken into tiers and implemented).

## Goal and scope

Tier 11b.

- `.github/workflows/tests.yml`: on every push and pull request, install with
  `uv sync --locked` (Python from `.python-version`) and run
  `uv run python -m unittest discover -s tests`.
- `.github/workflows/notebooks.yml`: manual (`workflow_dispatch`). Executes
  every verification notebook headless (Agg backend) without committing
  outputs, and uploads the executed notebooks as an artifact.
- README: a CI badge and a section.

Out of scope: lint and type checks (no concrete need yet), notebooks on every
push (too slow, about 10 min), and publishing artifacts.

## Assumptions

- GitHub-hosted `ubuntu-latest` runners; `astral-sh/setup-uv` action.
- Tests that need user-supplied local data (`data/engines/`) skip when it is
  absent. They already do, via `skipUnless`.

## Tests and acceptance

- A fresh clone with `uv sync --locked` passes the full suite locally,
  standing in for the runner, because no `gh` CLI is available to watch the
  hosted run. The suite must not depend on untracked files.
- Both workflow files are valid YAML with the expected triggers.
- The first push triggers the hosted run; its result is reported to the author
  as unverified until the author sees it.

## Progress and decisions

- 2026-10-02: Plan written.
- 2026-10-02: Implemented.
  - Local stand-in for the runner: a fresh clone with `uv sync --locked`
    passed 242 tests in 8 s, with 1 skip (the raw deck absent, as intended).
  - The local network needed `--system-certs` (a machine TLS proxy); the
    hosted runner does not.
  - The hosted run's result is unverified until it is seen on GitHub.

## Deferred

Branch protection (a repository setting the author controls) and caching
tuning.
