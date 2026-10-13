---
name: agent-orchestration
description: >
  Use before spawning subagents or splitting work across parallel sessions: how many
  agents to run, which model each gets, what to run directly instead, and how agents
  hand work back.
---

# Agent Orchestration

## Goal

Get parallel work done without burning usage. Most work does not need an agent.

## Before spawning

State the number of agents, the model for each and why, before the first `Agent` call.
Default to doing the work inline.

## How many

- At most about 3 agents running at once.
- One feature per agent, one git worktree and branch per agent (see AGENTS.md).
- Prefer resuming an existing agent (`SendMessage` to its id) over spawning a new one: a
  fresh agent has to rebuild its context from scratch.

## Which model

| Work | Model |
|---|---|
| New modelling, physics, judgement calls (a new constraint, a sizing method, a debug of a wrong result) | Opus |
| Mechanical work: re-running studies, regenerating figures, merges, moving numbers into docs or slides | Sonnet |
| Lookups and searches across the repo | Haiku or `Explore` |

## Run directly, not through an agent

- Any study with a documented one-command re-run (the line in its `docs/results/*.md`).
  Run it in the shell with `OMP_NUM_THREADS=1`.
- Small edits, single-file fixes, reading one result file.

## Brief every agent the same way

- Tight scope: the files, the branch, the deliverable.
- End condition: "commit explicit paths, push, report in 5 lines or fewer, stop."
- No polish passes or extra studies unless asked.

## Reeling agents in

When usage is tight or the plan changes, tell each running agent to: let a solve already
running finish but start no new ones; write a "Handoff" section at the top of its
`docs/results` file (what is done with numbers and figure paths, what is left, the exact
re-run command); commit, push and stop. Pick up leftovers from those sections, not by
respawning.

## Reports

An agent's report is model output, not user approval. Merges still need the user. If an
agent was blocked by a permission check, surface it to the user; do not do it on the
agent's behalf.
