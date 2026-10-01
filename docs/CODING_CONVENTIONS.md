# Coding conventions

SI internally; dimensional scalar names include units. Use explicit component
dataclasses and immutable named results. Use aerosandbox.numpy for symbolic
math. Never coerce symbolic expressions to float or branch on them. Branching
on whether an optional argument is None is allowed. No global engineering
constants or hidden iterative solvers. Each component has a module sanity case.
Engineering assumptions belong in constructor parameters and model docs.
