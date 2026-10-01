# Testing strategy

Run `python -m unittest discover -s tests -v` after installing the package.
Analytic tests cover identities, limiting cases, monotonic trends, signs,
mass/rating scaling, numeric arrays and symbolic compatibility. Every component
is evaluated with Opti variables in an integration solve. Test hover against
closed-form momentum theory and positive forward flight against AeroSandbox's
native helper. Future maps require node recovery, interpolation, boundaries,
an explicit out-of-domain policy and reference-point tests.
Passing these tests establishes algebraic consistency, not flight validation.
