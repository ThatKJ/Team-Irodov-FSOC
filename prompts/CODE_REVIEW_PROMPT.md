# Aerospace C++ Code Review Prompt

Review the diff as if it were a GNC simulation component that another engineer must trust.

Prioritize:
- coordinate/sign/unit correctness,
- closed-loop data integrity,
- ground-truth leakage,
- actuator constraint correctness,
- timestep and numerical edge cases,
- ownership/lifetime safety,
- module coupling,
- test coverage,
- deterministic behavior,
- unnecessary complexity.

Return blockers first, with file/function references and a concrete correction.
