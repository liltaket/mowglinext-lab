# Lab collaboration workflow

## Roles

- Use this fork for field observations, reversible tools, build notes, and
  work-in-progress fixes.
- Keep `origin` pointed at `liltaket/mowglinext-lab`.
- Keep `upstream` pointed at `mowglinext/mowglinext` when preparing a clean
  upstream contribution.
- Work on short-lived branches; use a draft pull request for shared review.

## Live mower safety rules

1. Confirm blade off, no emergency, no lift warning, and an operator present.
2. Prefer `RECORDING` mode for blade-free controlled motion; always cancel it
   afterward so no area is saved.
3. Begin a new controller with a short forward test only. No pivots, reverse,
   or autonomous coverage unless the operator explicitly asks.
4. Any stale sensor input, mode change, emergency, or timeout must command
   zero velocity immediately.
5. Record results in `docs/lab/STATUS.md` before promoting a change.

## Upstream path

When a lab experiment is repeatable:

1. Split the production-worthy code from lab notes and hardware-specific
   values.
2. Add tests or a deterministic replay where practical.
3. Rebase onto `upstream/main` and open a focused PR against
   `mowglinext/mowglinext`.
4. Never include credentials, local hostnames, recordings, maps, or generated
   build artifacts in an upstream PR.
