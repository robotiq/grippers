# Examples

**Start here: `move_gripper`** — the pattern every application should
use: a `Gripper` owns the bus; control code calls instant
setters and getters.

- `quick_start` — the minimal connect/activate/move sequence from the
  [Quick start](../../docs/02-quick-start.md) guide, with no error
  handling. Read alongside that guide.
- `move_gripper` — activate, close, and open a gripper through
  `Gripper`'s typed accessors, with error handling and logging. See the
  [walkthrough](../../docs/04-robust-example-walkthrough.md).
- `snippets.cpp` (built as `doc_snippets`) — not a real application: every
  `//! [tag]` region in it backs a `\snippet` reference in a header doc
  comment, or a `<!-- snippet: -->`-marked code fence in a `docs/*.md`
  guide (see [`sdk_cpp/tools/README.md`](../tools/README.md)). Compiled
  by every normal build so a broken doc example is a compile error, never
  run. Add to this file only for a concept with no natural home in
  `quick_start`/`move_gripper` themselves.

All three are built by `GRIPPERS_BUILD_EXAMPLES` (see
[Environment setup](../../docs/01-environment-setup.md)); `doc_snippets`
additionally needs `GRIPPERS_BUILD_FAKE` (default on together with it) for
its `makeFakeGripper()` example.
