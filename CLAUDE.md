# fast_adc — pointer for Claude sessions

Owner: David Smith. Teensy 4.x two-channel ADC capture, sketch in `ds_high_speed_ADC/`.
Develop on branch `claude/cowork-program-review-9k01ca`; open a PR only when asked.

## Read these first (Dropbox, via the Dropbox connector)
Cloud sessions do not mount `~/.claude` or Dropbox, so the global rules are not loaded
automatically. At session start read:
1. `/.claude/CLAUDE.md` — David's persistent context and preferences (includes the
   response-prefix format and timestamp rule).
2. `/AI_Staff/README.md` — staff roster and how to hand work off.

Do not copy their rules here; they change. This file only points to them.

## Staff and hand-offs
Staff: Terri (personal secretary agent), Cassie, Wendy, Chatica (see the roster for roles).
Hand-offs are shared files, not wake-ups: create
`/AI_Staff/<Entity>/Tasks/Task_<UTC timestamp>_<8 hex>/` with `task.md`, `task.json`
and `Exchanges/<From>_<To>_<kind>_<hex>_<YY-MM-DD>.md`. The recipient finds it on its next run.
Confirm with David before writing to Dropbox, and say who now owns the task.

## Repo notes
- David's blogs (engineeringnotes.blogspot.com, onlytwelvenotes.blogspot.com) are
  references and contribution targets; a cloud environment must allowlist them or
  they return 403. Agents cannot publish to Blogger; draft posts as Markdown for David.
- Open item: phase-accumulator timing from the engineeringnotes post
  (`/2026/01/a-better-way-to-create-non-blocking.html`) not yet read or applied.
- Sketch status: v0.9.8, not yet compiled on hardware. Hardware-timed ADC (timer + DMA) was
  considered and deferred.
