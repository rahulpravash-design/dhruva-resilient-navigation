# Blockers

## B1: design/stitch/ and _claude_code_handoff/ missing (open)
- Symptom: no `contracts/nav_state.ts`, tokens, `CONTENT_AUDIT.md` or `CLAUDE_CODE_PROMPT.md` on disk.
- Affects: P0 schema generation, Python output schema validation, Kotlin `NavState` mirror, all of P8/P9.
- Not invented: `NavState` is not guessed. Engine work proceeds on the fixture format until the file lands.
- Needed from user: copy `design/stitch/` into the repo.
- Also to confirm when unblocked: which npm tool generates the JSON Schema from `nav_state.ts` (not named in the master prompt, so it needs approval).

## B2: Android SDK not installed (deferred by choice)
- Needed for P9 only. JDK 17 and Gradle 8.10.2 are present for the pure-JVM `engine_kt` (P7).
