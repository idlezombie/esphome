# Agent notes

Repo-wide notes for AI agents working in this repo. See also the project rules in
`.cursor/rules/` (local, not committed) and per-component notes such as
`components/actron_modbus/TODO.md`.

## Local environment / tooling

- This machine is Windows with the shell set to PowerShell.
- `git` is **not** on `PATH`. Use the Git bundled with Sublime Merge:

  ```
  C:\Program Files\Sublime Merge\Git\cmd\git.exe
  ```

  Example:

  ```powershell
  & "C:\Program Files\Sublime Merge\Git\cmd\git.exe" status
  ```

  Only one tool (Cursor or Sublime Merge) should perform git mutations at a time;
  read-only git in parallel is fine.
