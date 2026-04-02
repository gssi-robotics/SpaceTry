# Install `spacetry-autonomy-scenario-driver` Skill

This repository contains the skill source at:

```text
skills/spacetry-autonomy-scenario-driver/
```

Installing it for an LLM agent usually means exposing that folder through a location the agent scans for `SKILL.md`-based skills.

This guide is split into:

- IDE installation
- CLI installation

The examples below use symlinks so the skill stays maintained in one place inside this repository.

## IDE Installation

Use this when you want the skill available from an IDE agent such as Codex or GitHub Copilot.

### Option A: Shared repo-local install for Codex and Copilot

This is the recommended setup if you want one repository-scoped installation that works well across tools that support `.agents/skills`.

From the repository root:

```bash
mkdir -p .agents/skills
ln -s ../../skills/spacetry-autonomy-scenario-driver .agents/skills/spacetry-autonomy-scenario-driver
```

This keeps the skill local to this repository and makes it discoverable to:

- Codex IDE, which scans `.agents/skills` in the repository tree
- GitHub Copilot agents, which also support project skills in `.agents/skills`

### Option B: GitHub Copilot repository install

If you want to use GitHub Copilot's repository-specific skills location instead:

```bash
mkdir -p .github/skills
ln -s ../../skills/spacetry-autonomy-scenario-driver .github/skills/spacetry-autonomy-scenario-driver
```

Use this option when your team already standardizes on `.github/skills`.

### Verify in the IDE

After creating the symlink:

1. Reload the IDE window or restart the agent session.
2. Open the repository in the IDE.
3. Ask the agent to use the skill for a scenario-generation task.

For Codex, you can also check the skill list with `/skills` and invoke it explicitly with:

```text
$spacetry-autonomy-scenario-driver
```

### Notes for Copilot

- This repository already includes an `AGENTS.md` file at the root. That file gives Copilot repository instructions and complements the skill.
- Skills are best for task-specific workflows. `AGENTS.md` and `.github/copilot-instructions.md` are better for general repository guidance.

## CLI Installation

Use this when you want the skill available in terminal-based agents such as Codex CLI or GitHub Copilot CLI.

### Option A: Shared user-level install for Codex CLI and Copilot CLI

This is the simplest cross-tool setup because both agents support `~/.agents/skills`.

```bash
mkdir -p ~/.agents/skills
ln -s /home/keila/robotics/marti/skills/spacetry-autonomy-scenario-driver ~/.agents/skills/spacetry-autonomy-scenario-driver
```

This makes the skill available across repositories for:

- Codex CLI
- GitHub Copilot CLI

### Option B: Codex-only user install

Codex also supports user-level skills through `~/.agents/skills`, so Option A is already the preferred Codex CLI setup.

If Codex does not pick up the new skill immediately, restart Codex.

### Option C: Copilot-only user install

GitHub Copilot also supports a dedicated personal skills directory:

```bash
mkdir -p ~/.copilot/skills
ln -s /home/keila/robotics/marti/skills/spacetry-autonomy-scenario-driver ~/.copilot/skills/spacetry-autonomy-scenario-driver
```

Use this only if you specifically want the skill tied to Copilot's own home-directory path instead of the shared `~/.agents/skills` location.

## Usage After Installation

### Codex

In Codex CLI or Codex IDE:

```text
/skills
```

Then invoke the skill explicitly with:

```text
$spacetry-autonomy-scenario-driver
```

You can also simply describe a SpaceTry autonomy-scenario task and let Codex choose the skill implicitly.

### GitHub Copilot

In GitHub Copilot agent mode or Copilot CLI, ask for a task that matches the skill description, for example:

```text
Use the spacetry-autonomy-scenario-driver skill to create an autonomy evaluation scenario for the rover.
```

Copilot may also select the skill automatically when the task matches the `description` in `SKILL.md`.

## Troubleshooting

### The skill does not appear

- Confirm the symlink points to `skills/spacetry-autonomy-scenario-driver`.
- Confirm the directory contains a file named exactly `SKILL.md`.
- Restart the IDE or CLI session.

### The agent sees the repository instructions but not the skill

- `AGENTS.md` and skills are separate mechanisms.
- `AGENTS.md` gives repository instructions.
- The skill must still be installed in a supported skills directory such as `.agents/skills`, `.github/skills`, `~/.agents/skills`, or `~/.copilot/skills`.

## Recommended Setup Summary

- For IDE use across Codex and Copilot: `.agents/skills/spacetry-autonomy-scenario-driver`
- For CLI use across Codex and Copilot: `~/.agents/skills/spacetry-autonomy-scenario-driver`
- For GitHub-specific repository setup: `.github/skills/spacetry-autonomy-scenario-driver`
- For GitHub-specific personal setup: `~/.copilot/skills/spacetry-autonomy-scenario-driver`

## References

- OpenAI Codex skills documentation: https://developers.openai.com/codex/skills
- GitHub Copilot agent skills documentation: https://docs.github.com/en/copilot/how-tos/use-copilot-agents/coding-agent/create-skills
- GitHub Copilot custom instructions: https://docs.github.com/en/copilot/how-tos/configure-custom-instructions/add-repository-instructions
