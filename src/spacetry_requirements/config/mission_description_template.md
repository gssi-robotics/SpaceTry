# Mission Description Template

Use this template to describe a SpaceTry mission in natural language.
The user fills in this file. The FRETish agent then uses
`config/agent_prompt.md`, `config/fretish_templates.yaml`, and
`config/signal_inventory.yaml` to translate the description into a
minimal, monitorable `requirements.yaml`.

## Scenario: <mission name>

## Natural-language mission description

Describe the mission in plain English. Include:
- The start location or starting condition.
- The main mission goals.
- Any required ordering between goals.
- Safety constraints that must hold throughout the mission.
- What the rover must do when hazards or obstacles are detected.
- Any "eventually" mission-completion obligations.

Example prompts:
- "The rover must travel from <start> to <goal>."
- "During the entire mission, the rover must avoid collisions."
- "If <hazard condition> occurs, the rover must <safe response>."
- "The rover must eventually reach <waypoint/objective>."

Write the mission description here:

<replace with mission narrative>

## Environment

List the relevant facts about the environment and available infrastructure.
Include only information the agent can use to choose monitorable signals.

- Terrain or map context:
  <replace>
- Obstacles, hazards, or known no-go regions:
  <replace>
- Sensor sources available in SpaceTry:
  <replace>
- Actuation or command interfaces relevant to safety:
  <replace>
- Named waypoints or coordinates:
  <replace>
- Mission resources or constraints not modeled in SpaceTry:
  <replace>

## Optional derived safety rules

If you already know the intended safety rules, list them in plain English.
These are hints for the agent, not formal requirements.

1. <replace or delete>
2. <replace or delete>
3. <replace or delete>

## Rationale for exclusions

List mission aspects that should intentionally NOT become FRETish
requirements, especially when they are not monitorable in the current
SpaceTry infrastructure.

- <replace>
- <replace>

## Notes for the agent

Use this section for anything the agent should preserve while translating
the scenario into FRETish requirements.

- Prefer only verified signals from `config/signal_inventory.yaml` unless
  an unverified signal should be explicitly flagged.
- Keep the requirement set minimal: include hard safety constraints,
  liveness obligations, and necessary environment assumptions only.
- Do not encode behavior tree tactics, exploration preferences, or
  convenience actions unless they are mission-critical and monitorable.
