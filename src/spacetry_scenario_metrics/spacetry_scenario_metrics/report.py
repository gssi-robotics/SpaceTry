from __future__ import annotations

import json
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from statistics import median
from typing import Any

from .schema import ScenarioMetricsBundle


def _format_value(value: Any) -> str:
    if value is None:
        return "`null`"
    if isinstance(value, (dict, list)):
        return f"`{json.dumps(value, sort_keys=True)}`"
    return f"`{value}`"


def _append_metric_lines(lines: list[str], metrics: dict[str, Any]) -> None:
    for key, value in metrics.items():
        lines.append(f"- {key}: {_format_value(value)}")


def _non_empty_counter(values: list[str]) -> Counter[str]:
    counter: Counter[str] = Counter()
    for value in values:
        normalized = str(value).strip()
        if not normalized:
            continue
        counter[normalized] += 1
    return counter


def _counter_summary(counter: Counter[str]) -> str:
    if not counter:
        return "`n/a`"
    ordered = sorted(counter.items(), key=lambda item: (-item[1], item[0]))
    return "`" + ", ".join(f"{key}={value}" for key, value in ordered) + "`"


def _top_counter_summary(counter: Counter[str], limit: int = 5) -> str:
    if not counter:
        return "`n/a`"
    ordered = sorted(counter.items(), key=lambda item: (-item[1], item[0]))[:limit]
    return "`" + ", ".join(f"{key}={value}" for key, value in ordered) + "`"


def _latency_summary(values: list[float]) -> str:
    if not values:
        return "`n/a`"
    sorted_values = sorted(values)
    return (
        "`"
        f"min={sorted_values[0]:.3f}, "
        f"median={median(sorted_values):.3f}, "
        f"max={sorted_values[-1]:.3f}"
        "`"
    )


def _append_if_present(lines: list[str], label: str, value: Any) -> None:
    if value is None:
        return
    if isinstance(value, str) and not value.strip():
        return
    lines.append(f"- {label}: {_format_value(value)}")


def render_markdown_report(bundle: ScenarioMetricsBundle) -> str:
    summary_metrics = asdict(bundle.summary_metrics)
    outcomes = asdict(bundle.outcomes)
    metadata = bundle.metadata or {}
    safety_preservation = summary_metrics.pop("safety_preservation", {})
    goal_viability = summary_metrics.pop("goal_viability", {})

    adaptation_latencies = [
        float(event.adaptation_latency_ms)
        for event in bundle.adaptation_events
        if isinstance(event.adaptation_latency_ms, (int, float))
    ]
    reaction_rationales = _non_empty_counter(
        [event.observed_control_rationale for event in bundle.reaction_events]
    )
    reaction_scopes = _non_empty_counter(
        [event.reaction_scope for event in bundle.reaction_events]
    )
    reaction_attribution_statuses = _non_empty_counter(
        [event.attribution_status for event in bundle.reaction_events]
    )
    adaptation_scopes = _non_empty_counter(
        [event.attribution_scope for event in bundle.adaptation_events]
    )
    adaptation_confidences = _non_empty_counter(
        [event.attribution_confidence for event in bundle.adaptation_events]
    )
    adaptation_sources = Counter(
        source
        for event in bundle.adaptation_events
        for source in event.candidate_sources
        if str(source).strip()
    )
    detection_attribution_statuses = _non_empty_counter(
        [event.attribution_status for event in bundle.detection_events]
    )
    detection_sources = Counter(
        source
        for event in bundle.detection_events
        if isinstance(event.source, str) and event.source.strip()
        for source in [event.source]
    )
    detection_rejection_reasons = Counter(
        str(reason)
        for event in bundle.detection_events
        for reason in [event.metadata.get("rejection_reason")]
        if isinstance(reason, str) and reason.strip()
    )
    trigger_kinds = _non_empty_counter([event.kind for event in bundle.trigger_events])
    injection_trigger_times = [
        float(event.time_s)
        for event in bundle.trigger_events
        if event.kind == "uncertainty_injection"
        and isinstance(event.time_s, (int, float))
    ]
    recovery_latencies = [
        float(event.recovery_latency_ms)
        for event in bundle.recovery_events
        if isinstance(event.recovery_latency_ms, (int, float))
    ]

    lines = [
        f"# Scenario Report: {bundle.scenario_name}",
        "",
        f"- schema_version: `{bundle.schema_version}`",
    ]
    if bundle.run_label is not None:
        lines.append(f"- run_label: `{bundle.run_label}`")
    if bundle.termination_reason is not None:
        lines.append(f"- termination_reason: `{bundle.termination_reason}`")
    if bundle.runtime_s is not None:
        lines.append(f"- runtime_s: `{bundle.runtime_s}`")
    for key in (
        "goal_status",
        "safety_status",
        "autonomy_assessment",
        "baseline_outcome_assessment",
        "injected_outcome_assessment",
        "outcome_assessment",
    ):
        value = outcomes.get(key)
        if value is not None:
            lines.append(f"- {key}: `{value}`")

    lines.extend(["", "## Scenario Summary", ""])
    interaction_hypothesis = metadata.get("interaction_hypothesis")
    if isinstance(interaction_hypothesis, str) and interaction_hypothesis.strip():
        lines.append(interaction_hypothesis.strip())
    else:
        primary_target = metadata.get("primary_evaluation_target")
        injected_sources = summary_metrics.get("injected_uncertainty_source")
        lines.append(
            "This report summarizes the main autonomy outcome, "
            "runtime facts, and aggregated event statistics for the scenario run."
        )
        if primary_target is not None:
            lines.append("")
            lines.append(f"- primary_evaluation_target: {_format_value(primary_target)}")
        if injected_sources:
            lines.append(f"- injected_uncertainty_source: {_format_value(injected_sources)}")

    lines.extend(["", "## Runtime Facts", ""])
    _append_if_present(lines, "injection_status", metadata.get("injection_status"))
    _append_if_present(
        lines,
        "injected_uncertainty_encounter_status",
        summary_metrics.get("injected_uncertainty_encounter_status"),
    )
    _append_if_present(
        lines,
        "meaningful_evaluation_window_satisfied",
        summary_metrics.get("meaningful_evaluation_window_satisfied"),
    )
    _append_if_present(
        lines,
        "baseline_uncertainty_exercised_status",
        summary_metrics.get("baseline_uncertainty_exercised_status"),
    )
    _append_if_present(
        lines,
        "baseline_confound_status",
        summary_metrics.get("baseline_confound_status"),
    )
    _append_if_present(
        lines,
        "injected_uncertainty_source",
        summary_metrics.get("injected_uncertainty_source"),
    )
    _append_if_present(
        lines,
        "primary_observed_control_rationale",
        metadata.get("primary_observed_control_rationale"),
    )
    _append_if_present(
        lines,
        "primary_reaction_scope",
        metadata.get("primary_reaction_scope"),
    )
    _append_if_present(
        lines,
        "primary_reaction_attribution_status",
        metadata.get("primary_reaction_attribution_status"),
    )
    _append_if_present(
        lines,
        "detection_signal_source",
        summary_metrics.get("detection_signal_source"),
    )

    lines.extend(["", "## Metrics", ""])
    _append_metric_lines(lines, summary_metrics)

    if bundle.additional_metrics:
        lines.extend(["", "## Additional Metrics", ""])
        for key, value in bundle.additional_metrics.items():
            lines.append(f"- {key}: {_format_value(value)}")

    lines.extend(["", "## Event Summaries", ""])

    if bundle.trigger_events:
        lines.extend(["", "### Triggers", ""])
        lines.append(f"- count: `{len(bundle.trigger_events)}`")
        lines.append(f"- kinds: {_counter_summary(trigger_kinds)}")
        if injection_trigger_times:
            lines.append(f"- uncertainty_injection_time_s: `{min(injection_trigger_times)}`")

    if bundle.detection_events:
        lines.extend(["", "### Detections", ""])
        lines.append(f"- count: `{len(bundle.detection_events)}`")
        lines.append(
            f"- attribution_statuses: {_counter_summary(detection_attribution_statuses)}"
        )
        lines.append(f"- top_sources: {_top_counter_summary(detection_sources)}")
        if detection_rejection_reasons:
            lines.append(
                f"- rejection_reasons: {_counter_summary(detection_rejection_reasons)}"
            )

    if bundle.reaction_events:
        lines.extend(["", "### Reactions", ""])
        lines.append(f"- count: `{len(bundle.reaction_events)}`")
        lines.append(f"- observed_control_rationale: {_counter_summary(reaction_rationales)}")
        lines.append(f"- reaction_scope: {_counter_summary(reaction_scopes)}")
        lines.append(
            f"- attribution_statuses: {_counter_summary(reaction_attribution_statuses)}"
        )

    if bundle.adaptation_events:
        lines.extend(["", "### Adaptations", ""])
        lines.append(f"- count: `{len(bundle.adaptation_events)}`")
        lines.append(f"- adaptation_latency_ms: {_latency_summary(adaptation_latencies)}")
        lines.append(f"- attribution_scope: {_counter_summary(adaptation_scopes)}")
        lines.append(
            f"- attribution_confidence: {_counter_summary(adaptation_confidences)}"
        )
        lines.append(f"- candidate_sources: {_top_counter_summary(adaptation_sources)}")

    if bundle.recovery_events:
        lines.extend(["", "### Recoveries", ""])
        lines.append(f"- count: `{len(bundle.recovery_events)}`")
        lines.append(f"- recovery_latency_ms: {_latency_summary(recovery_latencies)}")

    if safety_preservation or goal_viability:
        lines.extend(["", "## Safety And Goal Viability", ""])
        if safety_preservation:
            lines.append(f"- safety_preservation: {_format_value(safety_preservation)}")
        if goal_viability:
            lines.append(f"- goal_viability: {_format_value(goal_viability)}")

    lines.extend(["", "## Observability Notes", ""])
    if bundle.observability_gaps:
        for gap in bundle.observability_gaps:
            lines.append(f"- {gap}")
    detection_metric_note = metadata.get("detection_metric_note")
    if isinstance(detection_metric_note, str) and detection_metric_note.strip():
        lines.append(f"- {detection_metric_note.strip()}")
    lines.append(
        "- Detailed per-event records are intentionally omitted from this report; "
        "see the metrics JSON, timeline, and rosbag artifacts for raw execution data."
    )

    artifact_paths = {
        key: value for key, value in asdict(bundle.artifact_paths).items() if value is not None
    }
    if artifact_paths:
        lines.extend(["", "## Artifacts", ""])
        for key, value in artifact_paths.items():
            lines.append(f"- {key}: `{value}`")

    return "\n".join(lines) + "\n"


def write_markdown_report(bundle: ScenarioMetricsBundle, output_path: str | Path) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_markdown_report(bundle), encoding="utf-8")
