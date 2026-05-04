from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

SCHEMA_VERSION = "1.0.0"

JSONScalar = None | bool | int | float | str
JSONValue = JSONScalar | list["JSONValue"] | dict[str, "JSONValue"]


@dataclass(kw_only=True)
class Event:
    event_id: str
    time_s: Optional[float]
    evidence_refs: list[str] = field(default_factory=list)
    metadata: dict[str, JSONValue] = field(default_factory=dict)


@dataclass(kw_only=True)
class TriggerEvent(Event):
    kind: str
    source: Optional[str] = None
    description: Optional[str] = None


@dataclass(kw_only=True)
class DetectionEvent(Event):
    source: Optional[str] = None
    attribution_status: str = "unresolved"
    candidate_sources: list[str] = field(default_factory=list)
    linked_trigger_ids: list[str] = field(default_factory=list)
    minimum_fault_distance_m: Optional[float] = None


@dataclass(kw_only=True)
class ReactionEvent(Event):
    observed_control_rationale: str
    reaction_scope: str = "indeterminate"
    attribution_status: str = "unresolved"
    active_context_at_reaction: dict[str, JSONValue] = field(default_factory=dict)
    candidate_sources: list[str] = field(default_factory=list)
    linked_trigger_ids: list[str] = field(default_factory=list)


@dataclass(kw_only=True)
class RecoveryEvent(Event):
    linked_reaction_ids: list[str] = field(default_factory=list)
    recovery_latency_ms: Optional[float] = None


@dataclass(kw_only=True)
class AdaptationEvent(Event):
    trigger_id: str
    reaction_id: str
    adaptation_latency_ms: Optional[float]
    attribution_scope: str = "indeterminate"
    attribution_confidence: str = "unresolved"
    candidate_sources: list[str] = field(default_factory=list)


@dataclass
class ScenarioSummaryMetrics:
    autonomy_reaction_status: bool = False
    autonomy_reaction_count: int = 0
    adaptation_event_count: int = 0
    detection_event_count: int = 0
    recovery_event_count: int = 0
    route_deviation_m: Optional[float] = None
    safety_preservation: dict[str, JSONValue] = field(default_factory=dict)
    goal_viability: dict[str, JSONValue] = field(default_factory=dict)
    injected_uncertainty_encounter_status: Optional[bool] = None
    evaluation_window_after_encounter_s: Optional[float] = None
    meaningful_evaluation_window_satisfied: Optional[bool] = None
    baseline_uncertainty_exercised_status: Optional[bool] = None
    detection_signal_source: Optional[str] = None
    detection_attribution_status: Optional[bool] = None
    minimum_fault_distance_at_detection_m: Optional[float] = None
    baseline_confound_status: JSONValue = None
    injected_uncertainty_source: list[str] = field(default_factory=list)


@dataclass
class ScenarioOutcomes:
    goal_status: Optional[str] = None
    safety_status: Optional[str] = None
    autonomy_assessment: Optional[str] = None
    baseline_outcome_assessment: Optional[str] = None
    injected_outcome_assessment: Optional[str] = None
    outcome_assessment: Optional[str] = None


@dataclass
class ArtifactPaths:
    report: Optional[str] = None
    metrics: Optional[str] = None
    timeline: Optional[str] = None
    rosbags: Optional[str] = None


@dataclass
class ScenarioMetricsBundle:
    scenario_name: str
    schema_version: str = SCHEMA_VERSION
    run_label: Optional[str] = None
    termination_reason: Optional[str] = None
    started_at_sim_s: Optional[float] = None
    finished_at_sim_s: Optional[float] = None
    runtime_s: Optional[float] = None
    trigger_events: list[TriggerEvent] = field(default_factory=list)
    detection_events: list[DetectionEvent] = field(default_factory=list)
    reaction_events: list[ReactionEvent] = field(default_factory=list)
    recovery_events: list[RecoveryEvent] = field(default_factory=list)
    adaptation_events: list[AdaptationEvent] = field(default_factory=list)
    summary_metrics: ScenarioSummaryMetrics = field(default_factory=ScenarioSummaryMetrics)
    outcomes: ScenarioOutcomes = field(default_factory=ScenarioOutcomes)
    additional_metrics: dict[str, JSONValue] = field(default_factory=dict)
    observability_gaps: list[str] = field(default_factory=list)
    artifact_paths: ArtifactPaths = field(default_factory=ArtifactPaths)
    metadata: dict[str, JSONValue] = field(default_factory=dict)
