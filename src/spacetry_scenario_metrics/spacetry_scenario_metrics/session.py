from __future__ import annotations

from collections import defaultdict
from dataclasses import replace
from typing import Optional

from .schema import (
    AdaptationEvent,
    ArtifactPaths,
    DetectionEvent,
    JSONValue,
    RecoveryEvent,
    ReactionEvent,
    ScenarioMetricsBundle,
    ScenarioOutcomes,
    ScenarioSummaryMetrics,
    TriggerEvent,
)


def _round_or_none(value: Optional[float], digits: int = 3) -> Optional[float]:
    if value is None:
        return None
    return round(float(value), digits)


def _duration_ms(start_s: Optional[float], end_s: Optional[float]) -> Optional[float]:
    if start_s is None or end_s is None:
        return None
    return round((float(end_s) - float(start_s)) * 1000.0, 3)


class ScenarioMetricsSession:
    def __init__(
        self,
        *,
        scenario_name: str,
        run_label: Optional[str] = None,
        metadata: Optional[dict[str, JSONValue]] = None,
    ) -> None:
        self.scenario_name = scenario_name
        self.run_label = run_label
        self.metadata = metadata or {}
        self._counters: defaultdict[str, int] = defaultdict(int)
        self._trigger_events: dict[str, TriggerEvent] = {}
        self._detection_events: dict[str, DetectionEvent] = {}
        self._reaction_events: dict[str, ReactionEvent] = {}
        self._recovery_events: dict[str, RecoveryEvent] = {}
        self._adaptation_events: dict[str, AdaptationEvent] = {}

    def _next_id(self, prefix: str) -> str:
        self._counters[prefix] += 1
        return f"{prefix}_{self._counters[prefix]:03d}"

    def _require_trigger_ids(self, trigger_ids: list[str]) -> None:
        missing = [trigger_id for trigger_id in trigger_ids if trigger_id not in self._trigger_events]
        if missing:
            raise ValueError(f"Unknown trigger ids: {missing}")

    def _require_reaction_ids(self, reaction_ids: list[str]) -> None:
        missing = [reaction_id for reaction_id in reaction_ids if reaction_id not in self._reaction_events]
        if missing:
            raise ValueError(f"Unknown reaction ids: {missing}")

    def note_trigger(
        self,
        *,
        time_s: Optional[float],
        kind: str,
        source: Optional[str] = None,
        description: Optional[str] = None,
        trigger_id: Optional[str] = None,
        evidence_refs: Optional[list[str]] = None,
        metadata: Optional[dict[str, JSONValue]] = None,
    ) -> str:
        trigger_id = trigger_id or self._next_id("trigger")
        self._trigger_events[trigger_id] = TriggerEvent(
            event_id=trigger_id,
            time_s=_round_or_none(time_s),
            kind=kind,
            source=source,
            description=description,
            evidence_refs=list(evidence_refs or []),
            metadata=metadata or {},
        )
        return trigger_id

    def note_detection(
        self,
        *,
        time_s: Optional[float],
        source: Optional[str] = None,
        attribution_status: str = "unresolved",
        candidate_sources: Optional[list[str]] = None,
        linked_trigger_ids: Optional[list[str]] = None,
        evidence_refs: Optional[list[str]] = None,
        minimum_fault_distance_m: Optional[float] = None,
        detection_id: Optional[str] = None,
        metadata: Optional[dict[str, JSONValue]] = None,
    ) -> str:
        linked_trigger_ids = list(linked_trigger_ids or [])
        self._require_trigger_ids(linked_trigger_ids)
        detection_id = detection_id or self._next_id("detection")
        self._detection_events[detection_id] = DetectionEvent(
            event_id=detection_id,
            time_s=_round_or_none(time_s),
            source=source,
            attribution_status=attribution_status,
            candidate_sources=list(candidate_sources or []),
            linked_trigger_ids=linked_trigger_ids,
            evidence_refs=list(evidence_refs or []),
            minimum_fault_distance_m=_round_or_none(minimum_fault_distance_m),
            metadata=metadata or {},
        )
        return detection_id

    def note_reaction(
        self,
        *,
        time_s: Optional[float],
        observed_control_rationale: str,
        reaction_scope: str = "indeterminate",
        attribution_status: str = "unresolved",
        active_context_at_reaction: Optional[dict[str, JSONValue]] = None,
        candidate_sources: Optional[list[str]] = None,
        linked_trigger_ids: Optional[list[str]] = None,
        evidence_refs: Optional[list[str]] = None,
        reaction_id: Optional[str] = None,
        metadata: Optional[dict[str, JSONValue]] = None,
    ) -> str:
        linked_trigger_ids = list(linked_trigger_ids or [])
        self._require_trigger_ids(linked_trigger_ids)
        reaction_id = reaction_id or self._next_id("reaction")
        self._reaction_events[reaction_id] = ReactionEvent(
            event_id=reaction_id,
            time_s=_round_or_none(time_s),
            observed_control_rationale=observed_control_rationale,
            reaction_scope=reaction_scope,
            attribution_status=attribution_status,
            active_context_at_reaction=active_context_at_reaction or {},
            candidate_sources=list(candidate_sources or []),
            linked_trigger_ids=linked_trigger_ids,
            evidence_refs=list(evidence_refs or []),
            metadata=metadata or {},
        )
        return reaction_id

    def note_recovery(
        self,
        *,
        time_s: Optional[float],
        linked_reaction_ids: Optional[list[str]] = None,
        evidence_refs: Optional[list[str]] = None,
        recovery_id: Optional[str] = None,
        recovery_latency_ms: Optional[float] = None,
        metadata: Optional[dict[str, JSONValue]] = None,
    ) -> str:
        linked_reaction_ids = list(linked_reaction_ids or [])
        self._require_reaction_ids(linked_reaction_ids)
        if recovery_latency_ms is None and len(linked_reaction_ids) == 1:
            reaction = self._reaction_events[linked_reaction_ids[0]]
            recovery_latency_ms = _duration_ms(reaction.time_s, time_s)
        recovery_id = recovery_id or self._next_id("recovery")
        self._recovery_events[recovery_id] = RecoveryEvent(
            event_id=recovery_id,
            time_s=_round_or_none(time_s),
            linked_reaction_ids=linked_reaction_ids,
            recovery_latency_ms=_round_or_none(recovery_latency_ms),
            evidence_refs=list(evidence_refs or []),
            metadata=metadata or {},
        )
        return recovery_id

    def note_adaptation(
        self,
        *,
        trigger_id: str,
        reaction_id: str,
        attribution_scope: str = "indeterminate",
        attribution_confidence: str = "unresolved",
        candidate_sources: Optional[list[str]] = None,
        evidence_refs: Optional[list[str]] = None,
        adaptation_event_id: Optional[str] = None,
        metadata: Optional[dict[str, JSONValue]] = None,
    ) -> str:
        self._require_trigger_ids([trigger_id])
        self._require_reaction_ids([reaction_id])
        trigger = self._trigger_events[trigger_id]
        reaction = self._reaction_events[reaction_id]
        adaptation_event_id = adaptation_event_id or self._next_id("adaptation")
        self._adaptation_events[adaptation_event_id] = AdaptationEvent(
            event_id=adaptation_event_id,
            time_s=reaction.time_s,
            trigger_id=trigger_id,
            reaction_id=reaction_id,
            adaptation_latency_ms=_duration_ms(trigger.time_s, reaction.time_s),
            attribution_scope=attribution_scope,
            attribution_confidence=attribution_confidence,
            candidate_sources=list(candidate_sources or reaction.candidate_sources),
            evidence_refs=list(evidence_refs or reaction.evidence_refs),
            metadata=metadata or {},
        )
        return adaptation_event_id

    def build_bundle(
        self,
        *,
        termination_reason: Optional[str],
        started_at_sim_s: Optional[float] = None,
        finished_at_sim_s: Optional[float] = None,
        summary_metrics: Optional[ScenarioSummaryMetrics] = None,
        outcomes: Optional[ScenarioOutcomes] = None,
        additional_metrics: Optional[dict[str, JSONValue]] = None,
        observability_gaps: Optional[list[str]] = None,
        artifact_paths: Optional[ArtifactPaths] = None,
        metadata: Optional[dict[str, JSONValue]] = None,
    ) -> ScenarioMetricsBundle:
        ordered_trigger_events = sorted(
            self._trigger_events.values(),
            key=lambda item: (item.time_s is None, item.time_s, item.event_id),
        )
        ordered_detection_events = sorted(
            self._detection_events.values(),
            key=lambda item: (item.time_s is None, item.time_s, item.event_id),
        )
        ordered_reaction_events = sorted(
            self._reaction_events.values(),
            key=lambda item: (item.time_s is None, item.time_s, item.event_id),
        )
        ordered_recovery_events = sorted(
            self._recovery_events.values(),
            key=lambda item: (item.time_s is None, item.time_s, item.event_id),
        )
        ordered_adaptation_events = sorted(
            self._adaptation_events.values(),
            key=lambda item: (item.time_s is None, item.time_s, item.event_id),
        )

        summary_metrics = replace(summary_metrics or ScenarioSummaryMetrics())
        summary_metrics.autonomy_reaction_status = bool(ordered_reaction_events)
        summary_metrics.autonomy_reaction_count = len(ordered_reaction_events)
        summary_metrics.adaptation_event_count = len(ordered_adaptation_events)
        summary_metrics.detection_event_count = len(ordered_detection_events)
        summary_metrics.recovery_event_count = len(ordered_recovery_events)

        if summary_metrics.detection_signal_source is None and ordered_detection_events:
            summary_metrics.detection_signal_source = ordered_detection_events[0].source

        if summary_metrics.detection_attribution_status is None and ordered_detection_events:
            summary_metrics.detection_attribution_status = any(
                event.attribution_status == "supported" for event in ordered_detection_events
            )

        if summary_metrics.minimum_fault_distance_at_detection_m is None and ordered_detection_events:
            distances = [
                event.minimum_fault_distance_m
                for event in ordered_detection_events
                if event.minimum_fault_distance_m is not None
            ]
            if distances:
                summary_metrics.minimum_fault_distance_at_detection_m = min(distances)

        if not summary_metrics.injected_uncertainty_source:
            inferred_sources: list[str] = []
            for adaptation_event in ordered_adaptation_events:
                for source in adaptation_event.candidate_sources:
                    if source not in inferred_sources:
                        inferred_sources.append(source)
            summary_metrics.injected_uncertainty_source = inferred_sources

        metadata_payload = dict(self.metadata)
        metadata_payload.update(metadata or {})

        runtime_s = None
        if started_at_sim_s is not None and finished_at_sim_s is not None:
            runtime_s = round(float(finished_at_sim_s) - float(started_at_sim_s), 3)

        return ScenarioMetricsBundle(
            scenario_name=self.scenario_name,
            run_label=self.run_label,
            termination_reason=termination_reason,
            started_at_sim_s=_round_or_none(started_at_sim_s),
            finished_at_sim_s=_round_or_none(finished_at_sim_s),
            runtime_s=runtime_s,
            trigger_events=ordered_trigger_events,
            detection_events=ordered_detection_events,
            reaction_events=ordered_reaction_events,
            recovery_events=ordered_recovery_events,
            adaptation_events=ordered_adaptation_events,
            summary_metrics=summary_metrics,
            outcomes=outcomes or ScenarioOutcomes(),
            additional_metrics=additional_metrics or {},
            observability_gaps=list(observability_gaps or []),
            artifact_paths=artifact_paths or ArtifactPaths(),
            metadata=metadata_payload,
        )
