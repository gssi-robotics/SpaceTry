from spacetry_scenario_metrics import (
    ArtifactPaths,
    ScenarioMetricsSession,
    ScenarioOutcomes,
    ScenarioSummaryMetrics,
    bundle_to_dict,
    render_markdown_report,
)


def test_adaptation_event_latency_and_summary_counts() -> None:
    session = ScenarioMetricsSession(
        scenario_name="spacetry_scenario_metrics_demo",
        run_label="full_run_demo",
    )
    trigger_id = session.note_trigger(
        time_s=10.0,
        kind="uncertainty_injection",
        source="runtime_blocking_rock",
    )
    reaction_id = session.note_reaction(
        time_s=12.5,
        observed_control_rationale="obstacle_avoidance",
        reaction_scope="injected_only",
        attribution_status="supported",
        candidate_sources=["runtime_blocking_rock"],
        linked_trigger_ids=[trigger_id],
    )
    session.note_adaptation(
        trigger_id=trigger_id,
        reaction_id=reaction_id,
        attribution_scope="injected_only",
        attribution_confidence="supported",
    )
    session.note_recovery(
        time_s=14.0,
        linked_reaction_ids=[reaction_id],
    )

    bundle = session.build_bundle(
        termination_reason="goal_reached",
        started_at_sim_s=0.0,
        finished_at_sim_s=30.0,
        summary_metrics=ScenarioSummaryMetrics(
            route_deviation_m=1.25,
            safety_preservation={"MR_009": True, "MR_011": True},
            goal_viability={"science_rock_01_reached": True},
            injected_uncertainty_encounter_status=True,
            evaluation_window_after_encounter_s=18.0,
            meaningful_evaluation_window_satisfied=True,
            baseline_uncertainty_exercised_status=False,
        ),
        outcomes=ScenarioOutcomes(
            goal_status="PASS",
            safety_status="PASS",
            autonomy_assessment="PASS",
            baseline_outcome_assessment="NOT_EVALUATED",
            injected_outcome_assessment="PASS",
            outcome_assessment="PASS",
        ),
        additional_metrics={
            "raw_scan_detection_latency_ms": None,
            "post_injection_route_deviation_m": 0.75,
        },
        artifact_paths=ArtifactPaths(report="/tmp/report.md", metrics="/tmp/metrics.json"),
    )

    metrics_dict = bundle_to_dict(bundle)
    adaptation_event = metrics_dict["adaptation_events"][0]
    assert adaptation_event["adaptation_latency_ms"] == 2500.0
    assert metrics_dict["summary_metrics"]["autonomy_reaction_status"] is True
    assert metrics_dict["summary_metrics"]["autonomy_reaction_count"] == 1
    assert metrics_dict["summary_metrics"]["adaptation_event_count"] == 1
    assert metrics_dict["summary_metrics"]["recovery_event_count"] == 1
    assert metrics_dict["summary_metrics"]["injected_uncertainty_source"] == [
        "runtime_blocking_rock"
    ]


def test_report_summarizes_events_instead_of_dumping_raw_sections() -> None:
    session = ScenarioMetricsSession(scenario_name="spacetry_scenario_metrics_demo")
    trigger_id = session.note_trigger(time_s=1.0, kind="baseline_request", source="baseline")
    reaction_id = session.note_reaction(
        time_s=2.0,
        observed_control_rationale="monitor_enforcement",
        linked_trigger_ids=[trigger_id],
    )
    session.note_adaptation(trigger_id=trigger_id, reaction_id=reaction_id)

    bundle = session.build_bundle(
        termination_reason="timeout",
        summary_metrics=ScenarioSummaryMetrics(),
        outcomes=ScenarioOutcomes(outcome_assessment="DEGRADED"),
    )

    report = render_markdown_report(bundle)
    assert "## Event Summaries" in report
    assert "### Adaptations" in report
    assert "adaptation_latency_ms" in report
    assert "## Adaptation Events" not in report
    assert "## Reaction Events" not in report
    assert "## Trigger Events" not in report
    assert "Detailed per-event records are intentionally omitted" in report


def test_report_keeps_user_requested_metrics_under_additional_metrics() -> None:
    session = ScenarioMetricsSession(scenario_name="spacetry_scenario_metrics_demo")
    bundle = session.build_bundle(
        termination_reason="timeout",
        summary_metrics=ScenarioSummaryMetrics(route_deviation_m=1.2),
        outcomes=ScenarioOutcomes(outcome_assessment="DEGRADED"),
        additional_metrics={
            "obstacle_detection_latency_ms": 2400.0,
            "recovery_rate_ms": 6100.0,
        },
    )

    report = render_markdown_report(bundle)
    metrics_dict = bundle_to_dict(bundle)

    assert "## Additional Metrics" in report
    assert "## Metrics" in report
    assert metrics_dict["additional_metrics"]["obstacle_detection_latency_ms"] == 2400.0
    assert metrics_dict["additional_metrics"]["recovery_rate_ms"] == 6100.0
