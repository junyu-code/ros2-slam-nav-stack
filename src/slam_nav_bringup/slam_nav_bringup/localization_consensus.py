from __future__ import annotations

import math
from dataclasses import dataclass


NORMAL = "NORMAL"
NORMAL_DEGRADED = "NORMAL_DEGRADED"
AMCL_RESET_RECOMMENDED = "AMCL_RESET_RECOMMENDED"
FAST_LIO_GLOBAL_CORRECTION_RECOMMENDED = "FAST_LIO_GLOBAL_CORRECTION_RECOMMENDED"
GICP_REJECTED = "GICP_REJECTED"
DISAGREEMENT_OBSERVED = "DISAGREEMENT_OBSERVED"
DISAGREEMENT_UNRESOLVED = "DISAGREEMENT_UNRESOLVED"
MANUAL_RELOCALIZATION_REQUIRED = "MANUAL_RELOCALIZATION_REQUIRED"
INSUFFICIENT_DATA = "INSUFFICIENT_DATA"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PoseDelta:
    translation: float
    yaw: float


@dataclass(frozen=True)
class ConsensusDecision:
    decision: str
    reason: str
    reference: str | None
    pairwise: dict[str, PoseDelta]


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compose_pose(parent_child: Pose2D, child_body: Pose2D) -> Pose2D:
    cosine = math.cos(parent_child.yaw)
    sine = math.sin(parent_child.yaw)
    return Pose2D(
        x=parent_child.x + cosine * child_body.x - sine * child_body.y,
        y=parent_child.y + sine * child_body.x + cosine * child_body.y,
        yaw=normalize_angle(parent_child.yaw + child_body.yaw),
    )


def pose_delta(first: Pose2D, second: Pose2D) -> PoseDelta:
    return PoseDelta(
        translation=math.hypot(first.x - second.x, first.y - second.y),
        yaw=abs(normalize_angle(first.yaw - second.yaw)),
    )


def _agrees(delta: PoseDelta, translation_limit: float, yaw_limit: float) -> bool:
    return delta.translation <= translation_limit and delta.yaw <= yaw_limit


def _requires_action(delta: PoseDelta, translation_limit: float, yaw_limit: float) -> bool:
    return delta.translation >= translation_limit or delta.yaw >= yaw_limit


def _exceeds_auto_limit(delta: PoseDelta, translation_limit: float, yaw_limit: float) -> bool:
    return delta.translation > translation_limit or delta.yaw > yaw_limit


def evaluate_consensus(
    fast_lio: Pose2D | None,
    amcl: Pose2D | None,
    gicp: Pose2D | None,
    *,
    fast_lio_healthy: bool,
    amcl_healthy: bool,
    gicp_healthy: bool,
    agreement_translation: float = 0.25,
    agreement_yaw: float = 0.15,
    correction_translation: float = 0.50,
    correction_yaw: float = 0.25,
    max_auto_translation: float = 2.0,
    max_auto_yaw: float = 0.8,
) -> ConsensusDecision:
    """Classify three map-frame pose candidates without applying any correction."""
    if fast_lio is None or amcl is None or gicp is None:
        return ConsensusDecision(
            INSUFFICIENT_DATA,
            "all three fresh pose candidates are required",
            None,
            {},
        )

    pairwise = {
        "fast_lio_amcl": pose_delta(fast_lio, amcl),
        "fast_lio_gicp": pose_delta(fast_lio, gicp),
        "amcl_gicp": pose_delta(amcl, gicp),
    }
    fast_amcl_agree = _agrees(
        pairwise["fast_lio_amcl"], agreement_translation, agreement_yaw
    )
    fast_gicp_agree = _agrees(
        pairwise["fast_lio_gicp"], agreement_translation, agreement_yaw
    )
    amcl_gicp_agree = _agrees(
        pairwise["amcl_gicp"], agreement_translation, agreement_yaw
    )

    if fast_amcl_agree and fast_gicp_agree and amcl_gicp_agree:
        healthy = fast_lio_healthy and amcl_healthy and gicp_healthy
        return ConsensusDecision(
            NORMAL if healthy else NORMAL_DEGRADED,
            "all pose candidates agree" if healthy else "poses agree but at least one quality gate is low",
            None,
            pairwise,
        )

    if fast_gicp_agree and not amcl_gicp_agree:
        divergence = pairwise["amcl_gicp"]
        if not (fast_lio_healthy and gicp_healthy):
            return ConsensusDecision(
                DISAGREEMENT_UNRESOLVED,
                "FAST-LIO and GICP agree but their quality cannot support an AMCL reset",
                None,
                pairwise,
            )
        if not _requires_action(
            divergence, correction_translation, correction_yaw
        ):
            return ConsensusDecision(
                DISAGREEMENT_OBSERVED,
                "AMCL differs slightly from the agreeing FAST-LIO and GICP references",
                "gicp",
                pairwise,
            )
        if _exceeds_auto_limit(divergence, max_auto_translation, max_auto_yaw):
            return ConsensusDecision(
                MANUAL_RELOCALIZATION_REQUIRED,
                "AMCL disagreement exceeds the automatic reset limit",
                "gicp",
                pairwise,
            )
        return ConsensusDecision(
            AMCL_RESET_RECOMMENDED,
            "FAST-LIO and GICP agree while AMCL differs",
            "gicp",
            pairwise,
        )

    if amcl_gicp_agree and not fast_gicp_agree:
        divergence = pairwise["fast_lio_gicp"]
        if not (amcl_healthy and gicp_healthy):
            return ConsensusDecision(
                DISAGREEMENT_UNRESOLVED,
                "AMCL and GICP agree but their quality cannot support a global correction",
                None,
                pairwise,
            )
        if not _requires_action(
            divergence, correction_translation, correction_yaw
        ):
            return ConsensusDecision(
                DISAGREEMENT_OBSERVED,
                "FAST-LIO global pose differs slightly from the agreeing global references",
                "gicp",
                pairwise,
            )
        if _exceeds_auto_limit(divergence, max_auto_translation, max_auto_yaw):
            return ConsensusDecision(
                MANUAL_RELOCALIZATION_REQUIRED,
                "FAST-LIO global disagreement exceeds the automatic correction limit",
                "gicp",
                pairwise,
            )
        return ConsensusDecision(
            FAST_LIO_GLOBAL_CORRECTION_RECOMMENDED,
            "AMCL and GICP agree while the FAST-LIO global pose differs",
            "gicp",
            pairwise,
        )

    if fast_amcl_agree and not fast_gicp_agree:
        if fast_lio_healthy and amcl_healthy:
            return ConsensusDecision(
                GICP_REJECTED,
                "FAST-LIO and AMCL agree while GICP differs",
                "fast_lio_amcl",
                pairwise,
            )
        return ConsensusDecision(
            DISAGREEMENT_UNRESOLVED,
            "FAST-LIO and AMCL agree but their quality cannot reject GICP",
            None,
            pairwise,
        )

    return ConsensusDecision(
        MANUAL_RELOCALIZATION_REQUIRED,
        "the three pose candidates do not form a trustworthy agreeing pair",
        None,
        pairwise,
    )
