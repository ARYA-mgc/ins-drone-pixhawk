#!/usr/bin/env python3
"""
ekf3_blender.py
===============
Weighted blending between NavCore ESKF and ArduPilot EKF3 outputs.

When both estimators are running, this module can compute a weighted
average based on each estimator's covariance (confidence), producing
a fused output that is more robust than either alone.

Blending modes:
  - ESKF_ONLY:  Use NavCore ESKF exclusively (default).
  - EKF3_ONLY:  Use ArduPilot EKF3 exclusively.
  - WEIGHTED:   Covariance-weighted blend of both.
  - CROSSCHECK: Use ESKF but alarm if EKF3 diverges too far.

Status: STUB — interface defined, implementation pending.
"""

import logging
import numpy as np
from enum import Enum, auto
from typing import Optional

log = logging.getLogger("ekf3_blender")


class BlendMode(Enum):
    ESKF_ONLY  = auto()
    EKF3_ONLY  = auto()
    WEIGHTED   = auto()
    CROSSCHECK = auto()


class EKF3Blender:
    """
    Manages blending and cross-checking between ESKF and EKF3.
    """

    # Divergence thresholds for CROSSCHECK mode
    POS_DIVERGE_WARN = 2.0   # meters
    POS_DIVERGE_FAULT = 10.0
    ATT_DIVERGE_WARN = 10.0  # degrees
    ATT_DIVERGE_FAULT = 30.0

    def __init__(self, mode: BlendMode = BlendMode.ESKF_ONLY):
        self._mode = mode
        self._last_ekf3_state = None
        self._last_eskf_state = None
        self._divergence_count = 0

        log.info(f"EKF3 blender mode: {mode.name}")

    @property
    def mode(self) -> BlendMode:
        return self._mode

    def set_mode(self, mode: BlendMode):
        self._mode = mode
        log.info(f"EKF3 blender mode changed: {mode.name}")

    def update_eskf(self, pos: np.ndarray, vel: np.ndarray,
                    euler: np.ndarray, P: np.ndarray):
        """Update with latest ESKF state."""
        self._last_eskf_state = {
            "pos": pos.copy(),
            "vel": vel.copy(),
            "euler": euler.copy(),
            "P_diag": np.diag(P).copy(),
        }

    def update_ekf3(self, pos: np.ndarray, vel: np.ndarray,
                    euler: np.ndarray,
                    confidence: float = 1.0):
        """
        Update with latest ArduPilot EKF3 state.

        Typically extracted from GLOBAL_POSITION_INT or
        LOCAL_POSITION_NED MAVLink messages.
        """
        self._last_ekf3_state = {
            "pos": pos.copy(),
            "vel": vel.copy(),
            "euler": euler.copy(),
            "confidence": confidence,
        }

    def get_blended_output(self) -> Optional[dict]:
        """
        Compute the blended output based on current mode.

        Returns:
            Dict with 'pos', 'vel', 'euler', or None if data missing.
        """
        eskf = self._last_eskf_state
        ekf3 = self._last_ekf3_state

        if self._mode == BlendMode.ESKF_ONLY:
            return eskf

        if self._mode == BlendMode.EKF3_ONLY:
            return ekf3

        if eskf is None or ekf3 is None:
            # Fall back to whichever is available
            return eskf if eskf is not None else ekf3

        if self._mode == BlendMode.CROSSCHECK:
            return self._crosscheck(eskf, ekf3)

        if self._mode == BlendMode.WEIGHTED:
            return self._weighted_blend(eskf, ekf3)

        return eskf

    def _crosscheck(self, eskf: dict, ekf3: dict) -> dict:
        """
        Use ESKF output but alarm if EKF3 diverges significantly.
        """
        pos_diff = np.linalg.norm(eskf["pos"] - ekf3["pos"])
        att_diff = np.degrees(np.linalg.norm(eskf["euler"] - ekf3["euler"]))

        if pos_diff > self.POS_DIVERGE_FAULT or att_diff > self.ATT_DIVERGE_FAULT:
            self._divergence_count += 1
            log.error(f"ESKF/EKF3 DIVERGENCE FAULT: "
                      f"pos={pos_diff:.2f}m att={att_diff:.1f}deg")
        elif pos_diff > self.POS_DIVERGE_WARN or att_diff > self.ATT_DIVERGE_WARN:
            log.warning(f"ESKF/EKF3 divergence: "
                        f"pos={pos_diff:.2f}m att={att_diff:.1f}deg")

        return eskf  # Always use ESKF in crosscheck mode

    def _weighted_blend(self, eskf: dict, ekf3: dict) -> dict:
        """
        Covariance-weighted blend.

        TODO: Implement proper covariance intersection or
              inverse-variance weighting.
        """
        # Placeholder: simple average
        return {
            "pos": (eskf["pos"] + ekf3["pos"]) / 2.0,
            "vel": (eskf["vel"] + ekf3["vel"]) / 2.0,
            "euler": (eskf["euler"] + ekf3["euler"]) / 2.0,
        }
