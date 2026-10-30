"""정책 입출력 기록 -- 모델이 **실제로 본 숫자와 낸 숫자**를 JSON 으로 남김.

    t                  첫 정책 호출부터 흐른 초
    obs/...            모델 입력 그대로 (정규화 전, 라디안 단위)
    act/<관절>         모델 출력 그대로 (배율·기본자세 입히기 전)
    tgt/<관절>         관절 목표각 (도). 실제로 루프에 넘긴 것
    pos/<관절>         그 순간의 실측 관절각 (도)
    extra/<모터>/<값>  추가 값 (iq A, temp C, vbus V). `--extra` 를 켰을 때만 (`track_log`)

`tools/policy_viewer.html` 로 열면 재생하듯 볼 수 있음. 파일 모양은 `run_name.write`
가 정함 -- `columns` 에 열 이름, `rows` 에 한 주기 한 줄, `limits` 에 관절 가동범위,
`run` 에 이 실행의 인자.


## 두 군데를 감쌈

    wrap_model    벡터 -> 행동. obs 와 act 를 잡음
    wrap_motion   (t, 관찰) -> 목표. tgt 와 pos 를 잡고 한 줄을 완성함

모델만 감싸면 목표각과 실측각을 모름 -- 그 둘은 관절 이름이 붙은 사전에만 있음.
motion 만 감싸면 모델이 본 벡터를 모름. 그래서 둘 다 감싸고, motion 쪽에서 한 줄로
합침.


## 메모리에 쌓고 끝날 때 씀

제어 루프 안에서 파일을 쓰지 않음. 50Hz 로 한 시간이면 18만 줄 x 80칸 정도라
메모리로 충분함. 대신 **비정상 종료면 남지 않음** -- `run.py` 가 `finally` 에서
`save()` 를 부름.

관측이 제어를 멈추면 안 되므로 기록 중 예외는 세기만 함.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence

import numpy as np

from . import run_name
from .track_log import extra_values, joint_limits

logger = logging.getLogger(__name__)


class PolicyLog:
    """정책 한 번 실행의 입출력 기록."""

    def __init__(
        self,
        obs_names: Sequence[str],
        order: Sequence[str],
        *,
        robot: Any = None,
        limits: Optional[Mapping[str, Sequence[float]]] = None,
    ) -> None:
        self.obs_names = tuple(obs_names)
        self.limits = (
            joint_limits(robot) if limits is None
            else {k: (float(v[0]), float(v[1])) for k, v in limits.items()}
        )
        """관절 가동범위. 파일의 `limits` 칸으로 남아 뷰어가 정규화에 씀.

        `robot` 을 주면 그 로봇에서 읽음 (`huphy-run`). 로봇 객체가 없는 쪽은
        `limits` 로 직접 줌 -- ROS `log_node` 는 모터가 다른 기계에 있어서
        캘리브레이션 파일을 못 읽고, `control_node` 가 토픽으로 보낸 값을 씀.
        """
        self.order = tuple(order)
        self.base_columns = (
            ("t",)
            + self.obs_names
            + tuple(f"act/{j}" for j in self.order)
            + tuple(f"tgt/{j}" for j in self.order)
            + tuple(f"pos/{j}" for j in self.order)
        )
        self.extra_cols: Optional[tuple] = None
        """`extra/...` 열. 첫 줄의 관찰로 정해짐 -- `--extra` 를 안 켰으면 빈 것."""
        self.rows: List[List[float]] = []
        self.errors = 0
        self._t0: Optional[float] = None
        self._pending: Optional[List[float]] = None

    def __len__(self) -> int:
        return len(self.rows)

    @property
    def columns(self) -> tuple:
        return self.base_columns + tuple(self.extra_cols or ())

    # ---- 감싸기 -----------------------------------------------------------
    def wrap_model(self, model: Callable[[np.ndarray], Sequence[float]]):
        """모델을 감쌈. 입력 벡터와 출력을 다음 `wrap_motion` 줄에 넘김."""
        n_obs, n_act = len(self.obs_names), len(self.order)

        def logged(vector: np.ndarray):
            action = model(vector)
            try:
                obs = [float(v) for v in np.asarray(vector).reshape(-1)]
                act = [float(v) for v in np.asarray(action).reshape(-1)]
                if len(obs) != n_obs or len(act) != n_act:
                    raise ValueError(
                        f"기록 칸이 안 맞음: obs {len(obs)}/{n_obs}, act {len(act)}/{n_act}"
                    )
                self._pending = obs + act
            except Exception as e:
                self._fail(e)
            return action

        for attr in ("obs_dim", "action_dim"):
            if hasattr(model, attr):
                setattr(logged, attr, getattr(model, attr))
        return logged

    def wrap_motion(self, motion: Callable[[float, Dict[str, Any]], Optional[Dict[str, float]]]):
        """정책 motion 을 감쌈. 목표와 실측을 붙여 한 줄을 완성함."""

        def logged(t: float, observation: Dict[str, Any]):
            targets = motion(t, observation)
            try:
                self._finish(targets or {}, observation)
            except Exception as e:
                self._fail(e)
            return targets

        return logged

    def _finish(self, targets: Dict[str, float], observation: Dict[str, Any]) -> None:
        pending, self._pending = self._pending, None
        if pending is None:
            return                          # 모델이 안 불린 주기 -- 남길 것이 없음
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        n_obs = len(self.obs_names)
        self._append(
            now - self._t0,
            pending[:n_obs],
            pending[n_obs:],
            [targets.get(j, float("nan")) for j in self.order],
            [observation.get(f"{j}.pos", float("nan")) for j in self.order],
            extra_values(observation),
        )

    # ---- 줄 만들기 ---------------------------------------------------------
    def _append(self, t, obs, act, tgt, pos, extra: Mapping[str, float]) -> None:
        """한 줄을 `rows` 에 넣음. **열 순서를 정하는 곳은 여기 하나임.**

        `base_columns` 와 같은 순서여야 함 -- `t`, `obs/*`, `act/*`, `tgt/*`,
        `pos/*`, `extra/*`. 들어오는 길이 둘(`_finish` 와 `add_row`)이 같은 파일을
        내야 하므로 조립은 한 곳에서만 함.

        `extra_cols` 는 첫 줄의 `extra` 로 정해짐. `--extra` 를 안 켰으면 빈 것임.
        """
        row = [float(t)] + [float(v) for v in obs] + [float(v) for v in act]
        row += [float(v) for v in tgt]
        row += [float(v) for v in pos]
        if self.extra_cols is None:
            self.extra_cols = tuple(extra)
        row += [float(extra.get(c, float("nan"))) for c in self.extra_cols]
        self.rows.append(row)

    def add_row(self, t: float, obs, action, target_deg, measured_deg) -> None:
        """완성된 한 줄을 그대로 넣음. **감싸는 것 없이 밖에서 넣는 입구.**

        `wrap_model`/`wrap_motion` 은 모델과 motion 호출을 가로채 줄을 만드는데,
        가로챌 것이 없는 쪽(ROS `log_node` -- 토픽으로 숫자를 받음)이 쓸 입구가
        필요해서 둠. 같은 `_append` 를 거치므로 두 길이 같은 파일을 냄.

            t             초. **부르는 쪽이 찍은 상대 시각**을 그대로 씀. 여기서
                          `time.monotonic()` 을 찍으면 추론한 기계의 주기 지터가
                          아니라 이 프로세스의 수신 지터가 됨
            obs           `obs_names` 개수. 정규화 전
            action        `order` 개수. 모델 출력 날값
            target_deg    `order` 개수. 관절 목표각 (도)
            measured_deg  `order` 개수. 실측 관절각 (도)

        길이가 다르면 멈춤 -- 그대로 넣으면 열이 밀려 뷰어가 다른 관절의 그래프를
        그림. `extra/` 는 이 길로는 안 들어감 (`--extra` 가 ROS 쪽에 없음).
        """
        n_obs, n_act = len(self.obs_names), len(self.order)
        sizes = ((obs, n_obs, "obs"), (action, n_act, "action"),
                 (target_deg, n_act, "target_deg"), (measured_deg, n_act, "measured_deg"))
        bad = [f"{name} {len(v)}/{want}" for v, want, name in sizes if len(v) != want]
        if bad:
            raise ValueError(f"기록 칸이 안 맞음: {', '.join(bad)}")
        self._append(t, obs, action, target_deg, measured_deg, {})

    def _fail(self, e: Exception) -> None:
        self.errors += 1
        if self.errors == 1:
            logger.warning("정책 기록 실패 (제어는 계속함): %s", e)

    # ---- 저장 -------------------------------------------------------------
    def save(self, path: "str | Path", run: Mapping[str, Any] = {}) -> Path:
        """JSON 으로 씀. 폴더가 없으면 만듦.

        `run` 은 `run_name.header(...)` 가 만든 사전. 이름에서 뺀 값(초 단위 시각·
        체크포인트·기본값인 인자)이 거기 들어감.
        """
        # JSON 에 NaN 을 못 넣으므로 없는 값은 null 로. 자리수는 6 -- 모터
        # 해상도(0.022도)보다 촘촘해서 정보를 안 잃음.
        rows = [[None if v != v else round(float(v), 6) for v in row] for row in self.rows]
        return run_name.write(
            path, run,
            limits={k: [lo, hi] for k, (lo, hi) in self.limits.items()} or None,
            columns=list(self.columns),
            rows=rows,
        )


def default_path(directory: "str | Path", policy_name: str, parts=()) -> Path:
    """`<폴더>/policy/<월일>_policy_<이름>[_<인자들>].json`. 규칙은 `run_name` 에 있음."""
    return run_name.path(directory, "policy", policy_name, parts)
