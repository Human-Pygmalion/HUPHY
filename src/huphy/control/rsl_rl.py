"""rsl_rl 체크포인트(`.pt`)를 읽어 부를 수 있는 함수로 만듦.

**torch 없이 numpy 로만 함.** 신경망이 완전연결 4층이라 곱셈 5만 번이면 끝나고,
제어 루프가 도는 라즈베리파이에 torch(200MB+)를 넣을 이유가 없음.

`.pt` 는 zip 임. 안에 pickle 하나와 float32 원본 덩어리들이 들어 있음.

    model_34699/data.pkl     텐서 이름 -> (크기, 어느 덩어리의 몇 번째부터)
    model_34699/data/0,1,..  float32 원본

pickle 을 풀 때 torch 클래스를 만나므로, **텐서를 만드는 자리만 가로채** 크기와
위치만 받아 두고 나머지 클래스는 껍데기로 대체함.


## 무엇을 꺼내나

    mlp.0 / mlp.2 / mlp.4 / mlp.6      완전연결 4층. 사이에 ELU
    obs_normalizer._mean / ._std       학습 중에 쌓인 값

층 크기는 파일에서 읽으므로 학습 쪽에서 층을 바꿔도 그대로 따라감.

`distribution.std_param` 은 학습 때 탐색용이라 안 씀 -- 실물에서는 신경망 출력을
그대로 행동으로 씀.

`critic_state_dict` 도 안 씀. 학습에만 쓰이고 입력 항목이 다름.


## 정규화를 빼먹으면 안 됨

    정규화된 입력 = (관찰 - mean) / std

`mean`/`std` 는 학습 중 관찰의 통계임. 예를 들어 중력 z 성분의 평균이 -1.74 이고
표준편차가 8.8 같은 식임. 빼먹으면 신경망이 전혀 다른 크기의 값을 받아 **에러 없이
엉뚱한 행동을 냄.**
"""

from __future__ import annotations

import io
import pickle
import zipfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .policy import PolicySpec

ACTOR_KEY = "actor_state_dict"
LAYER_PREFIX = "mlp."


def _rebuild_tensor(storage, offset, size, stride, *rest) -> Dict[str, Any]:
    """torch 가 텐서를 만드는 자리. **만들지 않고 위치만 받아 둠.**"""
    return {"storage": storage, "offset": int(offset), "size": tuple(size)}


class _Ignored:
    """torch 쪽 클래스 자리. 값이 필요 없는 것들임."""

    def __init__(self, *args, **kwargs) -> None:
        pass


class _Unpickler(pickle.Unpickler):
    """torch 없이 `data.pkl` 을 푸는 것.

    `collections`/`builtins` 는 그대로 씀 -- state_dict 가 `OrderedDict` 임.
    나머지 클래스는 껍데기로 바꿔치기함.
    """

    def find_class(self, module: str, name: str):
        if name in ("_rebuild_tensor_v2", "_rebuild_tensor"):
            return _rebuild_tensor
        if module in ("collections", "builtins", "__builtin__"):
            return super().find_class(module, name)
        return _Ignored

    def persistent_load(self, pid) -> Dict[str, Any]:
        # ('storage', dtype, 덩어리 이름, 장치, 원소 수)
        return {"key": pid[2], "count": pid[4]}


def _read(path: "str | Path") -> Tuple[Dict[str, Any], Any, str]:
    """`.pt` 를 열어 (텐서 표, zip, 안쪽 폴더 이름) 을 냄."""
    archive = zipfile.ZipFile(str(path))
    names = [n for n in archive.namelist() if n.endswith("data.pkl")]
    if not names:
        raise ValueError(f"{path}: data.pkl 이 없음. rsl_rl 체크포인트가 아님")
    root = names[0].rsplit("/", 1)[0]
    loaded = _Unpickler(io.BytesIO(archive.read(names[0]))).load()

    if not isinstance(loaded, dict) or ACTOR_KEY not in loaded:
        raise ValueError(
            f"{path}: {ACTOR_KEY} 가 없음 (있는 것: {sorted(loaded) if isinstance(loaded, dict) else type(loaded).__name__})"
        )
    return loaded[ACTOR_KEY], archive, root


def _array(entry: Dict[str, Any], archive: Any, root: str) -> np.ndarray:
    """텐서 하나를 numpy 로. float32 원본을 잘라 모양만 맞춤."""
    raw = archive.read(f"{root}/data/{entry['storage']['key']}")
    flat = np.frombuffer(raw, dtype=np.float32)
    count = int(np.prod(entry["size"])) if entry["size"] else 1
    start = entry["offset"]
    return flat[start:start + count].reshape(entry["size"]).astype(np.float32)


def _layers(state: Dict[str, Any], archive: Any, root: str) -> List[Tuple[np.ndarray, np.ndarray]]:
    """`mlp.N.weight` / `mlp.N.bias` 짝을 번호 순으로. 층 크기는 파일이 정함."""
    numbers = sorted(
        {int(k.split(".")[1]) for k in state if k.startswith(LAYER_PREFIX)}
    )
    out = []
    for n in numbers:
        weight = state.get(f"{LAYER_PREFIX}{n}.weight")
        bias = state.get(f"{LAYER_PREFIX}{n}.bias")
        if weight is None or bias is None:
            raise ValueError(f"mlp.{n} 의 weight 나 bias 가 없음")
        out.append((_array(weight, archive, root), _array(bias, archive, root)))
    if not out:
        raise ValueError("mlp.* 층이 하나도 없음")
    return out


def _elu(x: np.ndarray) -> np.ndarray:
    """ELU. 학습 설정(`rl_cfg.py`)의 `activation="elu"` 임.

        x > 0 이면 x, 아니면 exp(x) - 1
    """
    return np.where(x > 0.0, x, np.expm1(np.minimum(x, 0.0)))


def load(path: "str | Path", *, spec: PolicySpec, action_dim: Optional[int] = None):
    """체크포인트를 읽어 **벡터를 받아 행동을 내는 함수**를 냄.

    정규화까지 안에서 함 -- 부르는 쪽은 관찰 벡터를 그대로 넘기면 됨.

    입력 개수가 `spec` 과 다르면 여기서 멈춤. 관찰 구성이 학습 때와 달라진 것이고,
    그대로 두면 신경망이 엉뚱한 자리의 숫자를 읽음.

    `action_dim` 을 주면 **출력 개수도** 대조함. 다리 하나 모델(6)에 양다리 순서(12)
    를 붙이거나 그 반대면 여기서 멈춤 -- 그대로 두면 `joint_targets` 에서야 걸리는데
    그때는 이미 토크가 들어간 뒤임.
    """
    state, archive, root = _read(path)
    layers = _layers(state, archive, root)

    obs_dim = layers[0][0].shape[1]
    out_dim = layers[-1][0].shape[0]
    if obs_dim != spec.obs_dim:
        raise ValueError(
            f"{path}: 신경망 입력이 {obs_dim}개인데 {spec.name} 은 {spec.obs_dim}개임. "
            f"정책 이름이 파일과 맞는지 볼 것"
        )
    if action_dim is not None and out_dim != action_dim:
        raise ValueError(
            f"{path}: 신경망 출력이 {out_dim}개인데 관절 순서는 {action_dim}개임. "
            f"다리 하나 모델인지 양다리 모델인지 볼 것 (--robot)"
        )

    mean = _array(state["obs_normalizer._mean"], archive, root).reshape(-1)
    std = _array(state["obs_normalizer._std"], archive, root).reshape(-1)
    if mean.size != obs_dim or std.size != obs_dim:
        raise ValueError(
            f"{path}: 정규화 값이 {mean.size}개인데 입력은 {obs_dim}개임"
        )
    # 0으로 나누는 것을 막음. 학습 중 한 번도 안 변한 항목이 여기 해당함.
    std = np.where(std > 1e-6, std, 1.0)

    def model(vector: np.ndarray) -> np.ndarray:
        x = (np.asarray(vector, dtype=np.float32) - mean) / std
        for weight, bias in layers[:-1]:
            x = _elu(weight @ x + bias)
        weight, bias = layers[-1]
        return weight @ x + bias

    model.obs_dim = obs_dim          # type: ignore[attr-defined]
    model.action_dim = out_dim       # type: ignore[attr-defined]
    return model
