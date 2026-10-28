"""파이썬 `logging` 을 ROS 로그로 넘김.

huphy 라이브러리는 ROS 를 모르고 `logging.getLogger(__name__)` 으로 남김. 노드 안에서
아무것도 안 하면 INFO 이하는 버려지고 WARNING 이상만 형식 없이 stderr 로 나감 --
출처가 안 보이고, `/rosout` · `~/.ros/log` · rqt_console 에도 안 남음.

노드가 시작할 때 `forward()` 를 부르면 모든 파이썬 로그가 노드 logger 의 자식으로 나감:

    huphy.motors.canbus  ->  [huphy_control.huphy.motors.canbus]

ROS 2 Humble 에는 이 다리가 없어서 `logging.Handler` 로 직접 잇는 것이 보통임.


## 레벨

파이썬 쪽 문턱은 노드 logger 의 레벨을 따름. `--ros-args --log-level debug` 를 주면
라이브러리도 DEBUG 까지 나옴. 그보다 높은 레벨은 모듈마다 따로 올릴 수 있음:

    --ros-args --log-level huphy_control.huphy.motors.canbus:=warn

(파이썬 문턱이 노드 레벨에 맞춰져 있으므로, 노드보다 **낮게** 내리는 것은 안 됨.)


## 레벨마다 다른 줄에서 부르는 이유

rclpy logger 는 부른 자리(파일·줄)마다 레벨을 처음 것으로 고정하고, 다른 레벨이 오면
`ValueError` 를 냄. 그래서 레벨마다 부르는 줄을 나누고, 이름마다 logger 를 따로 둠.
"""

from __future__ import annotations

import logging
from typing import Dict

from rclpy.impl.rcutils_logger import RcutilsLogger


class RosLogHandler(logging.Handler):
    """파이썬 로그 한 줄을 `<노드>.<파이썬 logger 이름>` ROS logger 로 보냄."""

    def __init__(self, parent: RcutilsLogger) -> None:
        super().__init__()
        self.parent = parent
        self._children: Dict[str, RcutilsLogger] = {}
        # 시간·레벨·이름은 ROS 가 붙임. 예외가 있으면 traceback 은 format 이 붙여줌.
        self.setFormatter(logging.Formatter("%(message)s"))

    def _child(self, name: str) -> RcutilsLogger:
        log = self._children.get(name)
        if log is None:
            log = self._children[name] = self.parent.get_child(name)
        return log

    def emit(self, record: logging.LogRecord) -> None:
        try:
            log = self._child(record.name)
            msg = self.format(record)
            level = record.levelno
            if level >= logging.CRITICAL:
                log.fatal(msg)
            elif level >= logging.ERROR:
                log.error(msg)
            elif level >= logging.WARNING:
                log.warning(msg)
            elif level >= logging.INFO:
                log.info(msg)
            else:
                log.debug(msg)
        except Exception:
            self.handleError(record)


def forward(node_logger: RcutilsLogger) -> RosLogHandler:
    """루트 파이썬 logger 에 다리를 붙임. 라이브러리가 로그를 남기기 전에 부를 것."""
    handler = RosLogHandler(node_logger)
    root = logging.getLogger()
    root.addHandler(handler)
    # LoggingSeverity 와 logging 의 숫자는 같음 (DEBUG 10 ... FATAL 50).
    root.setLevel(int(node_logger.get_effective_level()))
    # warnings.warn(...) 도 로그로 모음 (py.warnings).
    logging.captureWarnings(True)
    return handler


def stop(handler: RosLogHandler) -> None:
    """다리를 뗌. `rclpy.shutdown()` 전에 부를 것."""
    logging.getLogger().removeHandler(handler)
    logging.captureWarnings(False)
