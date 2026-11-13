#!/usr/bin/env python3
"""WCM1600 전류 센서 실시간 표시 (UART 9600 8N1, Continuous 모드).

프레임: 부호 1바이트 + 숫자 5자리 + '\r\n' (총 8바이트)
  '~' = AC, '+' = DC(+), '-' = DC(-)
  숫자 5자리 = mA 단위  (예: '01230' -> 1230 mA = 1.230 A)

사용법:  ./wcm1600.py [포트]    (종료: Ctrl+C)
"""
import glob
import os
import sys
import termios
import time

BAUD = termios.B9600


def find_port():
    ports = sorted(glob.glob("/dev/serial/by-id/*FTDI*")) or sorted(glob.glob("/dev/ttyUSB*"))
    if not ports:
        sys.exit("시리얼 장치를 찾을 수 없습니다. USB 연결을 확인하세요.")
    return ports[0]


def open_port(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = (cflag & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)) \
        | termios.CS8 | termios.CREAD | termios.CLOCAL
    cc[termios.VMIN] = 1
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, BAUD, BAUD, cc])
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def parse(line):
    """b'+01020' -> ('DC', 1.02) / 형식이 틀리면 None"""
    if len(line) != 6 or not line[1:].isdigit():
        return None
    sign, digits = chr(line[0]), line[1:].decode()
    value = int(digits) / 1000.0  # 5자리 = mA (Winson 공식 라이브러리와 동일)
    if sign == "~":
        return "AC", value
    if sign == "+":
        return "DC", value
    if sign == "-":
        return "DC", -value
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    fd = open_port(port)
    print(f"포트: {port}  (9600 8N1)   종료: Ctrl+C\n")
    buf = b""
    first = True  # 시작 시 중간부터 읽힌 첫 줄은 버림
    try:
        while True:
            buf += os.read(fd, 64)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                if first:
                    first = False
                    continue
                result = parse(raw.strip())
                ts = time.strftime("%H:%M:%S")
                if result:
                    kind, amps = result
                    print(f"{ts}  {kind}  {amps:+8.3f} A")
                elif raw.strip():
                    print(f"{ts}  (알 수 없는 데이터: {raw!r})")
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
