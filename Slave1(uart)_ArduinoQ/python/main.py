import time
from datetime import datetime, UTC
from threading import Lock

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.video_objectdetection import VideoObjectDetection


TARGET_LABEL = "chichi"   # 인형 라벨
FRAME_W = 416

START_DELAY = 3.0         # 앱 시작 후 인식 결과를 쓰기 시작하는 시간
HOLD_SEC = 0.5            # 마지막으로 본 뒤 이 시간 동안은 "보이는 중"으로 유지
HEARTBEAT_SEC = 0.3       # 결과가 같아도 이 주기로 MCU에 다시 전송
LOOP_SEC = 0.1
PRINT_BOX = False         # True면 인형이 보일 때마다 BOX 로그 출력

STATE_NAMES = {0: "RUN", 1: "LIMIT", 2: "STOP", 3: "WAIT_G", 4: "NO_APP", 5: "ALERT"}

ui = WebUI()
detector = VideoObjectDetection(confidence=0.5, debounce_sec=0.0)

state_lock = Lock()
target_box = None
target_t = 0.0
frames = 0

start_t = time.monotonic()
last_sent = None
last_sent_t = 0.0
last_fps_t = start_t


def area(box):
    x1, y1, x2, y2 = box
    return max(0, x2 - x1) * max(0, y2 - y1)


def steer_error(box):
    x1, _, x2, _ = box
    center_x = (x1 + x2) / 2.0
    error = (center_x - FRAME_W / 2.0) / (FRAME_W / 2.0) * 100.0
    return max(-100, min(100, round(error)))


def on_detections(detections: dict):
    global target_box, target_t, frames

    now = time.monotonic()
    items = detections.get(TARGET_LABEL, [])
    box = None

    if items:
        selected = max(
            items,
            key=lambda item: area(item["bounding_box_xyxy"]),
        )
        box = tuple(selected["bounding_box_xyxy"])

    with state_lock:
        frames += 1
        if box is not None:
            target_box = box
            target_t = now

    if box is not None and PRINT_BOX:
        print(f"BOX,{box},err={steer_error(box)}", flush=True)

    timestamp = datetime.now(UTC).isoformat()

    for label, instances in detections.items():
        for item in instances:
            ui.send_message("detection", message={
                "content": label,
                "confidence": item["confidence"],
                "timestamp": timestamp,
            })


def on_threshold(sid, threshold):
    value = float(threshold)
    # 앱 시작 직후 인식 엔진이 준비되기 전 호출되면 에러가 나므로 무시
    try:
        detector.override_threshold(max(0.0, min(1.0, value)))
    except ConnectionRefusedError:
        print("threshold override skipped: detector not ready yet", flush=True)


# MCU -> Python : sketch.ino의 Bridge.notify("telemetry", ...)
def on_telemetry(dist: int, seen: int, state: int, speed: int, limit: int):
    # dist: 정상 측정값(mm) / 2000 = 멀리 있음(범위 밖) / -1 = 센서 오류
    ui.send_message("distance", message={"distance_mm": dist})
    ui.send_message("status", message={
        "seen": seen,
        "state": STATE_NAMES.get(state, str(state)),
        "speed": speed,
        "limit": limit,
    })

    print(
        f"TLM,seen={seen},state={STATE_NAMES.get(state, state)},"
        f"dist={dist},speed={speed},limit={limit}",
        flush=True,
    )


def loop():
    global last_sent, last_sent_t, frames, last_fps_t

    now = time.monotonic()

    with state_lock:
        box = target_box
        last_seen_t = target_t

    seen = (
        now - start_t >= START_DELAY
        and box is not None
        and now - last_seen_t < HOLD_SEC
    )

    err = steer_error(box) if seen else 0
    cmd = (int(seen), err)

    # Python -> MCU : sketch.ino의 set_target (0.3초마다, 생존 신호 겸용)
    if cmd != last_sent or now - last_sent_t >= HEARTBEAT_SEC:
        Bridge.notify("set_target", cmd[0], cmd[1])
        last_sent = cmd
        last_sent_t = now

    if now - last_fps_t >= 5.0:
        with state_lock:
            frame_count = frames
            frames = 0

        callback_hz = frame_count / (now - last_fps_t)
        print(f"DETECTION_CALLBACK_HZ={callback_hz:.1f}", flush=True)
        last_fps_t = now

    time.sleep(LOOP_SEC)


detector.on_detect_all(on_detections)
ui.on_message("override_th", on_threshold)
Bridge.provide("telemetry", on_telemetry)

App.run(user_loop=loop)
