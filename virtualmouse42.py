import cv2
import mediapipe as mp
import math
import ctypes
import time
import threading

user32 = ctypes.windll.user32
screen_width = user32.GetSystemMetrics(0)
screen_height = user32.GetSystemMetrics(1)

def get_cursor_pos():
    class POINT(ctypes.Structure):
        _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]
    pt = POINT()
    ctypes.windll.user32.GetCursorPos(ctypes.byref(pt))
    return pt.x, pt.y

def move_mouse(x, y):
    ctypes.windll.user32.SetCursorPos(int(x), int(y))

def mouse_down(button='left'):
    if button == 'left':
        ctypes.windll.user32.mouse_event(2, 0, 0, 0, 0)
    else:
        ctypes.windll.user32.mouse_event(8, 0, 0, 0, 0)

def mouse_up(button='left'):
    if button == 'left':
        ctypes.windll.user32.mouse_event(4, 0, 0, 0, 0)
    else:
        ctypes.windll.user32.mouse_event(16, 0, 0, 0, 0)

def right_click():
    ctypes.windll.user32.mouse_event(8, 0, 0, 0, 0)
    ctypes.windll.user32.mouse_event(16, 0, 0, 0, 0)

cursor_pos = (screen_width / 2, screen_height / 2)
lock = threading.Lock()
click_queue = []
enable_virtual_mouse = True
pinch_active = False
middle_pinch_active = False
hand_detected = False

PINCH_FRAMES = 5
RELEASE_FRAMES = 2 # release thresh rate
PINCH_REL_THRESH = 0.11 # adjust for pinch dist 
MIDDLE_PINCH_REL_THRESH = 0.20
SMOOTHING = 3.5 # below 4 adj
MOVE_INTERVAL = 0.01
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
VIRTUAL_MOUSE_WEIGHT = 0.7

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1,
                       min_detection_confidence=0.85,
                       min_tracking_confidence=0.85,
                       model_complexity=1)

def cursor_loop():
    last_move_time = 0
    while True:
        if enable_virtual_mouse and time.time() - last_move_time > MOVE_INTERVAL:
            phys_x, phys_y = get_cursor_pos()
            with lock:
                if hand_detected:
                    blend_x = phys_x * (1 - VIRTUAL_MOUSE_WEIGHT) + cursor_pos[0] * VIRTUAL_MOUSE_WEIGHT
                    blend_y = phys_y * (1 - VIRTUAL_MOUSE_WEIGHT) + cursor_pos[1] * VIRTUAL_MOUSE_WEIGHT
                    move_mouse(blend_x, blend_y)
            last_move_time = time.time()
        time.sleep(0.001)

threading.Thread(target=cursor_loop, daemon=True).start()

def click_loop():
    while True:
        if enable_virtual_mouse and click_queue:
            action = click_queue.pop(0)
            if action == 'left_down':
                mouse_down('left')
            elif action == 'left_up':
                mouse_up('left')
            elif action == 'right_click':
                right_click()
        time.sleep(0.001)

threading.Thread(target=click_loop, daemon=True).start()

frame_lock = threading.Lock()
latest_frame = None

def capture_loop():
    global latest_frame
    cap = cv2.VideoCapture(0) # cam number (if external. index from 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while True:
        ret, frame = cap.read()
        if ret:
            frame = cv2.flip(frame, 1)
            with frame_lock:
                latest_frame = frame

threading.Thread(target=capture_loop, daemon=True).start()

prev_x, prev_y = 0.0, 0.0
pinch_counter = 0
release_counter = 0
middle_pinch_counter = 0
middle_release_counter = 0
last_click_time = 0
last_right_click_time = 0
SHOW_DEBUG = True

print("Virtual Mouse: ENABLED (working alongside physical mouse)")
print("Left click: Thumb and index finger pinch")
print("Right click: Thumb and middle finger pinch")
print("Press 'q' to quit")

cv2.namedWindow("Virtual Mouse", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Virtual Mouse", FRAME_WIDTH, FRAME_HEIGHT)
cv2.moveWindow("Virtual Mouse", screen_width - FRAME_WIDTH - 10, 10)

try:
    hwnd = user32.FindWindowW(None, "Virtual Mouse")
    user32.SetWindowPos(hwnd, -1, 0, 0, 0, 0, 0x0001 | 0x0002 | 0x0020)
except:
    print("Could not set window to always on top!")

while True:
    frame = None
    with frame_lock:
        if latest_frame is not None:
            frame = latest_frame.copy()
    if frame is None:
        time.sleep(0.001)
        continue

    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    hand_detected = False

    if results.multi_hand_landmarks and enable_virtual_mouse:
        hand_detected = True
        lm = results.multi_hand_landmarks[0].landmark
        ix_n, iy_n = lm[8].x, lm[8].y
        tx_n, ty_n = lm[4].x, lm[4].y
        mx_n, my_n = lm[12].x, lm[12].y
        index_pinch_dist = math.hypot(tx_n - ix_n, ty_n - iy_n)
        middle_pinch_dist = math.hypot(tx_n - mx_n, ty_n - my_n)

        xs = [p.x for p in lm]
        ys = [p.y for p in lm]
        hand_size = max(math.hypot(max(xs) - min(xs), max(ys) - min(ys)), 1e-6)

        index_pinch_ratio = index_pinch_dist / hand_size
        middle_pinch_ratio = middle_pinch_dist / hand_size

        if index_pinch_ratio < PINCH_REL_THRESH:
            pinch_counter += 1
            release_counter = 0
        else:
            release_counter += 1
            pinch_counter = 0

        if middle_pinch_ratio < MIDDLE_PINCH_REL_THRESH:
            middle_pinch_counter += 1
            middle_release_counter = 0
        else:
            middle_release_counter += 1
            middle_pinch_counter = 0

        now = time.time()
        if pinch_counter >= PINCH_FRAMES and not pinch_active:
            if now - last_click_time > 0.05:
                click_queue.append('left_down')
                pinch_active = True
                last_click_time = now
                pinch_counter = 0

        if release_counter >= RELEASE_FRAMES and pinch_active:
            if now - last_click_time > 0.05:
                click_queue.append('left_up')
                pinch_active = False
                last_click_time = now
                release_counter = 0

        if middle_pinch_counter >= PINCH_FRAMES and not middle_pinch_active:
            if now - last_right_click_time > 0.05:
                click_queue.append('right_click')
                middle_pinch_active = True
                last_right_click_time = now
                middle_pinch_counter = 0

        if middle_release_counter >= RELEASE_FRAMES and middle_pinch_active:
            middle_pinch_active = False
            middle_release_counter = 0

        screen_x = ix_n * screen_width
        screen_y = iy_n * screen_height
        smooth_x = prev_x + (screen_x - prev_x) / SMOOTHING
        smooth_y = prev_y + (screen_y - prev_y) / SMOOTHING
        prev_x, prev_y = smooth_x, smooth_y
        with lock:
            cursor_pos = (smooth_x, smooth_y)

        if SHOW_DEBUG:
            h, w, _ = frame.shape
            for id, landmark in enumerate(lm):
                if id in [4, 8, 12]:
                    cx, cy = int(landmark.x * w), int(landmark.y * h)
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), cv2.FILLED)

            cv2.putText(frame, f"Index pinch: {index_pinch_ratio:.2f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0) if index_pinch_ratio < PINCH_REL_THRESH else (0, 160, 255), 1)
            cv2.putText(frame, f"Middle pinch: {middle_pinch_ratio:.2f}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0) if middle_pinch_ratio < MIDDLE_PINCH_REL_THRESH else (0, 160, 255), 1)
            cv2.putText(frame, f"Left: {'PINCHED' if pinch_active else 'OPEN'}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255) if pinch_active else (255, 255, 255), 1)
            cv2.putText(frame, f"Right: {'PINCHED' if middle_pinch_active else 'OPEN'}", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 255) if middle_pinch_active else (255, 255, 255), 1)
            cv2.putText(frame, f"virtual weight: {VIRTUAL_MOUSE_WEIGHT:.1f}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (200, 200, 200), 1)
    else:
        pinch_active = False
        middle_pinch_active = False
        pinch_counter = 0
        release_counter = 0
        middle_pinch_counter = 0
        middle_release_counter = 0

    status_text = "Virtual Mouse: ACTIVE (with physical mouse)"
    cv2.putText(frame, status_text, (10, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    help_text = "Left: Thumb+Index | Right: Thumb+Middle"
    cv2.putText(frame, help_text, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    detection_status = "Hand: DETECTED" if hand_detected else "Hand: NOT DETECTED"
    cv2.putText(frame, detection_status, (10, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 0) if hand_detected else (0, 0, 255), 1)

    cv2.imshow("Virtual Mouse", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
