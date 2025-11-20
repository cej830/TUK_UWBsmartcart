import threading
import queue
import socket
import time
import math
import serial
import sys
import numpy as np
import RPi.GPIO as GPIO
from collections import deque


# =================================================
# === 1. 설정 및 전역 변수 정의 (프로그램 최상단) ===
# =================================================

 #스레드 동시 종료를 위한 신호
shutdown_event = threading.Event()

#스레드간 데이터 교환을 위한 큐
command_queue =queue.Queue() # 앱 -> 하드웨어
data_queue = queue.Queue()  #하드웨어 -> 앱

#네트워크 변수
HOST = ''           # 모든 인터페이스에서 대기
PORT = 5000         # 포트 번호
DELAY = 0.05         # 상태 전송 주기 (초)

# 상태 변수 설정
obstacle_avoiding = False
direction_state = "STRAIGHT"  # or "LEFT", "RIGHT"
speed_level = "LOW"           # or "MID", "HIGH"
waiting_from_obstacle = False #  



# para
SPEEDS = {"LOW": 35, "MID": 50, "HIGH": 70}
DISTANCE_THRESHOLDS = [0.6, 2]  

#바퀴 최소 속도
MIN_SPEED = 35


# =================================================
# === GPIO 핀 설정
# =================================================


#초음파 핀 설정
#왼쪽
ECHO1 = 17
TRIG1 = 27

#가운데
TRIG2 = 20
ECHO2 = 21

#오른쪽
ECHO3 = 22
TRIG3 = 25


IN1, IN2, ENA = 24, 23, 18  #left wheel
IN3, IN4, ENB = 6, 5, 13  #right wheel

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([TRIG1], GPIO.OUT)
GPIO.setup([ECHO1], GPIO.IN)
GPIO.setup([TRIG2], GPIO.OUT)
GPIO.setup([ECHO2], GPIO.IN)
GPIO.setup([TRIG3], GPIO.OUT)
GPIO.setup([ECHO3], GPIO.IN)
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

pwm_left = GPIO.PWM(ENA, 100)
pwm_right = GPIO.PWM(ENB, 100)
pwm_left.start(0)
pwm_right.start(0)



# =================================================
# === 시리얼 포트 설정
# =================================================

# Serial port setup (UWB)
ser_A = serial.Serial('/dev/esp_A', 115200, timeout=1)  # anchor A
ser_B = serial.Serial('/dev/esp_B', 115200, timeout=1)  # anchor B


# =================================================
# === 하드웨어 실행 함수 정의
# =================================================

# Read Range form ESP32 
def parse_range_line(line) :
    
    # ex: "Range: 1.234 m  1715793840.123"

    try:
        
        line = line.strip()
        
        if "R:" in line or "Range:" in line:
            start = line.find("R:") + 2 if "R:" in line else line.find("Range:") +6
            end = line.find("m")if "m" in line else len(line)
            
            for char in ["@",""]:
                if char in line[start:end]:
                    end = line.find(char,start)
                    break
            
            distance = float(line[start:end])

            return distance
            
    except Exception as e:
        print(f"parse_error:{e}, data:'{line}'")
        return None

#-------------거리 보정치 함수

# Distance calibration functions 
def cal_A(x):
    x = np.float64(x)
    a = np.float64(0.034143)
    b = np.float64(-0.048561)
    c = np.float64(0.642064)
    d = np.float64(0.56679094)
    
    return a * x**3 + b* x**2 + c*x+ d


def cal_B(x):
    x = np.float64(x)
    a = np.float64(-0.044688)
    b = np.float64(0.200146)
    c = np.float64(0.620484)
    d = np.float64(0.565925)
    
    return a * x**3 + b* x**2 + c*x+ d



#-----------보정 적용 조건 함수

def apply_calibration(raw, cal_func, max_cal_distance):
    """
    raw    : raw distance from ESP32 anchor
    cal_func : calibration function (cal_A or cal_B)
    max_cal_distance : max distance that calibration is valid
                       A: 1.5m  /  B: 3.0m
    """
    # 보정 불가 구간 → raw 그대로 사용
    if raw > max_cal_distance:
        return raw

    # 보정 적용
    calibrated = cal_func(raw)

    # 보정값이 0.5m 미만이면 → clamp
    if calibrated < 0.5:
        return 0.5

    return calibrated




def calculate_vertical_distance(dA, dB, L):
    x = (dA**2 - dB**2 + L**2) / (2 * L)
    y2 = dA**2 - x**2
    if y2 < 0:
        y2 = 0.0
    return math.sqrt(y2)


def calculate_tag_position(d1, d2, anchor_dist):
    """
    두 앵커로부터의 거리(d1, d2)와 앵커 간의 거리(anchor_dist)를 이용해
    태그의 상대적인 (x, y) 좌표를 계산합니다.
    y가 우리가 원하는 '직선 거리'입니다.
    """
    try:
        # 1. x 좌표 계산
        x = (d1**2 - d2**2) / (2 * anchor_dist)
        
        # 2. y 좌표 계산
        # sqrt 안의 값이 음수가 되는 경우 (물리적으로 불가능한 삼각 부등식 위배)를 대비
        y_squared = d1**2 - (x + anchor_dist / 2)**2
        if y_squared < 0:
            return None, None # 계산 불가
            
        y = math.sqrt(y_squared)
        
        return x, y
    except ZeroDivisionError:
        return None, None # 앵커 거리가 0일 경우

#-------------방향 결정 함수
def decide_direction(d1, d2):

    if d2 > d1 + 0.15:
        return "RIGHT"
    elif d1 > d2 + 0.15:
        return "LEFT"
    else:
        return "STRAIGHT"

# -------------속도 결정 함수
def decide_speed(rf_dis):
    if rf_dis <= DISTANCE_THRESHOLDS[0]:
        return "LOW"
    elif rf_dis <= DISTANCE_THRESHOLDS[1]:
        return "MID"
    else:
        return "HIGH"

# =================================================
# === 모터 제어 함수
# =================================================

# Motor control
def move(left_speed, right_speed):
    
    # left_wheel
    if left_speed >= 0:
        GPIO.output(IN1, True)
        GPIO.output(IN2, False)
    else:
        GPIO.output(IN1, False)
        GPIO.output(IN2, True)

    
     # right_wheel
    if right_speed >= 0:
        GPIO.output(IN3, True)
        GPIO.output(IN4, False)
    else:
        GPIO.output(IN3, False)
        GPIO.output(IN4, True)


    pwm_left.ChangeDutyCycle(abs(left_speed))
    pwm_right.ChangeDutyCycle(abs(right_speed))

def stop():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)


# 모터 구동 함수 -----------------------

def drive(direction, speed_level):
    spd = SPEEDS[speed_level]
    if direction == "LEFT":
        
            move(0,40)
                       
    elif direction == "RIGHT":
        
            move(40,0)
                      
    else:
        move(spd, spd)
#------------------------------------------
#------------ 방향 결정 및 모터 제어 ---------
#------------------------------------------
def drive_proportional(speed_level, current_A, current_B, DEAD_BAND=0.15):
    """비례 제어를 사용하여 방향과 속도를 제어합니다."""
    
    # 1. 기본 속도 설정
    base_speed = SPEEDS[speed_level]
    
    # 2. 오차 계산 (A와 B의 거리 차이)
    error = current_A - current_B

    # 데드존 설정
    if abs(error) < DEAD_BAND:
        error = 0

    
    # 3. 조향량 계산 (Kp는 튜닝이 필요한 비례 상수)
    Kp = 70.0  # 이 값을 조절하여 회전 민감도를 튜닝 (50 ~ 150 사이에서 시작)
    turning_adjustment = Kp * error
    
    # 4. 각 바퀴의 최종 속도 계산
    left_speed = base_speed - turning_adjustment
    right_speed = base_speed + turning_adjustment
    
    # 속도가 100을 넘지 않도록 제한
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

     # 양수일 때
    if 0 < left_speed < MIN_SPEED:
        left_speed = MIN_SPEED
    if 0 < right_speed < MIN_SPEED:
        right_speed = MIN_SPEED

     # 음수일 때 (후진)
    if -MIN_SPEED < left_speed < 0:
        left_speed = -MIN_SPEED
    if -MIN_SPEED < right_speed < 0:
        right_speed = -MIN_SPEED

    move(left_speed,right_speed)

    return left_speed , right_speed


 
# Ultrasonic sensor   ----- 초음파 센서 설정
def get_ultrasonic_distance(TRIG,ECHO):
    
    # 1. 트리거 핀으로 10us 펄스를 전송하여 초음파 발생
    GPIO.output(TRIG, True)
    time.sleep(0.00001) # 10 마이크로초
    GPIO.output(TRIG, False)
    
    # 2. 에코 펄스가 돌아오는 것을 기다림
    # 펄스가 시작될 때까지의 시간을 기록
    timeout_start = time.time()
    
    while GPIO.input(ECHO) == 0:
        if time.time() - timeout_start > 0.05:
            return -1

    # 펄스가 끝날 때까지의 시간을 기록    
    pulse_start = time.time()

    # 4. 에코 신호가 끝(LOW)날 때까지 기다림
    while GPIO.input(ECHO) == 1:
        if time.time() - timeout_start > 0.05:
            return -1
    
    # 5. 신호가 끝난 정확한 시간 기록
    pulse_end = time.time()

    duration = pulse_end - pulse_start
    distance = duration*17150

    return round(distance, 2)

    # 거리가 0보다 작거나 비정상적으로 크면 실패로 처리
    if distance < 0 or distance > 400:
        return -1

    return round(distance, 2)    



# =================================================
# === 장애물 회피 함수
# =================================================

# avoid_obstacle  -------------- 장애물 회피 함수
def avoid_obstacle(last_dist, timeout, threshold):  

    """
    timeout: 몇 초간 기다릴지
    threshold: 임계 거리
    """

    stop()  # 장애물 탐지시 정지
    print("Obstacle detected -> 1.5초 대기 시작")
    data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_DETECTED")

    time.sleep(timeout)
    print("대기종료")

    final_check_left = deque(maxlen=3)
    final_check_mid = deque(maxlen=3)
    final_check_right = deque(maxlen=3)

    #마지막 순간에 3번 재서 평균 내기

    for _ in range (3) :
        final_check_left.append(get_ultrasonic_distance(TRIG1,ECHO1))
        final_check_mid.append(get_ultrasonic_distance(TRIG2,ECHO2))
        final_check_right.append(get_ultrasonic_distance(TRIG3,ECHO3))
        time.sleep(0.1)
    
    valid_left = [d for d in final_check_left if d > 0 ]
    smooth_left = sum(valid_left) / len(valid_left) if valid_left else -1

    valid_mid = [d for d in final_check_mid if d > 0 ]
    smooth_mid = sum(valid_mid) / len(valid_mid) if valid_mid else -1

    valid_right = [d for d in final_check_right if d > 0 ]
    smooth_right = sum(valid_right) / len(valid_right) if valid_right else -1
    
    print(f"final Left: {smooth_left:.2f}, Mid: {smooth_mid:.2f}, Right: {smooth_right:.2f}")

    # 마지막 상태 확인
    last_good = (
        smooth_left > threshold and
        smooth_mid > threshold and
        smooth_right > threshold
    )

    if last_good :
        print("장애물 없어짐 -> 추적모드 복귀")
        data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_RETURN")
        return  # 추적 모드로 복귀
        

    print("장애물 계속 존재, 회피 행동 실행")  #회전하면서 장애물 추적 함수와 연동
    data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_REVERSING")
    move(-40, -40)  # 뒤로 후진
    time.sleep(1.0)
    

    if smooth_left < smooth_right:
        print("turn right", flush = True)
        data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_ROTATING_RIGHT")
        clear = rotate_until_clear(last_dist,'right')
    else:
        print("turn left", flush = True)
        data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_ROTATING_LEFT")
        clear = rotate_until_clear(last_dist,'left')
    
    if not clear:
        stop()
        global waiting_from_obstacle
        waiting_from_obstacle = True #스레드에 알리기 위한 플래그
        return

    success = drive_straight_with_check(last_dist,timeout=1.0)

    if not success:
        return


def rotate_until_clear(last_dist, turn_direction='left', timeout=3.5, threshold=50):
    """
    turn_direction: 'left' 또는 'right'
    timeout: 최대 회전 시간 (초)
    threshold: 장애물이 없다고 판단할 거리 (cm)
    """
    print(f"회전방향: {turn_direction} 빈공간 탐색 시작")

    start_time = time.time()

    # 1. 회전 시작
    if turn_direction == 'left':
        move(-45, 45)   # 왼쪽 회전
    else:
        move(45, -45)   # 오른쪽 회전

    # 2. 회전하면서 주변 확인
    while time.time() - start_time < timeout:
        left  = get_ultrasonic_distance(TRIG1, ECHO1)
        mid   = get_ultrasonic_distance(TRIG2, ECHO2)
        right = get_ultrasonic_distance(TRIG3, ECHO3)

        print(f"Rotating… Left: {left:.2f}, Mid: {mid:.2f}, Right: {right:.2f}")

        is_left_clear  = (left  > threshold + 10) or (left  == -1)
        is_mid_clear   = (mid   > threshold + 10) or (mid   == -1)
        is_right_clear = (right > threshold + 10) or (right == -1)

        # 세 방향 모두 충분히 확보되면 회전 종료
        if is_left_clear and is_mid_clear and is_right_clear:
            print("빈공간 확인 → 회전 종료")
            stop()
            time.sleep(1)
            data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_FIND")
            return True

        time.sleep(0.1)   # 너무 자주 읽지 않도록 딜레이

    # 3. timeout까지 공간 확보 못 함
    stop()
    time.sleep(1)
    data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_FIND")
    print("Timeout reached → stop rotation (공간 못 찾음)")
    return False







def drive_straight_with_check(last_dist,timeout=2, threshold=50):
    """
    직진하면서 초음파로 장애물을 계속 확인.
    timeout 동안 clear면 True, 장애물 만나면 False 반환
    """
    print("마무리 직진 진행")
    data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_FINAL")

    start_time = time.time()
    move(40, 40)

    while time.time() - start_time < timeout: #1초동안 초음파 확인
        left  = get_ultrasonic_distance(TRIG1, ECHO1)
        mid   = get_ultrasonic_distance(TRIG2, ECHO2)
        right = get_ultrasonic_distance(TRIG3, ECHO3)

        #print(f"Straight: Left={left:.2f}, Mid={mid:.2f}, Right={right:.2f}")

        is_left_obs = 0<= left <threshold
        is_mid_obs = 0<= mid <threshold
        is_right_obs = 0<= right <threshold

        if is_left_obs or is_mid_obs or is_right_obs:
            print("직진중 장애물 탐지")
            stop()
            time.sleep(1)
            return False  # 장애물 발견

        time.sleep(0.1)

    stop()
    print("장애물 회피 완료")
    data_queue.put(f"DIST:{last_dist:.2f} MODE:AVOID_CLEAR")
    return True


def read_latest_line_from_serial(ser):
    """시리얼 버퍼의 모든 데이터를 읽어 가장 마지막 줄을 반환"""
    try:
        # 버퍼에 데이터가 있는지 확인
        if ser.in_waiting > 0:
            # 버퍼에 있는 모든 데이터를 한번에 읽음
            data_bytes = ser.read(ser.in_waiting)
            # UTF-8로 디코딩하고, 줄바꿈으로 나눈 뒤, 비어있지 않은 마지막 줄을 반환
            lines = data_bytes.decode('utf-8', errors='ignore').strip().split('\n')
            return lines[-1]
    except Exception as e:
        print(f"Serial read error: {e}")
    return None


# =================================================
# === 하드웨어 스레드 메인 루프
# =================================================

def hardware_thread():
    print("H/W 스레드 시작")
    
    # --- 상태 변수 및 설정 초기화 ---
    tracking, close, waiting = False, False, False
    PROXIMITY_STOP_DISTANCE = 0.6
    threshold = 50
    data_queue.put("DIST:--- MODE:POWERON")
    

    # --- 상태 저장용 변수 ---
    consecutive_read_fails = 0
    last_good_direction, last_good_speed = "STRAIGHT", "LOW"
    latest_valid_A, latest_valid_B = None, None
    dis_to_tag = 0.0

    last_valid_distance = 0
    left_speed = 0.0
    right_speed = 0.0


    #----------------------------
    obstacle_count_left = 0
    obstacle_count_mid = 0
    obstacle_count_right = 0
    OBSTACLE_TRIGGER_THRESHOLD = 5 # 3번 연속 감지 시 발동



    #-----------------거리값 필터링------------------------------
    history_A = deque(maxlen=5)
    history_B = deque(maxlen=5)

    # 마지막으로 '성공한 원본 값'을 저장할 변수
    last_accepted_raw_A = 0.0
    last_accepted_raw_B = 0.0

    smooth_A, smooth_B = 0.0, 0.0

    while not shutdown_event.is_set():
        # 1. [명령 처리]
        latest_cmd = None
        while not command_queue.empty():
            latest_cmd = command_queue.get()

        if latest_cmd:
            if latest_cmd == 'START':
                tracking = True
                consecutive_read_fails = 0
            elif latest_cmd == 'STOP':
                tracking = False
                consecutive_read_fails = 0
            elif latest_cmd == 'RESUME':
                waiting, tracking = False, False

        # 2. [UWB 센서 읽기]
        line_A = read_latest_line_from_serial(ser_A)
        line_B = read_latest_line_from_serial(ser_B)
        d_Araw = parse_range_line(line_A) if line_A else None
        d_Braw = parse_range_line(line_B) if line_B else None


   
       # A센서 필터링
        if d_Araw is not None:
            # history가 비어있다면(최초 유효값), 비교 없이 바로 기준값으로 삼고 큐에 추가
            
            if not history_A or abs(d_Araw - last_accepted_raw_A) < 0.3:
                history_A.append(d_Araw)
                last_accepted_raw_A = d_Araw # 유효한 원본 값을 기준으로 업데이트
            
            else : #튀는 값이 감지되면, 태그가 빠르게 움직인 것으로 간주하고 history를 비운다.
                print(f"Spike detected on A: old={last_accepted_raw_A:.2f}, new={d_Araw:.2f}. Resetting filter.")
                history_A.clear()
                history_A.append(d_Araw)
                last_accepted_raw_A = d_Araw    


        if history_A:
            smooth_A = sum(history_A) / len(history_A)
            A_cal = apply_calibration(smooth_A, cal_A,1.5)

        else:
            smooth_A = last_accepted_raw_A
            A_cal = apply_calibration(smooth_A, cal_A,1.5)
            



       # B센서 필터링
        if d_Braw is not None:
            # history가 비어있다면(최초 유효값), 비교 없이 바로 기준값으로 삼고 큐에 추가
            
            if not history_B or abs(d_Braw - last_accepted_raw_B) < 0.3:
                history_B.append(d_Braw)
                last_accepted_raw_B = d_Braw # 유효한 원본 값을 기준으로 업데이트
            
            else : #튀는 값이 감지되면, 태그가 빠르게 움직인 것으로 간주하고 history를 비운다.
                print(f"Spike detected on B: old={last_accepted_raw_B:.2f}, new={d_Braw:.2f}. Resetting filter.")
                history_B.clear()
                history_B.append(d_Braw)
                last_accepted_raw_B = d_Braw  

        if history_B:
            smooth_B = sum(history_B) / len(history_B)
            B_cal = apply_calibration(smooth_B, cal_B,3)
        else:
            smooth_B = last_accepted_raw_B
            B_cal = apply_calibration(smooth_B, cal_B,3)          

        

        # 3. [거리 계산]
        current_mode = ""   
        if history_A and history_B :
            consecutive_read_fails = 0
            dis_to_tag = calculate_vertical_distance(A_cal, B_cal, 0.42)
            

        else:
            consecutive_read_fails += 1
           

        # 4. [상태에 따른 행동 결정]
        if waiting:
            stop()
            current_mode = "WAITING_FOR_RESUME"
        elif tracking:

            #  초음파 센서 값을 먼저 읽고 사용하도록 순서 변경
            ultra_left = get_ultrasonic_distance(TRIG1, ECHO1)
            ultra_mid = get_ultrasonic_distance(TRIG2, ECHO2)
            ultra_right = get_ultrasonic_distance(TRIG3, ECHO3)

            # 왼쪽 센서 카운트
            if 0 <= ultra_left < threshold:
                obstacle_count_left += 1
            else:
                obstacle_count_left = 0 # 장애물 없으면 0으로 리셋
            
             # 중간 센서 카운트
            if 0 <= ultra_mid < threshold:
                obstacle_count_mid += 1
            else:
                obstacle_count_mid = 0

            # 오른쪽 센서 카운트
            if 0 <= ultra_right < threshold:
                obstacle_count_right += 1
            else:
                obstacle_count_right = 0

        
            print(f"왼쪽 : {ultra_left:.2f} cm , 가운데: {ultra_mid:.2f} cm , 오른쪽: {ultra_right:.2f} cm")
            #print(f"왼쪽 : {obstacle_count_left}  , 가운데: {obstacle_count_mid} , 오른쪽: {obstacle_count_right}")


            # 3번 이상 연속으로 감지된 센서가 하나라도 있으면 회피 동작 시작
            if (not close) and (obstacle_count_left >= OBSTACLE_TRIGGER_THRESHOLD or
                obstacle_count_mid >= OBSTACLE_TRIGGER_THRESHOLD or
                obstacle_count_right >= OBSTACLE_TRIGGER_THRESHOLD) :
                    
                # 카운트 초기화 후 회피 함수 호출
                obstacle_count_left, obstacle_count_mid, obstacle_count_right = 0, 0, 0
                    
                print("Obstacle detected!", flush=True)
                avoid_obstacle(dis_to_tag, 1.5, threshold)

                global waiting_from_obstacle
                if waiting_from_obstacle:
                    waiting = True
                    waiting_from_obstacle = False
                continue
            
            if dis_to_tag < PROXIMITY_STOP_DISTANCE:
                close = True
                stop()
                current_mode = "PROXIMITY_STOP"
            else:
                close = False
                
                speed_level = decide_speed(dis_to_tag)
                left_speed , right_speed = drive_proportional(speed_level, smooth_A, smooth_B)
                #print(f"A 거리 : {smooth_A:.2f}  B 거리 : {smooth_B:.2f}  방향 {direction_state} , Left: {left_speed:.1f}, Right: {right_speed:.1f}" )
            
                current_mode = "TRACKING"

        else: # tracking이 False일 때 (WAITING 모드)
            stop()
            current_mode = "WAITING"
        

         # 5. [최종 상태 결정 및 전송]
        if consecutive_read_fails >= 30:
            current_mode = "TRACKING_FAIL"
            stop()

        status_msg = f"DIST:{dis_to_tag:.2f} MODE:{current_mode}"
        data_queue.put(status_msg)
        time.sleep(0.03)

def handle_client(conn, addr) :
        print(f"[+] 연결됨: {addr}")
    

        try:
            while not shutdown_event.is_set():
                # 클라이언트 명령 수신 (비어있으면 무시)
                conn.settimeout(0.1)
                try:
                    data_bytes = conn.recv(1024) # decode() 전에 원본 데이터 받기
                    
                    # [수정] 클라이언트가 정상적으로 연결을 끊으면 recv는 빈 값을 반환
                    if not data_bytes:
                        print("[-] 클라이언트가 정상적으로 연결을 종료했습니다")
                        break # handle_client 루프를 빠져나감
                    
                    data = data_bytes.decode().strip()

                    if data:
                        print(f"[<-] {data}")
                        command_queue.put(data)

                except socket.timeout:
                    pass

                # 상태 전송
                latest_data = None
                while not data_queue.empty():
                    latest_data = data_queue.get()

                if latest_data:
                    msg_to_send = latest_data + "\n"
                    conn.sendall(msg_to_send.encode())

                time.sleep(DELAY)

        except (ConnectionResetError, BrokenPipeError) as e:
            
            print(f"[!] 클라이언트 연결이 비정상 종료: {e}")
        
        except Exception as e:
            # 그 외 예상치 못한 심각한 오류일 때만 전체 종료
            print(f"[!!!] 심각한 오류 발생: {e}")
            shutdown_event.set()        
    
        finally:
            conn.close()
            print(f"[-] 연결 종료: {addr}")
            command_queue.put('STOP') 

def network_thread():

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
       
        # 1초 타임아웃 설정. accept()가 무한정 기다리지 않게 함.
        s.settimeout(1.0)
       
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[=] TCP 서버 대기중 (포트 {PORT})")

        while not shutdown_event.is_set(): 
            
            try :
                # 1초 동안 연결을 기다림
                conn, addr = s.accept()
                # 연결이 성공하면 클라이언트 처리
                handle_client(conn, addr)

            except socket.timeout:
                # 1초 동안 연결이 없으면 timeout 예외 발생. 
                # 이것은 정상적인 상황이므로 그냥 지나가고 while 루프의 조건을 다시 확인.
                continue

            except Exception as e:
                print(f"[!] 서버 오류: {e}")
                shutdown_event.set() # 오류 시 종료 신호

        print("[NET] 네트워크 스레드 종료")
    
    

#---메인 코드 ---#
if __name__ == "__main__":

    try :
        
        hw_thread = threading.Thread(target=hardware_thread)
        net_thread = threading.Thread(target=network_thread)
        
        print("[MAIN] 모든 스레드를 시작합니다...")
        hw_thread.start()
        net_thread.start()

        # 메인 스레드는 종료 신호가 올 때까지 여기서 대기
        shutdown_event.wait()

    except KeyboardInterrupt:
        print("\n[MAIN] 사용자에 의해 종료 신호 감지...")
        shutdown_event.set()

    finally:
        # 모든 스레드에 확실하게 종료 신호를 보내고, 스레드가 끝날 때까지 기다림
        print("[MAIN] 모든 스레드를 정리합니다...")
        shutdown_event.set()

        # 2. 모든 스레드가 끝날 때까지 기다리기
        hw_thread.join()
        net_thread.join()    # 프로그램 종료 직전 모든 리소스 정리
        
        
        # 3. 모든 스레드가 종료된 후 리소스 정리
        stop()
        print("[MAIN] GPIO 및 시리얼 포트 정리...")
        ser_A.close()
        ser_B.close()
        GPIO.cleanup()
        print("[MAIN] 프로그램이 완전히 종료되었습니다.")
