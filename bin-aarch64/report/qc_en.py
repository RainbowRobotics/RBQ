#!/usr/bin/env python3
"""보고서 JSON 의 영어 — 로봇 앱 'PTZ 점검'이 영어로 볼 때(PTZ docs/qc/03 §6 '언어').

QC PC 의 PDF · 웹은 한국어 그대로다. 여기는 qc_export.py 가 한국어 글 옆에 붙일 영어만 만든다.
  title(no)           단계 이름
  step(no, L, d)      무엇을 · 어떻게 · 왜 · 기준 — 한국어 글(qc_steps · qc_soak)과 같은 숫자(L = 그 검사의 문턱)로 따로 쓴다
  line(no, ko)        한 줄 결과 — 숫자가 든 글은 틀(정규식)로 옮긴다
  col · cell · text   표 머리 · 칸 · 그 밖의 글 — 같은 글이면 사전, 숫자가 든 글은 틀
못 옮기면 None — 앱은 한국어를 그대로 보인다(빈칸을 보이지 않는다). 한국어 글을 고치면 여기 사전 · 틀도 같이 본다.
"""
from __future__ import annotations

import re

TITLE = {
    1: "Communication", 2: "Identity & versions", 3: "Firmware up to date", 4: "Driver status", 5: "Settings (EEPROM)",
    6: "Current zero", 7: "Homing", 8: "Holding still", 9: "Point moves", 10: "Soft limits", 11: "Preset & EEPROM",
    12: "Camera video", 13: "Speaker → microphone", 14: "Moving via the camera", 15: "Back to zero",
    16: "Gateway communication load", 17: "Diagnostic counters", 18: "ONVIF control & home preset", 19: "Soak (aging)",
    20: "Recheck after the soak",
}


def title(no: int, ko: str = "") -> str | None:
    return TITLE.get(no)


# ---------------------------------------------------------------- 같은 글이면 사전
WORDS: dict[str, str] = {}


def _add(pairs):
    for k, v in pairs:
        WORDS[k] = v


# 판정 · 흔한 칸
_add([
    ("PASS", "PASS"), ("WARN", "WARN"), ("FAIL", "FAIL"), ("SKIP", "SKIP"), ("중지", "Stopped"), ("무효", "Invalid"),
    ("기록만", "Record only"), ("기록 없음", "No record"), ("안 함", "Not run"), ("공회전 중", "Soaking"), ("기록 멈춤", "Record stalled"),
    ("있음", "Present"), ("없음", "None"), ("예", "Yes"), ("아니오", "No"), ("—", "—"), ("", ""),
    ("게이트웨이", "Gateway"), ("YAW", "YAW"), ("PITCH", "PITCH"), ("줌", "Zoom"), ("열화상", "Thermal"),
    ("게이트웨이 · YAW · PITCH", "Gateway · YAW · PITCH"), ("가까움", "Near"), ("한계 밖", "Over the limit"),
    ("나빠짐", "Worse"), ("나아짐", "Better"), ("크게 변함", "Changed a lot"), ("도착만 봄", "Arrival only"),
    ("정상", "Normal"), ("못 읽음", "Not read"), ("응답 없음", "No response"), ("구동이 켜져 있어 재지 않음", "Not measured — drive on"),
    ("High(정상)", "High (normal)"), ("Low(이상)", "Low (fault)"), ("켜짐", "On"), ("꺼짐", "Off"), ("읽은 적 없음", "Never read"),
    ("12가지 모두 그대로", "All 12 unchanged"), ("0초", "0 s"), ("안 찍음", "Not recorded"),
    ("걸었음 · 상한에 닿은 적 없음", "Applied · never reached the limit"),
    ("0초 — 공회전 동안 로봇은 서 있었다", "0 s — the robot stood still during the soak"),
    ("기준을 벗어난 때 없음", "Never outside the limits"),
    ("모름(시리얼 칸이 없는 옛 펌웨어)", "Unknown (old firmware without the serial field)"),
    ("모름(검사가 시리얼을 읽기 전에 끊김)", "Unknown (the check stopped before reading it)"), ("비어 있음", "Empty"),
    ("써 넣지 못함(모터 보드가 썼다는 답을 보내지 않음)", "not written (the motor board did not confirm the write)"),
    ("써 넣지 못함(모터 보드가 형식이 틀리다고 거절)", "not written (the motor board refused the format)"),
    ("써 넣었지만 다시 읽을 때 응답 없음", "written, but no answer when read back"),
    ("못 걸었음 — 모터 펌웨어에 임시 상한 기능이 없어 상한 없이 돌았다",
     "Not applied — the motor firmware has no temporary limit, so it ran without one"),
    ("모터가 거절해 상한 없이 돌았다", "The motor refused, so it ran without a limit"),
])
# 표 머리
_add([
    ("항목", "Item"), ("값", "Value"), ("축", "Axis"), ("보드", "Board"), ("판정", "Verdict"), ("검사", "Check"), ("대상", "Target"),
    ("한계", "Limit"), ("이동", "Move"), ("목표", "Target"), ("명령", "Command"), ("멈춘 위치", "Stopped at"), ("오차", "Error"),
    ("목표를 넘어간 정도", "Overshoot"), ("도착 시간", "Arrival time"), ("서야 할 자리", "Where it should stop"),
    ("이동 중 최대 전류 (Cur MAX 대비 %)", "Peak current while moving (% of Cur MAX)"),
    ("이동 중 최대 전류 (괄호는 YAW 의 Cur MAX 대비 %)", "Peak current while moving (YAW: % of Cur MAX in brackets)"),
    ("앱 버전", "App version"), ("부트로더", "Bootloader"), ("부트로더 CRC", "Bootloader CRC"), ("앱 CRC", "App CRC"),
    ("칩 UID (보증 기준)", "Chip UID (warranty key)"), ("제품 시리얼", "Product serial"),
    ("굽기 전 앱 CRC", "App CRC before"), ("기준 CRC", "Reference CRC"), ("기준 파일", "Reference file"), ("구웠나", "Flashed"),
    ("굽기 뒤 CRC", "CRC after"), ("굽는 데 걸린 시간", "Flash time"),
    ("드라이버 폴트", "Driver fault"), ("nFAULT 핀", "nFAULT pin"), ("STATUS 레지스터", "STATUS register"), ("모터 구동", "Motor drive"),
    ("영점 U", "Zero U"), ("영점 V", "Zero V"), ("영점 W", "Zero W"), ("남은 전류", "Residual current"),
    ("저장한 자리", "Saved position"), ("돌아온 자리 오차", "Return error"),
    ("처음", "First"), ("재검사", "Recheck"), ("차이", "Difference"), ("변함", "Change"), ("재검사 결과", "Recheck result"),
    ("처음 → 재검사", "First → recheck"), ("한계에 다가간 정도 (재검사)", "Closeness to the limit (recheck)"),
    ("가장 나쁜 값", "Worst value"), ("통신 지연 탓", "Due to comm delay"), ("진짜 이상", "Real fault"),
    ("돌린 시간 / 계획", "Ran / planned"), ("기준을 벗어난 시간", "Time outside limits"), ("시작 ~ 끝", "Start ~ end"),
    ("번호", "No."), ("결과", "Result"),
])


def col(ko: str) -> str | None:
    return WORDS.get(ko) or text(ko)


# ---------------------------------------------------------------- 숫자가 든 글은 틀로
_NUM = r"[−+-]?[0-9][0-9,.]*"
RULES: list[tuple[re.Pattern, object]] = []


def rule(pat: str):
    def deco(fn):
        RULES.append((re.compile(pat), fn))
        return fn
    return deco


def cell(ko: str) -> str | None:
    if ko is None:
        return None
    ko = str(ko)
    if ko in WORDS:
        return WORDS[ko]
    if re.fullmatch(r"[−+\-0-9.,:/ °%x×()A-Za-z_\[\]\-→]*", ko):
        return ko                                    # 숫자 · 단위 · 16진수 · 영문만 — 그대로
    return text(ko)


def text(ko: str) -> str | None:
    if not ko:
        return None
    ko = str(ko)
    if ko in WORDS:
        return WORDS[ko]
    for pat, fn in RULES:
        m = pat.fullmatch(ko)
        if m:
            out = fn(m) if callable(fn) else m.expand(fn)
            if out:
                return out
    return None


def line(no: int, ko: str) -> str | None:
    return text(ko)


def step(no: int, L: dict, d: dict) -> dict:
    """무엇을 · 어떻게 · 왜 · 기준(판정 글자 → 글). 없는 단계는 빈 dict."""
    fn = STEPS.get(no)
    try:
        return fn(L, d) if fn else {}
    except (KeyError, TypeError, ValueError):       # 문턱이 빠진 옛 기록 — 영어 없이(앱은 한국어를 보인다)
        return {}


def _g(v) -> str:
    return f"{float(v):g}"


def _s1(L, d):
    return {"what": "The PC talks to the gateway and both motor boards and checks that all three answer. A PTZ has a YAW axis "
                    "that turns the camera left and right and a PITCH axis that tilts it up and down, each with its own motor board. "
                    "The gateway relays messages between the PC and the motor boards.",
            "how": "It sends an 'are you alive' signal (CAN_CHECK) to all three at once, resends every 0.8 s and waits up to 2.5 s. "
                   "A motor board that stays silent is asked once more whether it is stuck in its bootloader, to tell a dead board "
                   "from one that has no firmware. The bootloader is the small start-up program used to write (flash) new firmware.",
            "why": "Nothing else can be checked without an answer.",
            "crit": {"PASS": "All three boards answer within 2.5 s.",
                     "FAIL": "Any board does not answer. All later steps are skipped."}}


def _s2(L, d):
    return {"what": "Reads the firmware (app) version of the three boards and checks that each bootloader is in place. Also reads the "
                    "unique ID (UID) of the gateway chip and the product serial.",
            "how": "Each board is asked for its app version and boot info (BOOT_INFO), which tells whether a bootloader is present and "
                   "the CRC of the bootloader and the app. A CRC is a short number computed from the file contents; it changes when "
                   "the contents change even a little, so it works like a fingerprint. The warranty is tied to the gateway chip UID. "
                   "In the factory check, an empty serial field is filled with the serial entered at the start and read back to confirm.",
            "why": "Without a bootloader the firmware cannot be updated over the cable; a SWD programmer has to be attached to the "
                   "board. YAW and PITCH run the same firmware, so their versions must match.",
            "crit": {"PASS": "All three boards answer and have a bootloader. If reference versions were set, the versions match them.",
                     "WARN": "YAW and PITCH versions differ, the gateway UID could not be read, or a board did not pass through its "
                             "bootloader at power-up. It is also WARN if the product serial stored on the YAW board differs from the "
                             "serial entered for the check, or the serial could not be written into an empty field.",
                     "FAIL": "No answer to the version or boot info, a bootloader is missing, or a version differs from the reference."}}


def _s3(L, d):
    if d.get("검사.레벨") == "1":
        return {"what": "Only compares the firmware of the three boards with the release reference by CRC. This is the basic check "
                        "(level 1): it does not flash firmware, does not measure the current zeros again, and leaves out steps 11 and "
                        "18, which change presets.",
                "how": "The CRC in the reference list is compared with the app CRC read in step 2. A difference is only reported.",
                "why": "A rebuilt file can differ even with the same version number; the CRC catches that.",
                "crit": {"PASS": "The app CRC of all three boards matches the reference.",
                         "WARN": "A board differs from the reference — its firmware is not the latest. Flashing is done in the "
                                 "full check (level 2 and up).",
                         "FAIL": "The reference list could not be read. All later steps are skipped.",
                         "SKIP": "Run without a reference list."}}
    return {"what": "Checks by CRC that the firmware of the three boards is identical to the release reference, and flashes the "
                    "boards that differ.",
            "how": "The CRC in the reference list (bin/manifest.json) is compared with the app CRC read in step 2. If they differ "
                   "and automatic flashing is on, YAW, PITCH and then the gateway are flashed over the cable (OTA). When a board "
                   "answers again its CRC is read back to confirm.",
            "why": "A rebuilt file can differ even with the same version number; the CRC catches that.",
            "crit": {"PASS": "The app CRC of all three boards matches the reference (from the start, or after flashing).",
                     "FAIL": "Flashing failed, took over 10 minutes for one board, the board did not answer within 60 s, or the "
                             "CRC still differs. A difference with automatic flashing off, or an unreadable reference list, is "
                             "also FAIL. All later steps are skipped.",
                     "SKIP": "Run without a reference list."}}


def _s4(L, d):
    return {"what": "Checks that the driver chips that power the motors are not reporting a problem.",
            "how": "Both motor boards are asked for their axis status: the driver fault flag, the nFAULT pin (High when normal, Low "
                   "on a fault) and the STATUS register where the chip records under-voltage, overheating, over-current or lock-up. "
                   "The firmware reads STATUS only after a fault, so 'never read' is normal for a board without faults. 'Motor drive' "
                   "tells whether the motor is powered; step 6 measures the residual current only on axes where it is off.",
            "why": "Running a motor with a faulty driver can make it move wrongly or damage parts.",
            "crit": {"PASS": "No fault on either axis, nFAULT is High and STATUS shows no problem.",
                     "FAIL": "Any of the three points to a problem, or no answer. Steps that move the axes are skipped."}}


def _s5(L, d):
    return {"what": "Reads all 21 settings stored in each motor board's EEPROM (memory that keeps its contents without power) and "
                    "checks that each is within the range QC allows.",
            "how": "All settings are requested from the YAW and PITCH boards at once; missing items are asked again one by one. "
                   "It also checks that the board number (BNO) matches the axis (YAW 1, PITCH 2). The current zeros (items 9–11) "
                   "are checked in step 6.",
            "why": "Running a motor with wrong settings can drive it too hard or move the wrong axis.",
            "crit": {"PASS": "All 21 items answer on both axes, BNO matches the axis and every value is in range.",
                     "FAIL": "An item does not answer, BNO is swapped, or a value is out of range. Steps that move the axes are skipped."}}


def _s6(L, d):
    lo, hi, res = _g(L["null_lo"]), _g(L["null_hi"]), _g(L["null_resid"])
    if "읽기만" in d.get("영점.방식", ""):
        return {"what": "Checks that the stored current-sensor zeros (the reading at 0 A) are within the normal range. The basic check "
                        "does not measure them again.",
                "why": "A shifted zero makes the motor control a current that is not really there, so it shakes or its force is uneven.",
                "how": "The zeros of the three phases (U, V, W — the three currents that feed the motor coils) read in step 5 are "
                       "checked against the range. A zero is a number from 0 to 4095 from the ADC (the circuit that turns a voltage "
                       "into a number). On axes whose motor drive was off in step 4, the current is read for 0.5 s to see that it is "
                       "close to 0 A (residual current).",
                "crit": {"PASS": f"The stored U, V and W zeros are all within {lo}–{hi} (around the middle of the ADC range), and the "
                                 f"residual current on axes with the drive off is {res} A or less.",
                         "WARN": f"The residual current is above {res} A, or the current request got no answer.",
                         "FAIL": "A stored zero is out of range or could not be read in step 5. Steps that move the axes are skipped."}}
    return {"what": "Measures and stores the reading of the motor current sensor at 0 A (the zero), and checks that it is in the "
                    "normal range.",
            "how": "The motor current is cut, and after 1 s the sensor outputs of the three phases (U, V, W) are averaged for 0.5 s "
                   "and stored in EEPROM as the zero. The output is a number from 0 to 4095 from the ADC. With no torque for about "
                   "1.5 s, PITCH may sag a little. On axes whose motor drive was off in step 4, the current is read for 0.5 s more "
                   "to see that it is close to 0 A (residual current).",
            "why": "A shifted zero makes the motor control a current that is not really there, so it shakes or its force is uneven.",
            "crit": {"PASS": f"The U, V and W zeros are all within {lo}–{hi} (around the middle of the ADC range), and the residual "
                             f"current on axes with the drive off is {res} A or less.",
                     "WARN": f"The residual current is above {res} A, or the current request got no answer.",
                     "FAIL": "A zero is out of range or no completion report came within 7 s. Steps that move the axes are skipped.",
                     "SKIP": "Current zero calibration was turned off; the zero already in EEPROM is used."}}


def _s7(L, d):
    return {"what": "Moves both axes to the hard stops at both ends to find the zero position (0°). This is called homing.",
            "how": "The axis is pushed to one end; there the motor encoder (a sensor that counts motor rotation in pulses) is set to "
                   "0, then the axis moves to the other end while counting. The firmware reports success only if this end-to-end "
                   "distance is larger than the minimum. End to end is taken as YAW 345° and PITCH 114°.",
            "why": "Angle commands mean nothing without a zero. A distance shorter than designed means the mechanism is caught "
                   "somewhere and every angle afterwards would be off.",
            "crit": {"PASS": "Homing succeeds on both axes.",
                     "FAIL": "Homing failed, was refused because of a driver fault, or did not finish within 80 s. Steps that move "
                             "the axes are skipped.",
                     "기록만": "The end-to-end distance and its ratio to the minimum (setting 5, Home Zero TH — by default 95% of the "
                              "designed distance)."}}


def _s8(L, d):
    p, w = _g(L["hold_pass"]), _g(L["hold_warn"])
    return {"what": "Checks that both axes stand still after homing (shake) and how much current they need to hold (holding current).",
            "how": "The current position is held for 1 s, then both axes' position and current are read for 3 s. The difference "
                   "between the highest and lowest position is the shake; the average current is the holding current.",
            "why": "An axis that shakes while it should be still shakes the camera image. PITCH needs current to carry the camera "
                   "weight; YAW is close to 0. A holding current higher than usual for the model means the balance is off or "
                   "something is pressing on the axis.",
            "crit": {"PASS": f"Shake on both axes is {p}° or less.", "WARN": f"Shake is above {p}° and up to {w}°.",
                     "FAIL": f"Shake is above {w}°.", "기록만": "Holding current."}}


def _cur_en(L) -> tuple[str, str]:
    r = f"{L['cur_ratio'] * 100:.0f}"
    if L.get("pitch_abs"):
        return (f"below {r}% of Cur MAX on YAW and below {_g(L['pitch_cur'])} A on PITCH",
                f"{r}% of Cur MAX or more on YAW, or {_g(L['pitch_cur'])} A or more on PITCH")
    return f"below {r}% of Cur MAX", f"{r}% of Cur MAX or more"


def _s9(L, d):
    ok, bad = _cur_en(L)
    return {"what": "Sends each axis to set angles and checks that it gets there accurately, without overshooting and without strain.",
            "how": "One axis at a time: YAW to −90°, +90°, 0°, and PITCH to +60°, −15°, 0°. During each move the largest overshoot "
                   "and the peak current are recorded, and the time until it first comes within 0.5° of the target (arrival time, "
                   "not judged). When the position stays within 0.03° for 0.3 s the axis is taken as stopped; that angle (from the "
                   "motor encoder) is the stop position, and its distance from the target is the error.",
            "why": "The camera looks where it should only if the axis stops exactly where commanded. A current spike while moving "
                   "means the mechanism is stiff or caught.",
            "crit": {"PASS": f"All six moves have an error of {_g(L['err_pass'])}° or less, an overshoot of {_g(L['os_pass'])}° or "
                             f"less, and a peak current {ok}.",
                     "WARN": f"Error above {_g(L['err_pass'])}° up to {_g(L['err_warn'])}°, overshoot above {_g(L['os_pass'])}° "
                             f"up to {_g(L['os_warn'])}°, or a peak current {bad} (possibly caught).",
                     "FAIL": f"Error above {_g(L['err_warn'])}° or overshoot above {_g(L['os_warn'])}°. Not stopping within the time "
                             "limit or losing the position signal for over 0.5 s is also FAIL.",
                     "": "The step verdict is the worst of the six moves."}}


def _s10(L, d):
    ok, bad = _cur_en(L)
    return {"what": "Checks that the firmware stops the axis before the mechanical end even when an angle past the end is "
                    "commanded. These firmware limits are called soft limits.",
            "how": f"The axes are commanded to the hard-stop positions. The firmware must stop them 4° inside: YAW ±{_g(L['yaw_lim'])}°, "
                   f"PITCH +{_g(L['pitch_max'])}° and {_g(L['pitch_min'])}° (shifted by the zero offset if there is one). The distance "
                   "between where the axis stops and that position is measured. After each end the axis returns to 0°.",
            "why": "A wrong command must not drive the axis into the hard stop and damage the mechanism or cables.",
            "crit": {"PASS": f"At both ends of both axes it stops within ±{_g(L['lim_pass'])}° of where it should, with a peak current {ok}.",
                     "WARN": f"It stops outside ±{_g(L['lim_pass'])}° but within ±{_g(L['lim_warn'])}°, or the peak current is {bad}.",
                     "FAIL": f"It stops outside ±{_g(L['lim_warn'])}°, does not stop within the time limit, or the position signal "
                             "is lost for over 0.5 s.",
                     "": "Moves back to 0° are checked for arrival only."}}


def _s11(L, d):
    slot, tol = int(L["preset_slot"]), _g(L["preset_tol"])
    return {"what": "Checks the preset function (returning to a saved position) and EEPROM writing and reading on the motors.",
            "how": f"Motor preset slot {slot} (for testing only) is checked to be empty. Both axes go to YAW 30°, PITCH 20°, the "
                   f"position is saved in slot {slot} (written to EEPROM), the axes move to 0°, and then 'go to {slot}' is sent. "
                   f"The difference between where the motors return and the saved position is measured, and slot {slot} is cleared.",
            "why": "Presets are only useful if the axes return exactly. A failed EEPROM write would be lost at power-off.",
            "crit": {"PASS": f"Both axes return within {tol}° and the save, go and clear commands all answer OK.",
                     "FAIL": f"Return error above {tol}°, no answer within 1.5 s or a non-OK answer, or a move did not finish in time.",
                     "SKIP": f"Slot {slot} already holds a position and is not overwritten. In the robot check it is cleared and "
                             "checked when preset changes were agreed; the step is skipped when they were not agreed or in the basic "
                             "check (level 1)."}}


def _s12(L, d):
    return {"what": "Checks that the zoom and thermal camera images are not just arriving but actually look right.",
            "how": f"Each camera's video (RTSP) must deliver {_g(L['cam_frames'])} frames within {_g(L['cam_first_s'])} s. For "
                   f"{_g(L['cam_watch_s'])} s from the first frame, the average brightness, how uneven it is (σ, standard deviation) "
                   "and the change from the previous frame are measured. For the thermal camera the σ of the temperature display "
                   "(OSD) area is measured too. Brightness runs from 0 (black) to 255 (white).",
            "why": "A black image (lens cap), a single-colour image (dead sensor) or a frozen image still delivers frames.",
            "crit": {"PASS": "Neither camera hits a FAIL or WARN below.",
                     "WARN": f"The σ of the thermal temperature area is below {_g(L['cam_osd_std'])} (the temperature text is not visible).",
                     "FAIL": f"Fewer than {_g(L['cam_frames'])} frames within {_g(L['cam_first_s'])} s, a black image (average below "
                             f"{_g(L['cam_dark_mean'])} and σ below {_g(L['cam_dark_std'])}), a single colour (σ below "
                             f"{_g(L['cam_flat_std'])}), or a frozen image (frame-to-frame change {_g(L['cam_freeze'])} or less).",
                     "SKIP": "Camera and sound checks were turned off, or there are no camera settings."}}


def _s13(L, d):
    p, f = _g(L["mic_pass"]), _g(L["mic_fail"])
    return {"what": "Checks that the camera speaker makes sound and the camera microphone hears it.",
            "how": "The camera speaker plays 1000 Hz and 2000 Hz beeps for 2 s each. In the same camera's microphone recording only "
                   "those frequencies are measured, against the level before the beep (background).",
            "why": "Only the sent frequencies are measured, so background noise is mostly filtered out. Two tones catch a speaker "
                   "or microphone that is dead in one band.",
            "crit": {"PASS": f"Both tones rise {p} dB or more above the background.",
                     "WARN": f"The weaker tone rises {f} dB or more but less than {p} dB.",
                     "FAIL": f"A tone rises less than {f} dB, or recording/playback cannot connect.",
                     "SKIP": "Camera and sound checks were turned off, or there is no zoom camera address/password."}}


def _s14(L, d):
    deg = _g(L["rs485_deg"])
    return {"what": "Checks that a pan command sent to the camera travels over the RS-485 cable to the gateway and the motor, so the "
                    "axis actually moves.",
            "how": f"A pan move is sent to the zoom camera with ONVIF (the standard network-camera protocol) every 0.5 s until YAW "
                   f"has moved {deg}°, then a stop is sent and the time is recorded.",
            "why": "It checks the whole path for commands that come through the camera, especially the RS-485 wiring.",
            "crit": {"PASS": f"YAW moves {deg}° or more within 15 s.",
                     "FAIL": f"YAW does not move {deg}° within 15 s (check the camera link, RS-485 wiring and gateway).",
                     "SKIP": "Camera and sound checks were turned off, or there is no zoom camera address/password."}}


def _s15(L, d):
    return {"what": "Returns both axes to 0° before the check ends.",
            "how": "YAW and then PITCH are sent to 0° and the step waits until they stop.",
            "why": "The next user or the soak starts from 0°. Accuracy was already checked in step 9, so only arrival is checked here.",
            "crit": {"PASS": "Both axes return to 0° and stop.",
                     "FAIL": "They do not stop in time, or the position signal is lost for over 0.5 s."}}


def _s16(L, d):
    p99, ll = _g(L["gw_p99"]), _g(L["gw_lossless"])
    return {"what": "Checks that the gateway does not drop messages when requests pile up, and how much headroom it has at normal load.",
            "how": "With both axes still, extra motor-current requests are added on top of normal traffic: 0 (L0), 100 (L1, "
                   "normal load), 300 (L2), 500 (L3) and 700 (L4) per second, 3 s each. For each load the answered requests, missing position "
                   "signals and the round-trip time of the check signal (CAN_CHECK) are measured as p50 (median) and p99. During "
                   "steps 1–15 a check signal is also sent about 4 times a second in the background to record drops and round-trip times.",
            "why": "The gateway forwards at most about 800 messages per second to the PC and drops the rest without counting, so "
                   "the PC counts them.",
            "crit": {"PASS": f"Nothing lost at L0 and L1, L1 round-trip p99 {p99} ms or less, at least {ll} extra requests per "
                             "second without loss, and no disconnection during the whole check.",
                     "WARN": f"L1 round-trip p99 above {p99} ms, fewer than {ll} per second without loss, or less than 90% of the "
                             "planned requests sent. Any disconnection, bootloader stall, lost position signal or unanswered check "
                             "signal during the whole check is also WARN.",
                     "FAIL": "Anything lost at L0 or L1 (up to 5 position signals per axis are tolerated), or a load stage was aborted.",
                     "기록만": "L4 (700 more per second) deliberately goes past the limit and is not judged.",
                     "SKIP": "Connected directly by CAN instead of LAN, or firmware was being flashed."}}


def _s17(L, d):
    return {"what": "Checks whether the problems the motor boards count by themselves (EEPROM write failures, dropped CAN messages, "
                    "driver faults…) went up during the check.",
            "how": "The motor boards count unusual events since power-up (diagnostic counters). They are read in step 2 and in this "
                   "step, and only the increase during the check is judged. The gateway has no counters, so crash reports and "
                   "bootloader stalls during the check are counted instead.",
            "why": "A problem that came and went during the check may not show anywhere else. A lost EEPROM write may mean the "
                   "zero or preset just saved is gone.",
            "crit": {"PASS": "None of the five judged counters increased, and there was no gateway crash report or bootloader stall.",
                     "WARN": "CAN TX drops, driver faults or bus-off increased, or the gateway crashed or stalled in the bootloader. "
                             "A counter lower than at the start (possible reboot during the check) is also WARN.",
                     "FAIL": "EEPROM writes were dropped or EEPROM timeouts increased."}}


def _s18(L, d):
    tp, tw = _g(L.get("homep_pass", 0.2)), _g(L.get("homep_warn", 0.5))
    return {"what": "Checks that the axes can be moved with the camera standard commands (ONVIF), and saves the 0° position from "
                    "step 15 as the camera preset 'home'. Calling home on site brings the PTZ back to this front position.",
            "how": "The camera preset list is read with the Hanwha command set (SUNAPI); home is saved in its existing slot or the "
                   "lowest free one. Saving makes the camera tell the gateway over RS-485, and both motors write the position to "
                   "EEPROM. Then YAW is moved over 10° with ONVIF, home is found in the ONVIF preset list and called (GotoPreset). "
                   f"The return error (the larger of the two axes) and the time to come within {tp}° are measured. Home is kept "
                   "as the shipping setting.",
            "why": "Home must always return to the same place, and the command path from the camera through RS-485 and the gateway "
                   "to the motors is checked as well.",
            "crit": {"PASS": f"After calling home both axes return within {tp}° of the saved position.",
                     "WARN": f"They return beyond {tp}° but within {tw}°.",
                     "FAIL": f"They stop beyond {tw}° or do not return, YAW does not move with ONVIF, the camera refuses or does "
                             "not answer within 5 s, or home is missing from the ONVIF list.",
                     "SKIP": "Camera checks were off, no zoom camera address/password, or a test video source. In the robot check: "
                             "preset changes not agreed, or the basic check (level 1)."}}


def _s19(L, d):
    return {"what": "Moves the product back and forth without a break for the soak time (12 hours by default) and checks that the "
                    "position signal, communication and camera video do not drop out or slow down.",
            "how": "Both axes follow a sine wave and four things are measured every second: position signals received per axis, "
                   "the tracking error, the communication round-trip time (CAN_CHECK) and the camera frames per second. Limits for "
                   "the tracking error and camera frames are set from the product's own values in the first part (baseline). In a "
                   "second where communication was delayed, the tracking limit is widened by the delay, to separate communication "
                   "trouble from product trouble.",
            "why": "It catches drop-outs or slowdowns that only show after a long run."}


def _s20(L, d):
    return {"what": "Runs QC steps 1–18 once more on a product that finished the soak, and compares with the first check.",
            "how": "After the soak, QC runs again while recording. Firmware is not flashed (step 3 only compares CRCs). Each step's "
                   "first and recheck verdicts, and each key value's first and recheck value, are set side by side.",
            "why": "During the soak only position signals, tracking error, round-trip time and camera frames are judged. Whether "
                   "the mechanism loosened or stiffened after 12 hours of motion, and whether the motors, communication, cameras and "
                   "sound still work as at first, can only be seen by running the same check again.",
            "crit": {"PASS": "The recheck is PASS and nothing changed a lot from the first check.",
                     "WARN": "The recheck is WARN or something changed a lot: a step verdict got worse, or a key value moved toward "
                             "its limit by 25% of the limit or more.",
                     "FAIL": "The recheck is FAIL or was stopped.",
                     "기록만": "Values without a limit, and changes away from the limit, are only recorded."}}


STEPS: dict = {1: _s1, 2: _s2, 3: _s3, 4: _s4, 5: _s5, 6: _s6, 7: _s7, 8: _s8, 9: _s9, 10: _s10, 11: _s11, 12: _s12,
               13: _s13, 14: _s14, 15: _s15, 16: _s16, 17: _s17, 18: _s18, 19: _s19, 20: _s20}


def soak_crit(j: dict | None) -> dict:
    """19 기준 — qc_soak.crit_v2 와 같은 숫자(soak.json limits)."""
    lim = (j or {}).get("limits") or {}
    base = (j or {}).get("baseline") or {}
    sin = (j or {}).get("sin") or {}

    def num(x, dflt):
        try:
            return float(x)
        except (TypeError, ValueError):
            return dflt
    fb, rtt = num(lim.get("fbMinPerS"), 90.0), num(lim.get("rttMaxMs"), 100.0)
    to = num(lim.get("pingTimeoutMs"), 1000.0) / 1000
    f, add = num(lim.get("trackFactor"), 1.5), num(lim.get("trackAddDeg"), 0.5)
    ratio = num(lim.get("fpsMinRatio"), 0.8) * 100
    bs = base.get("seconds") or lim.get("baselineS")
    bt = f"the first {dur_en(bs)}" if bs else "10% of the soak time, 30 s to 10 min"
    pk = num((sin.get("yaw") or {}).get("peakDps"), 0.0)
    ex = (f" For example, a round trip 0.1 s slower than usual widens the YAW limit by {pk * 0.1:.1f}° "
          f"(YAW moves at up to {pk:g} °/s)." if pk else "")
    return {"PASS": "It ran to the end and hit nothing below.",
            "FAIL": f"Any of these: the gateway connection dropped; fewer than {fb:g} position signals in a second; a camera gave 0 "
                    f"frames or fewer than {ratio:.0f}% of its baseline median; the tracking error went over the widened limit for "
                    "3 seconds or more in a row; the check signal went unanswered for 3 seconds or more in a row; a driver fault; "
                    "a motor restarted during the soak (a diagnostic counter went down); the PC went to sleep.",
            "중지": "No FAIL, but it stopped before the end. A communication delay still counts as stopped.",
            "WARN": f"It ran to the end with no FAIL but something showed: a communication delay (a second whose check-signal round "
                    f"trip was over {rtt:g} ms — the link between the PC and the gateway was slow, not a product fault), or seconds "
                    f"over the widened tracking limit or with no answer within {to:g} s that did not last 3 seconds in a row.",
            "": f"How the limits are set: tracking error and camera frames differ by product, so their limits come from the baseline "
                f"({bt}); during the baseline only those two are not judged. The tracking limit is the baseline p99 × {f:g} + {add:g}°. "
                "When communication is slow the PC compares an older position with the command, so the limit is widened by the "
                f"delay times the axis' top speed.{ex} The camera limit is {ratio:.0f}% of the baseline median."}


def dur_en(sec) -> str:
    try:
        s = int(round(float(sec)))
    except (TypeError, ValueError):
        return "—"
    h, m, x = s // 3600, s % 3600 // 60, s % 60
    out = []
    if h:
        out.append(f"{h} h")
    if m:
        out.append(f"{m} min")
    if x or not out:
        out.append(f"{x} s")
    return " ".join(out)


# ---------------------------------------------------------------- 표의 이름 칸 · 뜻 · 흔한 줄
_add([
    # 2
    ("응답함", "Answered"), ("확인 못 함 — 옛 펌웨어라 응답 없음", "Unknown — old firmware does not answer"),
    # 3
    ("안 구움 — 이미 같음", "Not flashed — already the same"), ("안 구움 — 기본 점검은 견주기만", "Not flashed — the basic check only compares"),
    ("예", "Yes"),
    # 4
    ("High (정상)", "High (normal)"), ("읽은 적 없음 (정상)", "Never read (normal)"),
    # 5
    ("보드 번호 — YAW 1 · PITCH 2", "Board number — YAW 1 · PITCH 2"), ("전류 제어 P 게인", "Current control P gain"),
    ("전류 제어 I 게인", "Current control I gain"), ("모터에 거는 최대 전압 [V]", "Max motor voltage [V]"),
    ("모터에 흘리는 최대 전류 [A]", "Max motor current [A]"), ("홈잉 최소 기준 — 끝~끝 거리 [펄스]", "Homing minimum — end-to-end distance [pulses]"),
    ("홈잉 최소 기준 — 끝에서 끝까지 거리 [펄스]", "Homing minimum — end-to-end distance [pulses]"),
    ("위치 제어 P 게인", "Position control P gain"), ("위치 제어 I 게인", "Position control I gain"), ("위치 제어 D 게인", "Position control D gain"),
    ("U 상 전류 영점 — 6단계가 새로 재기 전 값", "Phase U current zero — before step 6 measures it again"),
    ("V 상 전류 영점 — 6단계가 새로 재기 전 값", "Phase V current zero — before step 6 measures it again"),
    ("W 상 전류 영점 — 6단계가 새로 재기 전 값", "Phase W current zero — before step 6 measures it again"),
    ("원점 보정 각 [°]", "Zero offset angle [°]"), ("게이트웨이 통신 포트", "Gateway port"),
    ("전류 제어 안티와인드업 게인 [1/s] — 출력이 한계에 걸렸을 때 쌓인 적분을 덜어 내는 세기",
     "Current control anti-windup gain [1/s] — how fast built-up integral is removed when the output is limited"),
    ("움직일 때 최고 속도 [rad/s]", "Top speed [rad/s]"), ("가속 · 감속 [rad/s²]", "Acceleration / deceleration [rad/s²]"),
    ("따라가기 오차 한계 [rad]", "Following error limit [rad]"), ("홈잉 때 끝으로 다가가는 속도 [rad/s]", "Homing approach speed [rad/s]"),
    ("위치 제어 안티와인드업 게인 [1/s]", "Position control anti-windup gain [1/s]"), ("2단 감속비", "Second-stage gear ratio"),
    ("뜻", "Meaning"), ("QC 허용 범위", "Allowed range"),
    # 7
    ("끝에서 끝까지 거리", "End-to-end distance"), ("최소 기준", "Minimum"), ("최소 기준 대비", "Ratio to minimum"),
    ("출력축 1° 당 펄스", "Pulses per output degree"), ("홈 명령 다시 보낸 횟수", "Home command resent"),
    # 8
    ("흔들림 폭 (3초 동안)", "Shake (over 3 s)"), ("유지전류 (3초 평균)", "Holding current (3 s average)"),
    # 12
    ("해상도 · 초당 프레임", "Resolution · frames per second"), ("첫 프레임이 온 시간", "Time to first frame"),
    ("3초 동안 받은 프레임", "Frames in 3 s"), ("밝기 평균", "Average brightness"), ("밝기 σ (가장 큰 값)", "Brightness σ (largest)"),
    ("연속 프레임 밝기 차 (가장 큰 값)", "Frame-to-frame change (largest)"), ("온도 표시 칸 σ", "Temperature area σ"), ("기준", "Limit"),
    # 13
    ("소리", "Tone"), ("틀기 전 (그 주파수)", "Before (that frequency)"), ("틀었을 때", "While playing"), ("커진 만큼", "Rise"),
    # 14
    ("움직인 YAW 각도", "YAW moved"), ("5° 움직일 때까지 걸린 시간", "Time to move 5°"),
    # 16
    ("부하", "Load"), ("더 보낸 요청 (초당)", "Extra requests (per second)"), ("응답 손실", "Lost answers"), ("위치 신호 손실", "Lost position signals"),
    ("왕복 p50", "Round trip p50"), ("왕복 p99", "Round trip p99"), ("왕복 최대", "Round trip max"),
    ("40 ms 이상 걸린 왕복", "Round trips of 40 ms or more"), ("부하 시험", "Load test"), ("검사 내내 감시 (1~15단계)", "Watch during the whole check (steps 1–15)"),
    ("L1 (평소)", "L1 (normal)"), ("L4 (기록만)", "L4 (record only)"), ("지켜본 시간", "Time watched"), ("연결 끊김 / 다시 붙음", "Disconnects / reconnects"),
    ("부트로더에 멈춘 횟수", "Bootloader stalls"), ("게이트웨이 왕복 p50 / p99 / 최대", "Gateway round trip p50 / p99 / max"),
    ("YAW 보드 왕복 p50 / p99 / 최대", "YAW board round trip p50 / p99 / max"), ("PITCH 보드 왕복 p50 / p99 / 최대", "PITCH board round trip p50 / p99 / max"),
    ("YAW 위치 신호 받은 수 / 기대한 수", "YAW position signals received / expected"), ("PITCH 위치 신호 받은 수 / 기대한 수", "PITCH position signals received / expected"),
    ("YAW 위치 신호 간격 p50 / p99 / 최대", "YAW position signal gap p50 / p99 / max"), ("PITCH 위치 신호 간격 p50 / p99 / 최대", "PITCH position signal gap p50 / p99 / max"),
    ("YAW 위치 신호 간격이 25 ms 를 넘은 횟수", "YAW position signal gaps over 25 ms"), ("PITCH 위치 신호 간격이 25 ms 를 넘은 횟수", "PITCH position signal gaps over 25 ms"),
    # 17
    ("늘면 판정", "If it rises"), ("검사 동안", "During the check"), ("크래시 기록", "Crash reports"),
    ("EEPROM 드롭", "EEPROM drops"), ("EEPROM 타임아웃", "EEPROM timeouts"), ("EEPROM busy", "EEPROM busy"), ("CAN TX 드롭", "CAN TX drops"),
    ("폴트 복구", "Fault recoveries"), ("STATUS", "STATUS"), ("nFAULT", "nFAULT"), ("복구 I2C", "Recovery I2C"), ("bus-off", "bus-off"),
    ("CAN 재시작", "CAN restarts"), ("재시작 실패", "Restart failures"),
    ("EEPROM 에 쓸 것을 못 쓰고 버린 수", "EEPROM writes dropped"), ("EEPROM 이 제때 응답하지 않은 수", "Times EEPROM did not answer in time"),
    ("EEPROM 이 바빠 미룬 수", "Writes delayed because EEPROM was busy"), ("보낼 CAN 메시지를 못 보내고 버린 수", "CAN messages dropped before sending"),
    ("드라이버 칩이 폴트를 낸 수", "Driver chip faults"), ("폴트에서 스스로 되살아난 수", "Recoveries from a fault"),
    ("드라이버 STATUS 마지막 값(255 = 읽은 적 없음)", "Last driver STATUS (255 = never read)"), ("nFAULT 핀 지금 상태(1 = 정상)", "nFAULT pin now (1 = normal)"),
    ("마지막 폴트 복구 때 드라이버 칩과 주고받은(I2C) 결과(255 = 복구한 적 없음)", "Result of the last fault recovery over I2C (255 = never recovered)"),
    ("CAN 통신에서 떨어져 나간 수", "Times dropped off the CAN bus"), ("CAN 을 다시 켠 수", "CAN restarts"), ("CAN 을 다시 켜다 실패한 수", "Failed CAN restarts"),
    # 18
    ("저장 전 카메라 프리셋 목록 (SUNAPI)", "Camera preset list before saving (SUNAPI)"), ("home 을 저장한 번호", "Slot home was saved in"),
    ("SUNAPI 로 저장한 결과", "SUNAPI save result"), ("저장 뒤 카메라 프리셋 목록 (SUNAPI)", "Camera preset list after saving (SUNAPI)"),
    ("ONVIF 로 옮긴 YAW 각도", "YAW moved with ONVIF"), ("ONVIF 프리셋 목록", "ONVIF preset list"),
    ("home 을 부른 뒤 멈춘 위치 YAW / PITCH", "Stopped at after calling home, YAW / PITCH"), ("돌아오는 데 걸린 시간", "Time to return"),
    # 19
    ("한눈에", "At a glance"), ("까닭", "Reason"), ("돌린 시간", "Run time"), ("시작과 끝", "Start and end"),
    ("게이트웨이 연결이 끊긴 시간", "Gateway disconnected for"), ("확인 신호(CAN_CHECK)", "Check signal (CAN_CHECK)"),
    ("PC 가 잠든 시간", "PC asleep for"), ("진단 카운터", "Diagnostic counters"), ("움직인 모양 (사인파)", "Motion (sine wave)"),
    ("끝 영상", "End-of-soak video"), ("로봇이 움직인 시간", "Time the robot moved"), ("기록 당시 판정 (옛 기준)", "Verdict at the time (old rule)"),
    ("진짜 이상 (통신 지연 몫을 빼고도 남은 것)", "Real fault (left after removing communication delay)"),
    ("기준을 벗어난 시간 (옛 기준으로 센 것)", "Time outside limits (old rule)"),
    ("위치 신호 수", "Position signals"), ("따라가는 오차", "Tracking error"), ("통신 왕복 시간", "Communication round trip"),
    ("카메라 초당 프레임", "Camera frames per second"), ("모터 전류", "Motor current"), ("게이트웨이 · YAW · PITCH 번갈아", "Gateway · YAW · PITCH in turn"),
    ("시작부터 지난 시간", "Time since start"), ("시각", "Time"), ("새 기준으로 가르면", "By the new rule"), ("있었던 일", "What happened"),
    ("무엇이", "What"), ("공회전 동안 값이 어떻게 변했나", "How the values changed during the soak"),
    # 20
    ("단계 판정 — 처음과 재검사", "Step verdicts — first and recheck"), ("주요 수치 — 처음과 재검사", "Key values — first and recheck"),
    # 주요 수치
    ("PITCH 이동 중 최대 전류", "PITCH peak current while moving"), ("YAW 이동 중 최대 전류 (Cur MAX 대비)", "YAW peak current while moving (vs Cur MAX)"),
    ("PITCH 유지전류 (멈춰 있을 때)", "PITCH holding current (standing still)"),
    ("PITCH 홈잉 끝에서 끝까지 거리 (최소 기준 대비)", "PITCH homing end-to-end (vs minimum)"),
    ("YAW 홈잉 끝에서 끝까지 거리 (최소 기준 대비)", "YAW homing end-to-end (vs minimum)"),
    ("PITCH 이동 중 최대 전류 (Cur MAX 대비)", "PITCH peak current while moving (vs Cur MAX)"),
    ("PITCH 멈춘 위치 오차 (가장 큰 것)", "PITCH stop error (largest)"), ("YAW 멈춘 위치 오차 (가장 큰 것)", "YAW stop error (largest)"),
    ("게이트웨이 왕복 시간 p99 (평소 부하)", "Gateway round trip p99 (normal load)"),
    ("게이트웨이가 손실 없이 버틴 추가 요청", "Extra requests the gateway handled without loss"),
    ("검사 내내 게이트웨이 왕복 시간 p99", "Gateway round trip p99 during the whole check"),
    ("스피커 → 마이크 배경보다 커진 소리 (두 소리 중 약한 쪽)", "Speaker → microphone rise above background (weaker tone)"),
    ("1배 넘어야 홈잉 성공", "Must be over ×1 for homing to succeed"), ("≥ 500 번/초 (못 미치면 WARN)", "≥ 500 /s (WARN if below)"),
    ("항목별 가장 나쁜 값과 한계", "Worst value and limit per item"),
    ("YAW 홈잉 끝에서 끝까지 거리", "YAW homing end-to-end"), ("PITCH 홈잉 끝에서 끝까지 거리", "PITCH homing end-to-end"),
    ("YAW 이동 중 최대 전류", "YAW peak current while moving"),
])


@rule(r"(\d[\d,]*)번")
def _times(m):
    return f"{m.group(1)} times"


@rule(r"(\d[\d,]*)장")
def _frames(m):
    return f"{m.group(1)} frames"


@rule(r"(\d[\d,]*) 펄스")
def _pulses(m):
    return f"{m.group(1)} pulses"


@rule(r"([0-9.]+)배")
def _ratio(m):
    return f"×{m.group(1)}"


@rule(r"(\d+)초")
def _secs(m):
    return f"{m.group(1)} s"


@rule(r"((?:\d+시간 ?)?(?:\d+분 ?)?(?:\d+초)?)")
def _hms(m):
    t = m.group(1).strip()
    if not t:
        return None
    t = t.replace("시간", " h").replace("분", " min").replace("초", " s")
    return re.sub(r"\s+", " ", t).strip()


@rule(r"(.+?) ~ (.+)")
def _range(m):
    a, b = m.group(1), m.group(2)
    if re.fullmatch(r"[−\-0-9.,]+", a) and re.fullmatch(r"[−\-0-9.,]+", b):
        return f"{a} – {b}"
    return None


@rule(r"(YAW|PITCH) ([+−]) 쪽 끝")
def _end(m):
    return f"{m.group(1)} {m.group(2)} end"


@rule(r"(YAW|PITCH) 0° 로 돌아옴")
def _back0(m):
    return f"{m.group(1)} back to 0°"


@rule(r"(\d+) (.+)")
def _numbered_title(m):
    n = int(m.group(1))
    t = m.group(2).strip()
    return f"{n} {TITLE[n]}" if n in TITLE and (t in KO_TITLES or (n == 19 and re.fullmatch(r"(?:" + SOAK_LEN + r") 공회전", t))) else None


SOAK_LEN = r"\d+시간(?: \d+분)?|\d+분"     # 공회전 길이(qc_steps.soak_length) — 로봇 에이징은 30분 · 2시간 · 12시간


@rule(r"(" + SOAK_LEN + r") 공회전")
def _soak_title(m):
    return TITLE[19]


@rule(r"(" + SOAK_LEN + r") 공회전을 돌리지 않음")
def _soak_not_run(m):
    return f"The {_hms_len(m.group(1))} soak was not run"


@rule(r"QC 가 끝나고 (" + SOAK_LEN + r") 공회전을 돌리는 중이다\. 끝나면 이 페이지가 최종 보고서로 바뀌고 여기에 판정이 붙는다\.")
def _soak_running(m):
    return (f"The QC is done and the {_hms_len(m.group(1))} soak is running. When it ends, this page becomes the final report "
            "and the verdict appears here.")


def _hms_len(ko: str) -> str:
    """'12시간' → '12-hour', '30분' → '30-minute', '2시간 30분' → '2 h 30 min'"""
    if re.fullmatch(r"\d+시간", ko):
        return f"{ko[:-2]}-hour"
    if re.fullmatch(r"\d+분", ko):
        return f"{ko[:-1]}-minute"
    return ko.replace("시간", " h").replace("분", " min")


KO_TITLES = {"통신 응답", "식별 · 버전", "펌웨어 최신화", "드라이버 상태", "설정값 (EEPROM)", "전류 영점", "홈잉", "정지 유지",
             "지점 이동", "소프트 리밋", "프리셋 · EEPROM", "카메라 영상", "스피커 → 마이크", "카메라 경유 구동", "원위치",
             "게이트웨이 통신 부하", "진단 카운터", "ONVIF 제어 · home 프리셋", "12시간 공회전", "공회전 뒤 재검사"}


@rule(r"(PITCH|YAW) (.+) \[(.+)\]")
def _metric_unit(m):
    base = WORDS.get(f"{m.group(1)} {m.group(2)}")
    unit = {"배": "×", "펄스": "pulses", "번/초": "/s"}.get(m.group(3), m.group(3))
    return f"{base} [{unit}]" if base else None


# ---------------------------------------------------------------- 한 줄 결과(요약표) — 흔한 모양
LINE_RULES = [
    (r"게이트웨이 · YAW · PITCH 모두 응답함", "Gateway, YAW and PITCH all answered"),
    (r"펌웨어 버전 게이트웨이 (\S+), YAW·PITCH (\S+) · 부트로더 셋 다 있음 · 게이트웨이 UID 읽음(.*)",
     lambda m: _tail(f"Firmware: gateway {m.group(1)}, YAW/PITCH {m.group(2)} · all three bootloaders present · gateway UID read",
                     m.group(3))),
    (r"세 보드 모두 기준 펌웨어와 같아 굽지 않음", "All three boards match the reference firmware — not flashed"),
    (r"(.+?)(?:의)? 펌웨어가 기준과 달라 최신이 아님 · 기본 점검\(레벨 1\)이라 굽지 않음",
     lambda m: f"{'Some boards' if m.group(1) == '일부 보드' else m.group(1)}: firmware differs from the reference — not the latest "
               "· basic check (level 1), not flashed"),
    (r"두 축 드라이버 모두 폴트\(이상 표시\) 없음 · nFAULT 핀 High\(정상\)", "No driver fault on either axis · nFAULT pin High (normal)"),
    (r"두 축 21항목 모두 범위 안 · 최대 전류\(Cur MAX\) YAW ([0-9.]+) A, PITCH ([0-9.]+) A",
     r"All 21 items in range on both axes · max current (Cur MAX) YAW \1 A, PITCH \2 A"),
    (r"전류 영점 (\d+)~(\d+) \(허용 (\d+)~(\d+)\) · 모터 구동이 켜져 있어 남은 전류는 재지 않음",
     r"Current zeros \1–\2 (allowed \3–\4) · motor drive on, residual current not measured"),
    (r"전류 영점 (\d+)~(\d+) \(허용 (\d+)~(\d+)\) · 남은 전류 (.+) \(기준 ([0-9.]+) A 이하\)",
     r"Current zeros \1–\2 (allowed \3–\4) · residual current \5 (limit \6 A)"),
    (r"전류 영점 (\d+)~(\d+) \(허용 (\d+)~(\d+)\)", r"Current zeros \1–\2 (allowed \3–\4)"),
    (r"끝에서 끝까지 YAW ([\d,]+) 펄스\(최소 기준의 ([0-9.]+)배\), PITCH ([\d,]+) 펄스\(([0-9.]+)배\)",
     r"End to end: YAW \1 pulses (×\2 the minimum), PITCH \3 pulses (×\4)"),
    (r"흔들림 폭 YAW ([0-9.]+)°, PITCH ([0-9.]+)° · 유지전류 YAW ([0-9.]+) A, PITCH ([0-9.]+) A",
     r"Shake YAW \1°, PITCH \2° · holding current YAW \3 A, PITCH \4 A"),
    (r"오차 최대 ([0-9.]+)° · 목표를 넘어간 정도 최대 ([0-9.]+)°(.*)", lambda m: f"Max error {m.group(1)}° · max overshoot {m.group(2)}°" + _cur_tail(m.group(3))),
    (r"두 축의 양쪽 끝 모두 서야 할 자리에서 ([0-9.]+)° 안에 멈춤 — 멈춘 곳 (.+)",
     r"Both ends of both axes stopped within \1° of the limit — stopped at \2"),
    (r"돌아온 자리 오차 YAW ([0-9.]+)°, PITCH ([0-9.]+)° \(기준 ([0-9.]+)° 이하\)", r"Return error YAW \1°, PITCH \2° (limit \3°)"),
    (r"줌 (\d+×\d+) ([0-9.]+) fps, 열화상 (\d+×\d+) ([0-9.]+) fps — 검은 화면도, 한 가지 색 화면도, 멈춘 화면도 아님",
     r"Zoom \1 \2 fps, thermal \3 \4 fps — not black, not a single colour, not frozen"),
    (r"배경보다 1000 Hz ([+−-]?[0-9.]+) dB, 2000 Hz ([+−-]?[0-9.]+) dB 커짐 — 둘 다 기준 ([0-9.]+) dB 이상",
     r"Above background: 1000 Hz \1 dB, 2000 Hz \2 dB — both at least \3 dB"),
    (r"카메라 명령으로 YAW 가 ([0-9.]+)° 움직임 · 5° 움직이는 데 ([0-9.]+)초", r"YAW moved \1° on a camera command · \2 s to move 5°"),
    (r"0° 로 돌아와 YAW ([−\-0-9.]+)°, PITCH ([−\-0-9.]+)° 에 멈춤", r"Back to 0°, stopped at YAW \1°, PITCH \2°"),
    (r"YAW · PITCH 모두 늘어난 이상 없음 · 게이트웨이 크래시도 부트로더 멈춤도 없음",
     "No counter rose on YAW or PITCH · no gateway crash or bootloader stall"),
    (r"home 을 (\d+)번에 저장(\(있던 home 을 덮어씀\))? · ONVIF 로 불러 ([0-9.]+)초 만에 돌아옴 · 돌아온 자리 오차 ([0-9.]+)° \(기준 ([0-9.]+)° 이하\)",
     lambda m: f"Home saved in slot {m.group(1)}{' (replaced the old home)' if m.group(2) else ''} · called with ONVIF, back in "
               f"{m.group(3)} s · return error {m.group(4)}° (limit {m.group(5)}°)"),
    (r"계획한 (.+) 다 돌림 · (.+)", lambda m: f"Ran the full {_hms_en(m.group(1))} · {text(m.group(2)) or m.group(2)}"),
    (r"계획한 (.+) 다 돌림", lambda m: f"Ran the full {_hms_en(m.group(1))}"),
    (r"통신 지연 (\d+)초\(왕복 최대 (\d+) ms\) — 제품 고장 아님, 게이트웨이 통신이 밀림",
     r"Communication delay \1 s (round trip up to \2 ms) — not a product fault, the gateway link was slow"),
    (r"재검사 (PASS|WARN|FAIL|중지) · 처음 검사와 견줘 크게 변한 곳 없음", r"Recheck \1 · nothing changed a lot from the first check"),
    (r"재검사 (PASS|WARN|FAIL|중지) · 크게 변한 곳 (\d+)곳 — (.+)", r"Recheck \1 · \2 thing(s) changed a lot — \3"),
    (r"이번에 고르지 않은 단계", "Not picked this time"),
    (r"이번에는 사람이 고른 단계만 다시 돌렸고, 이 단계는 고르지 않았다\.", "Only the steps a person picked were run again, and this one was not picked."),
    (r"프리셋을 바꾸는 단계라 기본 점검에서 뺌", "Left out of the basic check — it changes presets"),
    (r"프리셋 자리를 바꾸는 단계라 기본 점검\(레벨 1\)에서는 돌리지 않았다\.", "This step changes preset positions, so the basic check (level 1) did not run it."),
    (r"프리셋 변경에 동의하지 않음", "Preset changes not agreed"),
    (r"검사를 시작할 때 프리셋 변경에 동의하지 않아, 프리셋을 바꾸는 이 단계는 돌리지 않았다\.",
     "Preset changes were not agreed at the start, so this preset step was not run."),
    (r"원점이 잡혀 있어 다시 잡지 않음", "Already homed — not homed again"),
    (r"두 축의 원점이 이미 잡혀 있어서, 고른 단계만 다시 돌릴 때는 홈잉을 다시 하지 않았다\.",
     "Both axes were already homed, so homing was not repeated when re-running picked steps."),
    (r"사람이 검사를 중지함", "Stopped by a person"), (r"이 단계에 오기 전에 사람이 검사를 중지했다\.", "A person stopped the check before this step."),
    (r"(\d+)단계\((.+)\) 실패로 검사가 멈춤", lambda m: f"Stopped after step {m.group(1)} ({TITLE.get(int(m.group(1)), m.group(2))}) failed"),
    (r"(\d+)단계\((.+)\) 실패 — 축을 움직이는 검사라 안 함", lambda m: f"Not run — step {m.group(1)} ({TITLE.get(int(m.group(1)), m.group(2))}) failed and this step moves the axes"),
    (r"QC 판정", "QC verdict"), (r"QC 판정 — 공회전 전 중간 보고", "QC verdict — interim report before the soak"),
]
for _pat, _out in LINE_RULES:
    RULES.append((re.compile(_pat), _out if callable(_out) else (lambda o: (lambda m: m.expand(o)))(_out)))


def _hms_en(ko: str) -> str:
    return text(ko) or ko


def _cur_tail(t: str) -> str:
    """' · 이동 중 최대 전류 YAW 1.67 A (Cur MAX 의 42%), PITCH 3.60 A (기준 5 A 미만)' → 영어."""
    if not t.strip():
        return ""
    t = t.replace(" · 이동 중 최대 전류 ", " · peak current while moving ")
    t = re.sub(r"\(Cur MAX 의 (\d+)%\)", r"(\1% of Cur MAX)", t)
    t = re.sub(r"\(기준 ([0-9.]+) A 미만\)", r"(limit below \1 A)", t)
    return t


# 판정 도장 까닭 — 'QC PASS · 공회전 WARN(통신 지연 92초) · 재검사 WARN · 전·후 크게 변함 1곳'
@rule(r"(QC .+)")
def _final_why(m):
    parts = m.group(1).split(" · ")
    out = []
    for p in parts:
        q = (p.replace("공회전 중", "soaking").replace("공회전", "soak").replace("재검사", "recheck")
             .replace("통신 지연", "communication delay").replace("전·후 크게 변함", "big changes before/after")
             .replace("단계를 다시 돌림(처음 결과는 남김)", "steps re-run (first results kept)")
             .replace("공회전 기록을 못 읽음", "soak record unreadable").replace("공회전을 끝까지 못 돌았음", "soak did not finish")
             .replace("결과 없음", "no result"))
        q = re.sub(r"(\d+)초", r"\1 s", q)
        q = re.sub(r"(\d+)곳", r"\1", q)
        if re.search(r"[가-힣]", q):
            return None
        out.append(q)
    return " · ".join(out)


# ---------------------------------------------------------------- 남은 고정 글
_add([
    ("응답", "Answer"), ("설정 21가지 값 모두 보기", "Show all 21 settings"),
    ("UID 는 게이트웨이(LAN2CAN) 칩에서만 읽는다.", "The UID is read from the gateway (LAN2CAN) chip only."),
    ("UID 는 게이트웨이(LAN2CAN) 칩에서만 읽는다. 제품 시리얼은 YAW 보드 EEPROM 에 저장되고, 처음에는 출하 검사가 써 "
     "넣는다. 시리얼 칸이 없는 옛 펌웨어는 '모름'으로 적는다.",
     "The UID is read from the gateway (LAN2CAN) chip only. The product serial is stored in the YAW board EEPROM and is first "
     "written by the factory check. Old firmware without the serial field is shown as 'Unknown'."),
    ("재검사 결과 칸은 재검사가 PASS 가 아니거나 판정이 바뀐 단계만 적는다. 단계마다 자세한 값은 아래 '재검사 단계별 자세히'에 있다.",
     "The recheck result is shown only for steps that are not PASS or whose verdict changed. Details per step are below."),
    ("1초에 받은 위치 신호(모터가 지금 각도를 알려 주는 신호) 수. 모터는 1초에 100번 보낸다.",
     "Position signals (the motor reporting its current angle) received in a second. The motor sends 100 a second."),
    ("사인파로 움직이라고 명령한 각도와 실제 각도의 차이. 1초마다 그사이 가장 큰 값을 적는다. 기구마다 크기가 달라서 한계는 "
     "제품마다 기준 구간에서 정한다.",
     "The difference between the commanded sine-wave angle and the actual angle; the largest in each second is recorded. It "
     "differs by mechanism, so each product's limit is set from its baseline."),
    ("확인 신호(CAN_CHECK)를 PC 에서 보드로 보내 응답이 돌아오기까지 걸린 시간. 1초마다 그사이 가장 긴 값을 적는다. "
     "1초에 4번쯤 게이트웨이 · YAW · PITCH 에 돌아가며 보낸다.",
     "The time for a check signal (CAN_CHECK) to go from the PC to a board and back; the longest in each second is recorded. "
     "About 4 are sent a second, to the gateway, YAW and PITCH in turn."),
    ("1초에 받은 영상 장수. 한계는 제품마다 기준 구간에서 정하고, 0장이면 언제나 기준을 벗어난다.",
     "Video frames received in a second. The limit is set from each product's baseline; 0 frames is always out of limits."),
    ("모터에 물어 받은 전류 크기다(10-08 부터는 0.1초마다, 그 전에는 1초마다 물었다). 판정에는 쓰지 않고, 공회전하는 동안 "
     "전류가 점점 오르는지 보려고 남긴다. 같은 움직임에 전류가 더 들면 기구가 뻑뻑해졌거나 모터가 달아오른 것일 수 있다. "
     "그래프 위 한 줄의 '앞쪽'은 사인파로 들어가는 처음 30초를 빼고 잡는다. PITCH 그래프의 점선은 판정 한계가 아니라 공회전 "
     "동안만 걸어 둔 임시 전류 상한이다(걸었을 때만 그린다). 모터를 보호하려고 공회전 동안만 낮추고, 검사가 끝나면 풀며 모터 "
     "전원을 다시 넣어도 풀린다.",
     "Motor current read from the motor (every 0.1 s since 10-08, every second before). Not judged — kept to see whether the "
     "current creeps up during the soak; more current for the same motion can mean a stiffer mechanism or a hot motor. The "
     "'early' window in the line above each chart leaves out the first 30 s (entering the sine wave). The dashed line on the "
     "PITCH chart is not a judging limit but a temporary current limit set only for the soak (drawn only when it was set). It "
     "protects the motor during the soak; it is released when the check ends, and a motor power cycle also clears it."),
    ("이동 중 최대 전류는 9 · 10 · 15단계 이동 전부에서, 멈춘 위치 오차는 9 · 15단계에서 가장 큰 값이다. "
     "PITCH 전류는 Cur MAX 와 상관없이 5 A 를 한계로 본다. 10-08 에 바뀐 기준이라, 그 전에 돈 검사는 그때처럼 Cur MAX "
     "비율로 적는다. 왕복 시간은 PC 가 확인 신호를 보내고 응답을 받기까지 걸린 시간이고, p99 는 100번 가운데 99번이 이보다 "
     "빠른 값이다. 한계에 다가간 정도가 85% 이상이면 '가까움', 한계에 닿거나 넘으면 '한계 밖'이 붙는다.",
     "Peak current is the largest over all moves in steps 9, 10 and 15; stop error is the largest in steps 9 and 15. PITCH "
     "current is judged against 5 A regardless of Cur MAX. This rule started on 10-08, so older checks are shown as a ratio of "
     "Cur MAX, as they were judged then. The round trip is the time from the PC sending a check signal to getting the answer; "
     "p99 is the value 99 out of 100 round trips are faster than. 'Near' is added at 85% of the limit or more, 'Over the limit' "
     "at or past the limit."),
    ("처음 결과는 남김", "first result kept"),
    ("앱에서 멈춤", "stopped from the app"), ("중지 — 앱에서 멈춤", "Stopped — stopped from the app"),
    ("이 검사가 없던 QC 프로그램으로 돌림", "Run with a QC program that did not have this check"),
    ("이 검사가 아직 들어가지 않은 QC 프로그램으로 돌렸다.", "The check was run with a QC program that did not have this step yet."),
    ("12시간 공회전을 돌리지 않음", "The 12-hour soak was not run"),
    ("공회전 기록(soak.json) 없이 최종 보고서를 냈다. 공회전을 돌리지 않았거나 시작하지 못했다.",
     "The final report was made without a soak record (soak.json): the soak was not run or could not start."),
    ("13번은 펌웨어에서 쓰지 않는 번호라 없다. P · I · D 게인은 제어가 오차에 얼마나 세게 반응할지 정하는 수다. "
     "단위를 적지 않은 값은 펌웨어가 쓰는 값 그대로다.",
     "No. 13 is not used by the firmware. The P, I and D gains set how strongly the control reacts to an error. Values without "
     "a unit are the firmware's raw values."),
    ("도착 시간은 명령부터 서야 할 자리 0.5° 안에 처음 들어올 때까지다. 나머지 칸은 9단계 표와 같은 뜻이다.",
     "Arrival time is from the command until the axis first comes within 0.5° of where it should stop. The other columns mean "
     "the same as in the step 9 table."),
    ("모터 보드 왕복은 게이트웨이에서 CAN(게이트웨이와 모터 보드를 잇는 통신선)을 한 번 더 거쳐서 게이트웨이 왕복보다 길다.",
     "A motor board round trip also crosses the CAN bus (the line between the gateway and the motor boards), so it is longer "
     "than the gateway round trip."),
    ("값은 '시작 → 끝'이고 괄호는 검사 동안 늘어난 수다. STATUS · nFAULT · 복구 I2C 는 횟수가 아니라 상태 값이다. "
     "'응답 없음'은 보드가 그 카운터 값을 보내지 않았다는 뜻이다(판정에 쓰는 줄이면 WARN).",
     "Values are 'start → end' and the brackets show the increase during the check. STATUS, nFAULT and recovery I2C are states, "
     "not counts. 'No response' means the board did not send that counter (WARN on a judged row)."),
    ("출력축 1° 당 펄스는 끝에서 끝까지 잰 펄스를 YAW 345°, PITCH 114° 로 나눈 값이다. 엔코더가 감속기(톱니바퀴로 "
     "속도를 줄이고 힘을 키우는 장치) 앞 모터축에 있어서, 출력축(카메라를 실제로 돌리는 축)이 1° 돌 때 수백~천 "
     "펄스가 나온다. 축이 홈잉을 시작하지 않으면 홈 명령을 다시 보낸다.",
     "Pulses per output degree is the end-to-end pulse count divided by YAW 345° or PITCH 114°. The encoder sits on the motor "
     "shaft before the gearbox (gears that lower the speed and raise the force), so one degree of the output shaft (the one that "
     "turns the camera) gives hundreds to a thousand pulses. If an axis does not start homing, the home command is sent again."),
    ("dBFS 는 녹음이 담을 수 있는 가장 큰 소리를 0 으로 둔 dB 다(음수가 클수록 작은 소리).",
     "dBFS is dB relative to the loudest sound the recording can hold (0); the more negative, the quieter."),

    ("공회전 탭이 남긴 일 — 게이트웨이 연결 끊김과 다시 붙음, 카메라 다시 연결, PC 절전 막기 풀림, 기록 실패.",
     "What the soak tab logged — gateway disconnects and reconnects, camera reconnects, the PC sleep block being lifted, write failures."),
    ("그래프에 마우스를 올리거나 눌러 방향키로 옮기면 그 칸의 값이 나온다.",
     "Hover over a chart, or click it and use the arrow keys, to see the values at that point."),
])


@rule(r"버전 (\S+) · (\S+) · 소스 커밋 ([0-9a-f]+)(?: \(커밋 안 된 변경 (\d+)건\))?")
def _image(m):
    tail = f" ({m.group(4)} uncommitted changes)" if m.group(4) else ""
    return f"Version {m.group(1)} · {m.group(2)} · source commit {m.group(3)}{tail}"


@rule(r"(PASS|WARN|FAIL|중지)\(그때 까닭: (.+)\) — 이 보고서는 soak\.csv 를 새 기준\((.+)\)으로 다시 셌다")
def _old_soak(m):
    return f"{text(m.group(1)) or m.group(1)} (reason then: {text(m.group(2)) or m.group(2)}) — this report recounted soak.csv with the new rule ({m.group(3)})"


@rule(r"기준을 벗어난 시간 (\d+)초")
def _out_secs(m):
    return f"{m.group(1)} s outside limits"


@rule(r"기준을 벗어난 때\(옛 기준으로 센 것\) — 모두 (\d+)초(?:, 앞 (\d+)개)?")
def _out_cap(m):
    return f"Times outside limits (old rule) — {m.group(1)} s in all" + (f", first {m.group(2)}" if m.group(2) else "")


@rule(r"기준을 벗어난 때 목록 보기 \((\d+)개\)")
def _out_fold(m):
    return f"Show the times outside limits ({m.group(1)})"


@rule(r"진짜 이상이 있던 때 — 통신 지연 몫을 빼고도 남은 것, 모두 (\d+)초(?:, 앞 (\d+)개)?")
def _real_cap(m):
    return f"Real faults — what remains after removing communication delay, {m.group(1)} s in all" + (
        f", first {m.group(2)}" if m.group(2) else "")


@rule(r"진짜 이상 목록 보기 \((\d+)개\)")
def _real_fold(m):
    return f"Show the real faults ({m.group(1)})"


@rule(r"사건 — (\d+)건(?:, 앞 (\d+)건)?")
def _events_cap(m):
    return f"Events — {m.group(1)}" + (f", first {m.group(2)}" if m.group(2) else "")


@rule(r"사건 목록 보기 \((\d+)건\)")
def _events_fold(m):
    return f"Show the events ({m.group(1)})"


@rule(r"PITCH 임시 전류 상한 ([0-9.]+) A \(공회전 동안만\)")
def _cap_head(m):
    return f"PITCH temporary current limit {m.group(1)} A (soak only)"


@rule(r"걸었음 · 전류가 상한의 (\d+)% 이상이던 시간 모두 ([0-9.]+)초\. 이때 따라가는 오차가 한계를 넘었다면 상한 탓일 수 있다")
def _cap_hit(m):
    return (f"Applied · current was at {m.group(1)}% of the limit or more for {m.group(2)} s in all. If the tracking error went "
            "over its limit then, the current limit may be the cause")


@rule(r"(\d+)초 — 그동안 잰 값에는 로봇이 움직여 생긴 흔들림이 섞였을 수 있다")
def _robot_moved(m):
    return f"{m.group(1)} s — values measured then may include shaking from the robot moving"


@rule(r"(\S+) \(입력한 시리얼은 (\S+)\)")
def _serial_differs(m):
    return f"{m.group(1)} (entered serial {m.group(2)})"


@rule(r"(\S+) \(이번 검사가 써 넣음\)")
def _serial_written(m):
    return f"{m.group(1)} (written by this check)"


@rule(r"써 넣지 못함\(EEPROM 쓰기 실패, 결과 (\d+)\)")
def _serial_eeprom(m):
    return f"not written (EEPROM write failed, result {m.group(1)})"


@rule(r"써 넣었지만 다시 읽은 값이 다름\((.*)\)")
def _serial_readback(m):
    return f"written, but read back differently ({m.group(1)})"


@rule(r"(YAW|PITCH)(?: · (YAW|PITCH))? 따라가는 오차가 늘린 한계도 넘음 (\d+)초 연속")
def _track_run(m):
    axes = m.group(1) + (f" and {m.group(2)}" if m.group(2) else "")
    return f"{axes} tracking error over the widened limit for {m.group(3)} s in a row"


@rule(r"확인 신호 응답 없음 (\d+)(초|번)(.*)")
def _lost(m):
    unit = "s" if m.group(2) == "초" else "times"
    rest = m.group(3)
    tail = {"": "", "(따로따로)": " (apart)"}.get(rest)
    if tail is None:
        mm = re.fullmatch(r"\(가장 길게 (\d+)초 연속\)", rest)
        tail = f" (longest {mm.group(1)} s in a row)" if mm else None
    if tail is None:
        mm = re.fullmatch(r" 연속", rest)
        tail = " in a row" if mm else None
    return None if tail is None else f"Check signal unanswered {m.group(1)} {unit}{tail}"


def _tail(head: str, ko_tail: str) -> str | None:
    """' · 조각 · 조각' 꼬리를 조각마다 옮겨 붙인다 — 못 옮기는 조각이 있으면 None(한국어 그대로)."""
    parts = [p for p in ko_tail.split(" · ") if p.strip()]
    out = [head]
    for p in parts:
        e = text(p)
        if e is None:
            return None
        out.append(e)
    return " · ".join(out)


@rule(r"이번에 켤 때 부트로더를 안 거침: (.+)")
def _noboot(m):
    return f"did not pass through the bootloader at power-up: {m.group(1)}"


@rule(r"제품 시리얼\((\S+)\)이 입력한 시리얼\((\S+)\)과 다름")
def _serial_line(m):
    return f"product serial ({m.group(1)}) differs from the entered serial ({m.group(2)})"


@rule(r"시리얼을 (.+)")
def _serial_line2(m):
    e = text(m.group(1))
    return f"serial {e}" if e else None


@rule(r"재검사 기록 — (\S+) · 시작 (.+)")
def _rc_note(m):
    return f"Recheck record — {m.group(1)} · started {m.group(2)}"


@rule(r"(YAW|PITCH) (?:→ (−?[0-9.]+)°|([+−]) 쪽 끝): (.+)")
def _bad_move(m):
    what = f"{m.group(1)} → {m.group(2)}°" if m.group(2) else f"{m.group(1)} {m.group(3)} end"
    parts = []
    for p in m.group(4).split(", "):
        e = _move_reason(p)
        if e is None:
            return None
        parts.append(e)
    return f"{what}: " + ", ".join(parts)


def _move_reason(p: str) -> str | None:
    rules = [
        (r"이동 중 최대 전류 ([0-9.]+) A — ([0-9.]+) A 부터 WARN\(걸림 의심\)", r"peak current \1 A — WARN from \2 A (possibly caught)"),
        (r"이동 중 최대 전류 ([0-9.]+) A", r"peak current \1 A"),
        (r"Cur MAX 의 (\d+)% — (\d+)% 부터 WARN\(걸림 의심\)", r"\1% of Cur MAX — WARN from \2% (possibly caught)"),
        (r"오차 ([0-9.]+)°\(([0-9.]+)° 넘으면 (WARN|FAIL)\)", r"error \1° (\3 above \2°)"),
        (r"목표를 넘어간 정도 ([0-9.]+)°\(([0-9.]+)° 넘으면 (WARN|FAIL)\)", r"overshoot \1° (\3 above \2°)"),
        (r"([−\-0-9.]+)° 에 멈춤 — 서야 할 ([−\-0-9.]+)° 에서 ([0-9.]+)° 벗어남\(([0-9.]+)° 넘으면 (WARN|FAIL)\)",
         r"stopped at \1°, \3° from \2° (\5 above \4°)"),
    ]
    for pat, out in rules:
        mm = re.fullmatch(pat, p.strip())
        if mm:
            return mm.expand(out)
    return None


@rule(r"(.+) · (.+)")
def _joined(m):
    """' · ' 로 이은 글 — 조각마다 옮길 수 있을 때만."""
    parts = m.group(0).split(" · ")
    if len(parts) < 2:
        return None
    out = []
    for p in parts:
        e = WORDS.get(p) or next((fn(mm) if callable(fn) else mm.expand(fn) for pat, fn in RULES
                                  if pat is not _JOINED and (mm := pat.fullmatch(p))), None)
        if not e:
            return None
        out.append(e)
    return " · ".join(out)


_JOINED = RULES[-1][0]

_add([
    ("평소 부하(L1)에서 잃은 메시지 없음", "No messages lost at normal load (L1)"),
    ("평소 부하(L1)에서 잃은 메시지 있음", "Messages lost at normal load (L1)"),
    ("검사 내내 연결 끊김 0회", "No disconnects during the whole check"),
])


@rule(r"왕복 시간 p99 ([0-9.]+) ms")
def _p99(m):
    return f"round trip p99 {m.group(1)} ms"


@rule(r"초당 (\d+)번 더 걸어도 손실 없음")
def _lossless(m):
    return f"no loss even with {m.group(1)} extra per second"


@rule(r"검사 내내 연결 끊김 (\d+)회")
def _drops(m):
    return f"{m.group(1)} disconnects during the whole check"


@rule(r"([<≤≥>]) ?([0-9.]+) ?(A|%|°|ms|dB)")
def _limit(m):
    return f"{m.group(1)} {m.group(2)} {m.group(3)}".replace(" °", "°")


@rule(r"평소 부하\(L1\)에서 (잃은 메시지 0|잃은 메시지 있음|빠진 메시지는 위치 신호 (\d+)개뿐\(축마다 (\d+)개까지 봐줌\)) · (.+)")
def _l1_line(m):
    if m.group(2):
        head = (f"At normal load (L1) only {m.group(2)} position signals were missing (up to {m.group(3)} per axis "
                "is tolerated)")
    else:
        head = "Nothing lost at normal load (L1)" if m.group(1).endswith("0") else "Messages lost at normal load (L1)"
    return _tail(head, m.group(4))


@rule(r"([0-9.]+)% \((\d+)개 빠짐\)")
def _fb_cell(m):
    return f"{m.group(1)}% ({m.group(2)} missing)"


@rule(r"검사가 멈춤 — (.+)")
def _stopped_why(m):
    return f"The check stopped — {text(m.group(1)) or m.group(1)}"


@rule(r"이 단계에 오기 전에 검사가 멈췄다\. 까닭: (.+)\.")
def _stopped_why2(m):
    return f"The check stopped before this step. Reason: {text(m.group(1)) or m.group(1)}."


@rule(r"배터리 (\d+) % — (\d+) % 이하라 멈춤")
def _battery(m):
    return f"Battery {m.group(1)} % — stopped at {m.group(2)} % or less"


@rule(r"다른 PTZ 가 물려 있다\(UID (\S+) — 보고서는 (\S+)\) — 다시 돌리기를 멈춤")
def _other_ptz(m):
    return f"A different PTZ is attached (UID {m.group(1)} — the report is {m.group(2)}) — re-run stopped"


@rule(r"돌아온 자리 오차는 home 을 부른 뒤 두 축이 멈춘 위치와 저장한 자리의 차이 가운데 큰 쪽이다\. "
      r"걸린 시간은 부른 때부터 ([0-9.]+)° 안에 처음 들어올 때까지다\.")
def _home_note(m):
    return (f"The return error is the larger of the two axes' differences between where they stopped after home was called and "
            f"the saved position. The time is from the call until both first come within {m.group(1)}°.")


@rule(r"카메라 주소 — (.+)")
def _cam_addr(m):
    parts = []
    for p in m.group(1).split(" · "):
        mm = re.fullmatch(r"(줌|열화상) (\S+)", p)
        if not mm:
            return None
        parts.append(f"{WORDS[mm.group(1)]} {mm.group(2)}")
    return "Camera addresses — " + " · ".join(parts)


@rule(r"(손실 없이 버틴 부하는 없다\(L0 에서도 손실\)|손실 없이 버틴 최대 부하는 초당 (\d+)번( 넘게)?이다)\. "
      r"응답 손실 괄호 안은 못 받은 응답 수 / 보낸 요청 수다\.")
def _gw_note(m):
    head = ("No load was handled without loss (lost even at L0)" if not m.group(2) else
            f"The highest load handled without loss was {'over ' if m.group(3) else ''}{m.group(2)} extra per second")
    return head + ". In the lost answers column, the brackets show answers not received / requests sent."


@rule(r"기준 목록 — (.+)\. 소스 커밋은 펌웨어 소스를 저장소에 기록해 둔 번호이고, 커밋 안 된 변경은 그 뒤에 고치고 아직 "
      r"기록하지 않은 것이다\.")
def _manifest_note(m):
    man = (m.group(1).replace("소스 커밋", "source commit").replace("만든 때", "made").replace("커밋 안 된 변경", "uncommitted changes")
           .replace("건", ""))
    if re.search(r"[가-힣]", man):
        return None
    return (f"Reference list — {man}. The source commit is the number under which the firmware source was recorded in the "
            "repository; uncommitted changes are edits made after it that are not recorded yet.")


@rule(r"(dBFS 는 녹음이 담을 수 있는 가장 큰 소리를 0 으로 둔 dB 다\(음수가 클수록 작은 소리\)\.)(?: 튼 소리 — (.+?)\.)?(?: 참고 — (.+)\.)?")
def _mic_note(m):
    out = [WORDS[m.group(1)]]
    if m.group(2):
        tone = (m.group(2).replace("각 ", "each ").replace("크기 설정 ", "volume setting "))
        if re.search(r"[가-힣]", tone):
            return None
        out.append(f"Tones played — {tone}.")
    if m.group(3):
        refs = []
        for p in m.group(3).split(", "):
            mm = re.fullmatch(r"배경 소음 전체 크기 (.+)", p)
            mt = re.fullmatch(r"스피커에서 마이크까지 걸린 시간\(추정\) (.+) ms", p)
            if mm:
                refs.append(f"overall background noise {mm.group(1)}")
            elif mt:
                refs.append(f"speaker-to-microphone delay (estimated) {mt.group(1)} ms")
            else:
                return None
        out.append("For reference — " + ", ".join(refs) + ".")
    return " ".join(out)


@rule(r"기준 구간은 처음 (.+?)이다\. 따라가는 오차와 카메라 초당 프레임은 제품마다 크기가 달라서, 이 구간에서 잰 값으로 한계를 "
      r"정하고 그 뒤부터 견준다\. p99 는 100번 가운데 99번이 이보다 작다는 값이다\.(?: 기준 구간 안의 가장 큰 따라가는 오차는 "
      r"YAW (\S+) · PITCH (\S+) 다\. 사인파에 들어가는 첫 몇 초가 들어 있어 한계와 견주지 않는다\.)? (마지막 두 칸은 .+)")
def _soak_notes(m):
    rest = text(m.group(4))
    if rest is None:
        return None
    out = (f"The baseline is the first {_hms_en(m.group(1))}. Tracking error and camera frames differ by product, so their limits "
           "are set from values measured in this window and compared from then on. p99 is the value 99 out of 100 are below.")
    if m.group(2):
        out += (f" The largest tracking error inside the baseline was YAW {m.group(2)} · PITCH {m.group(3)}; it includes the first "
                "seconds of entering the sine wave, so it is not compared with the limit.")
    return out + " " + rest


@rule(r"마지막 두 칸은 그런 일이 있던 시간\(초\)이다\. .+ 평소 왕복(\(([0-9.]+ ms)\))?을 뺀 값이다\. 평소 왕복은 기준 구간 왕복의 "
      r"가운데값이고, 왕복을 못 잰 초에는 직전 (\d+)초 안의 마지막 왕복을 쓴다\.")
def _soak_cols(m):
    base = f" ({m.group(2)})" if m.group(2) else ""
    return ("The last two columns are how long it happened (seconds). 'Due to comm delay' counts time outside the limits because "
            "communication was slow: tracking error over the limit but within the widened limit, and round trips over the delay "
            "threshold. 'Real fault' counts what remains after removing communication delay: tracking error over even the widened "
            "limit, unanswered check signals, and position signals or camera frames outside their limits. Large tracking error and "
            "low frame rates are counted from the end of the baseline; the rest from the start. The delay used to widen the limit "
            f"is the larger of the longest round trips in that second and the one before, minus the usual round trip{base}. The "
            "usual round trip is the median in the baseline; a second without a round trip uses the last one within the previous "
            f"{m.group(3)} s.")


@rule(r"까닭의 숫자는 그 초에 잰 값이다\. .+")
def _outlier_note(m):
    return ("The numbers in the reason are values measured in that second: position signals received, the largest tracking "
            "difference and the longest round trip; 'check signal unanswered' is how many got no answer within 1 s. Every time "
            "outside the limits is in the 'outlier' column of soak.csv. The 'By the new rule' column tells whether it was due to "
            "slow communication (comm delay) or remains after removing that (real fault).")


@rule(r"차이는 재검사 값에서 처음 값을 뺀 것이다\. .+ 한계 쪽으로 한계의 (\d+)% 이상 움직인 것이다\. .+")
def _recheck_note(m):
    return ("The difference is the recheck value minus the first value. Closeness to the limit is how near the recheck value is "
            "to its limit; 100% means it reached the limit (for values where bigger is better, like sound, the limit is divided "
            f"by the value). 'Changed a lot' means it moved toward the limit by {m.group(1)}% of the limit or more. The brackets on "
            "the homing end-to-end distance convert that difference to an output-shaft angle (end to end taken as YAW 345°, "
            "PITCH 114°).")
