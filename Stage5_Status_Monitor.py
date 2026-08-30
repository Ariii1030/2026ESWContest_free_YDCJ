"""Stage5 ESP32 상태 확인창 (그래프 없음).

필요 패키지: pyserial
실행: python Stage5_Status_Monitor.py
Arduino IDE의 Serial Monitor는 닫고 사용한다.
"""

import queue
import re
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("pyserial이 필요합니다: py -m pip install pyserial") from exc


BAUD = 115200
SYSTEM_STATES = {
    0: "사용 안 함",
    1: "데이터 대기",
    2: "근피로 측정 중",
    3: "진동 중",
    4: "발열 중",
    5: "재캘리브레이션 중",
    6: "접촉 상태 확인 필요",
    7: "기준값 저장 대기",
    8: "기준값 반영 대기",
    9: "오류",
}


class StatusMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("근피로 밴드 상태 확인")
        self.root.geometry("720x600")
        self.root.minsize(650, 540)
        self.ser = None
        self.reader = None
        self.stop_event = threading.Event()
        self.lines = queue.Queue()

        self.values = {
            "connection": tk.StringVar(value="연결 안 됨"),
            "baseline": tk.StringVar(value="기준값 측정 전"),
            "mdf_change": tk.StringVar(value="-- %"),
            "rms_change": tk.StringVar(value="-- %"),
            "fatigue_progress": tk.StringVar(value="0.0 %"),
            "fatigue": tk.StringVar(value="판정 대기 (유효 MDF 6개 필요)"),
            "system": tk.StringVar(value="대기"),
            "live": tk.StringVar(value="RMS --   MDF -- Hz   수축 --   IMU --"),
            "last": tk.StringVar(value="수신 대기"),
        }
        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self.poll_lines)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self):
        style = ttk.Style()
        style.configure("Title.TLabel", font=("Malgun Gothic", 18, "bold"))
        style.configure("CardTitle.TLabel", font=("Malgun Gothic", 11, "bold"))
        style.configure("CardValue.TLabel", font=("Malgun Gothic", 14))

        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="근피로 밴드 확인 화면", style="Title.TLabel").pack(anchor="w")

        connect = ttk.Frame(outer)
        connect.pack(fill="x", pady=(14, 8))
        self.port_box = ttk.Combobox(connect, width=18, state="readonly")
        self.port_box.pack(side="left")
        ttk.Button(connect, text="포트 새로고침", command=self.refresh_ports).pack(side="left", padx=6)
        self.connect_button = ttk.Button(connect, text="연결", command=self.toggle_connection)
        self.connect_button.pack(side="left")
        ttk.Label(connect, text="기준값 재측정: 기기의 확인 버튼(GPIO33)").pack(side="right")

        self._card(outer, "연결 상태", "connection")
        self._card(outer, "기준값 측정", "baseline")

        changes = ttk.Frame(outer)
        changes.pack(fill="x", pady=5)
        self._small_card(changes, "현재 MDF 변화율", "mdf_change").pack(side="left", fill="x", expand=True, padx=(0, 4))
        self._small_card(changes, "현재 RMS 변화율", "rms_change").pack(side="left", fill="x", expand=True, padx=(4, 0))

        self._card(outer, "근피로 판정", "fatigue")
        self._card(outer, "근피로 확정 기준 도달률", "fatigue_progress")
        self._card(outer, "시스템 동작", "system")
        self._card(outer, "현재 측정값", "live")
        self._card(outer, "최근 수신 메시지", "last", small=True)

        ttk.Label(
            outer,
            text="※ 이 프로그램을 사용할 때 Arduino IDE의 Serial Monitor를 닫으세요.\n"
                 "※ 발열 시험은 패드를 몸에서 떼고 사람이 지켜보는 상태에서 진행하세요.",
            foreground="#9b2c2c",
        ).pack(anchor="w", pady=(10, 0))

    def _card(self, parent, title, key, small=False):
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        frame.pack(fill="x", pady=5)
        style = "CardTitle.TLabel" if small else "CardValue.TLabel"
        ttk.Label(frame, textvariable=self.values[key], style=style, wraplength=650).pack(anchor="w")

    def _small_card(self, parent, title, key):
        frame = ttk.LabelFrame(parent, text=title, padding=10)
        ttk.Label(frame, textvariable=self.values[key], style="CardValue.TLabel").pack(anchor="w")
        return frame

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_box.get() not in ports:
            self.port_box.current(0)

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
            return
        port = self.port_box.get()
        if not port:
            messagebox.showwarning("포트 없음", "ESP32의 COM 포트를 선택하세요.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.25)
        except serial.SerialException as exc:
            messagebox.showerror("연결 실패", f"{port}를 열 수 없습니다.\n\n{exc}\n\nArduino Serial Monitor를 닫았는지 확인하세요.")
            return
        self.stop_event.clear()
        self.reader = threading.Thread(target=self.read_serial, daemon=True)
        self.reader.start()
        self.values["connection"].set(f"{port} / {BAUD} baud 연결됨")
        self.connect_button.configure(text="연결 해제")

    def disconnect(self):
        self.stop_event.set()
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.values["connection"].set("연결 안 됨")
        self.connect_button.configure(text="연결")

    def read_serial(self):
        while not self.stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                raw = self.ser.readline()
                if raw:
                    self.lines.put(raw.decode("utf-8", errors="replace").strip())
            except serial.SerialException as exc:
                self.lines.put(f"__ERROR__{exc}")
                break

    def poll_lines(self):
        for _ in range(100):
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            if line.startswith("__ERROR__"):
                self.values["connection"].set("연결 오류: " + line[9:])
                self.disconnect()
            elif line:
                self.parse_line(line)
        self.root.after(50, self.poll_lines)

    def parse_line(self, line):
        self.values["last"].set(line[:180])

        if line in ("RELAX_MUSCLE_FOR_RMS_BASELINE", "RMS_RECALIBRATING",
                    "RMS_RECALIBRATING_BUTTON"):
            self.values["baseline"].set("측정 중 — 근육에 힘을 빼고 움직이지 마세요")
            self.values["fatigue"].set("판정 대기 (새 기준값 측정 중)")

        if line.startswith("RMS_BASELINE_QUALITY_FAILED"):
            self.values["baseline"].set("기준값 측정 실패 — 자세를 고정하고 다시 측정하세요")

        if line.startswith("기준값측정완료,"):
            mean = re.search(r"평균RMS=([-+0-9.eE]+)", line)
            std = re.search(r"표준편차=([-+0-9.eE]+)", line)
            self.values["baseline"].set(
                f"완료  |  평균 RMS {mean.group(1) if mean else '?'}  |  "
                f"표준편차 {std.group(1) if std else '?'}"
            )

        if line.startswith("status="):
            ready = re.search(r"baselineReady=(\d+)", line)
            mean = re.search(r"baselineMean=([-+0-9.eE]+)", line)
            std = re.search(r"baselineStd=([-+0-9.eE]+)", line)
            if ready and ready.group(1) == "1":
                mean_text = mean.group(1) if mean else "?"
                std_text = std.group(1) if std else "?"
                self.values["baseline"].set(f"완료  |  평균 RMS {mean_text}  |  표준편차 {std_text}")

        if line.startswith("FATIGUE_TREND,"):
            mdf = re.search(r"MDF_CHANGE_PERCENT=([-+0-9.]+)", line)
            rms = re.search(r"RMS_CHANGE_PERCENT=([-+0-9.]+)", line)
            fatigue = re.search(r"FATIGUED=(\d+)", line)
            if mdf:
                mdf_value = float(mdf.group(1))
                self.values["mdf_change"].set(f"{mdf_value:+.1f} %")
                progress = max(0.0, min(100.0, (-mdf_value / 15.0) * 100.0))
                self.values["fatigue_progress"].set(f"{progress:.1f} %")
            if rms:
                self.values["rms_change"].set(f"{float(rms.group(1)):+.1f} %")
            if fatigue:
                self.values["fatigue"].set("근피로 확정" if fatigue.group(1) == "1" else "근피로 조건 미충족")

        if line.startswith("STAGE5_SYSTEM_STATE="):
            try:
                state = int(line.split("=", 1)[1].split(",", 1)[0])
                self.values["system"].set(SYSTEM_STATES.get(state, f"알 수 없는 상태 {state}"))
            except ValueError:
                pass

        if line.startswith("DATA,"):
            parts = line.split(",")
            if len(parts) >= 8:
                try:
                    rms = float(parts[2])
                    contracting = "예" if int(parts[3]) else "아니오"
                    mdf = float(parts[6])
                    imu = "정상" if int(parts[7]) == 0 else f"오류({parts[7]})"
                    self.values["live"].set(f"RMS {rms:.6f}   MDF {mdf:.1f} Hz   수축 {contracting}   IMU {imu}")
                except (ValueError, IndexError):
                    pass

    def close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    app_root = tk.Tk()
    StatusMonitor(app_root)
    app_root.mainloop()
