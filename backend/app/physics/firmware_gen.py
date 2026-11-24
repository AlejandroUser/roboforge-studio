from __future__ import annotations

from typing import Any


def _clamp_servo_deg(val: float, lo: int = 0, hi: int = 180) -> int:
    return max(lo, min(hi, int(round(val))))


def _pose_to_servo_angles(pose: dict[str, Any]) -> tuple[int, int, int, int, int]:
    """
    Convert RoboForge UI joint coordinates to 0..180 servo degrees for Brazo 5.0:
    - j0_base (-180..180 deg) -> 0..180 (90 = center)
    - j1_shoulder (-60..90 deg) -> 30..180 (90 + j1)
    - j2_elbow (-130..130 deg) -> 15..165 (90 + j2 * 0.6)
    - j3_wrist (-90..90 deg) -> 0..180 (90 + j3)
    - gripper_pct (0..100 %) -> 25..135 deg (25=closed, 135=open)
    """
    j0 = float(pose.get("j0_base", 0.0))
    j1 = float(pose.get("j1_shoulder", 35.0))
    j2 = float(pose.get("j2_elbow", -55.0))
    j3 = float(pose.get("j3_wrist", 20.0))
    grip = float(pose.get("gripper_pct", 50.0))

    s_base = _clamp_servo_deg(90.0 + j0 * 0.5, 0, 180)
    s_shoulder = _clamp_servo_deg(90.0 + j1, 15, 165)
    s_elbow = _clamp_servo_deg(90.0 + j2 * 0.6, 15, 165)
    s_wrist = _clamp_servo_deg(90.0 + j3, 10, 170)
    s_grip = _clamp_servo_deg(25.0 + (grip / 100.0) * 110.0, 20, 145)
    return s_base, s_shoulder, s_elbow, s_wrist, s_grip


def generate_arduino_pca9685_ino(
    arm_name: str,
    poses: list[dict[str, Any]],
    speed_ms: int = 18,
) -> str:
    """
    Generate a complete, compilable Arduino IDE (.ino) sketch for the Brazo 5.0
    using <Wire.h> and <Adafruit_PWMServoDriver.h> (PCA9685 @ 0x40, 50 Hz).
    Maps CH0..CH7 with automatic inversion (180 - ang) on dual-servo channels
    CH2 (Shoulder B) and CH4 (Elbow B), plus smooth step interpolation in loop().
    """
    safe_name = (arm_name or "Brazo Robotico 5.0").replace("\n", " ").strip()
    step_delay = max(4, min(120, int(speed_ms)))

    effective_poses = poses if poses else [
        {"j0_base": 0.0, "j1_shoulder": 35.0, "j2_elbow": -55.0, "j3_wrist": 20.0, "gripper_pct": 80.0},
        {"j0_base": -45.0, "j1_shoulder": 15.0, "j2_elbow": -35.0, "j3_wrist": 15.0, "gripper_pct": 15.0},
        {"j0_base": 45.0, "j1_shoulder": 45.0, "j2_elbow": -65.0, "j3_wrist": 25.0, "gripper_pct": 15.0},
        {"j0_base": 45.0, "j1_shoulder": 15.0, "j2_elbow": -35.0, "j3_wrist": 15.0, "gripper_pct": 85.0},
    ]

    pose_rows: list[str] = []
    for idx, p in enumerate(effective_poses):
        sb, sh, se, sw, sg = _pose_to_servo_angles(p)
        comma = "," if idx < len(effective_poses) - 1 else ""
        pose_rows.append(
            f"  {{{sb:3d}, {sh:3d}, {se:3d}, {sw:3d}, {sg:3d}}}{comma}  // Pose {idx + 1}"
        )
    poses_block = "\n".join(pose_rows)

    return f"""/*
 * ============================================================================
 *  RoboForge Studio Pro v4.2 — Firmware Arduino UNO + PCA9685 (16ch I2C)
 *  Robot: {safe_name}
 *  Arquitectura: 8x Servos TD-8120MG (20 kg.cm @ 6.5V / 10A) + Pinza 2-Dedos
 *
 *  Mapa de Canales PCA9685 (Dir. I2C 0x40, 50 Hz):
 *    CH0 -> Base Yaw (J0)
 *    CH1 -> Hombro Servo A (J1)
 *    CH2 -> Hombro Servo B (J1 invertido automaticamente: 180 - ang)
 *    CH3 -> Codo Servo A (J2)
 *    CH4 -> Codo Servo B (J2 invertido automaticamente: 180 - ang)
 *    CH5 -> Muneca Pitch (J3)
 *    CH6 -> Muneca Roll (J4 neutro 90 deg)
 *    CH7 -> Pinza / Gripper (25 deg cerrada .. 135 deg abierta)
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Pulsos calibrados para TD-8120MG / DS3218MG a 50 Hz (12-bit: 0..4095)
const uint16_t SERVO_MIN_PULSE = 110;  // ~540 us (0 deg)
const uint16_t SERVO_MAX_PULSE = 510;  // ~2490 us (180 deg)

// Definicion de canales PCA9685
const uint8_t CH0_BASE_YAW     = 0;
const uint8_t CH1_SHOULDER_A   = 1;
const uint8_t CH2_SHOULDER_B   = 2;
const uint8_t CH3_ELBOW_A      = 3;
const uint8_t CH4_ELBOW_B      = 4;
const uint8_t CH5_WRIST_PITCH  = 5;
const uint8_t CH6_WRIST_ROLL   = 6;
const uint8_t CH7_GRIPPER      = 7;

const int SPEED_MS = {step_delay};
const int PAUSA_ENTRE_POSES_MS = 650;

struct Pose {{
  int base;
  int hombro;
  int codo;
  int muneca;
  int pinza;
}};

const Pose SECUENCIA[] = {{
{poses_block}
}};

const int NUM_POSES = sizeof(SECUENCIA) / sizeof(SECUENCIA[0]);
Pose poseActual = SECUENCIA[0];

uint16_t degToPulse(int ang) {{
  int clamped = constrain(ang, 0, 180);
  return (uint16_t)map(clamped, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}}

void aplicarPose(const Pose& p) {{
  pwm.setPWM(CH0_BASE_YAW,    0, degToPulse(p.base));

  // Doble servo paralelo en Hombro (CH2 invertido mecanicamente)
  pwm.setPWM(CH1_SHOULDER_A,  0, degToPulse(p.hombro));
  pwm.setPWM(CH2_SHOULDER_B,  0, degToPulse(180 - p.hombro));

  // Doble servo paralelo en Codo (CH4 invertido mecanicamente)
  pwm.setPWM(CH3_ELBOW_A,     0, degToPulse(p.codo));
  pwm.setPWM(CH4_ELBOW_B,     0, degToPulse(180 - p.codo));

  pwm.setPWM(CH5_WRIST_PITCH, 0, degToPulse(p.muneca));
  pwm.setPWM(CH6_WRIST_ROLL,  0, degToPulse(90));
  pwm.setPWM(CH7_GRIPPER,     0, degToPulse(p.pinza));
}}

void moverSuave(const Pose& objetivo, int pasoMs) {{
  int maxDelta = max(
    max(abs(objetivo.base - poseActual.base), abs(objetivo.hombro - poseActual.hombro)),
    max(abs(objetivo.codo - poseActual.codo), max(abs(objetivo.muneca - poseActual.muneca), abs(objetivo.pinza - poseActual.pinza)))
  );
  if (maxDelta <= 0) {{
    aplicarPose(objetivo);
    return;
  }}

  Pose inicio = poseActual;
  for (int step = 1; step <= maxDelta; step++) {{
    float t = (float)step / (float)maxDelta;
    // Curva suave ease-in-out cubica
    float s = t * t * (3.0f - 2.0f * t);
    poseActual.base   = inicio.base   + (int)round((objetivo.base   - inicio.base)   * s);
    poseActual.hombro = inicio.hombro + (int)round((objetivo.hombro - inicio.hombro) * s);
    poseActual.codo   = inicio.codo   + (int)round((objetivo.codo   - inicio.codo)   * s);
    poseActual.muneca = inicio.muneca + (int)round((objetivo.muneca - inicio.muneca) * s);
    poseActual.pinza  = inicio.pinza  + (int)round((objetivo.pinza  - inicio.pinza)  * s);
    aplicarPose(poseActual);
    delay(pasoMs);
  }}
  poseActual = objetivo;
  aplicarPose(poseActual);
}}

void setup() {{
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // 50 Hz estandar para servos RC TD-8120MG
  delay(200);

  Serial.println(F("RoboForge Studio v4.2 - Iniciando secuencia PCA9685..."));
  aplicarPose(poseActual);
  delay(800);
}}

void loop() {{
  for (int i = 0; i < NUM_POSES; i++) {{
    Serial.print(F("Moviendo a Pose #"));
    Serial.println(i + 1);
    moverSuave(SECUENCIA[i], SPEED_MS);
    delay(PAUSA_ENTRE_POSES_MS);
  }}
}}
"""


def generate_esp32_wifi_pca9685_ino(
    arm_name: str = "RoboForge Mars-Mini",
    poses: list[dict[str, Any]] | None = None,
    speed_ms: int = 18,
) -> str:
    """
    Generate a complete ESP32 Wi-Fi AP + embedded WebServer + PCA9685 I2C sketch
    (mars_mini_esp32_wifi.ino) inspired by EEZYbotARM-ESP32 / OmArm-Zero:
    - Creates Wi-Fi AP "RoboForge-MarsMini" (192.168.4.1)
    - Serves mobile-friendly dark HTML UI with live J0/J1/J2/Pinza sliders + Play Secuencia
    - Drives 3x MG996R (CH0..CH2) + 1x MG90S Gripper (CH3) via PCA9685 @ 0x40, 50 Hz.
    """
    safe_name = (arm_name or "RoboForge Mars-Mini").replace("\n", " ").strip()
    step_delay = max(4, min(120, int(speed_ms)))
    effective_poses = poses if poses else [
        {"j0_base": 0.0, "j1_shoulder": 35.0, "j2_elbow": -55.0, "j3_wrist": 20.0, "gripper_pct": 80.0},
        {"j0_base": -45.0, "j1_shoulder": 15.0, "j2_elbow": -35.0, "j3_wrist": 15.0, "gripper_pct": 15.0},
        {"j0_base": 45.0, "j1_shoulder": 40.0, "j2_elbow": -60.0, "j3_wrist": 20.0, "gripper_pct": 85.0},
    ]

    pose_rows: list[str] = []
    for idx, p in enumerate(effective_poses):
        sb, sh, se, _, sg = _pose_to_servo_angles(p)
        comma = "," if idx < len(effective_poses) - 1 else ""
        pose_rows.append(f"  {{{sb:3d}, {sh:3d}, {se:3d}, {sg:3d}}}{comma}  // Pose {idx + 1}")
    poses_block = "\n".join(pose_rows)

    return f"""/*
 * ============================================================================
 *  RoboForge Studio Pro v7.1 — Firmware ESP32 Wi-Fi + PCA9685 (Mars-Mini)
 *  Robot: {safe_name}
 *  Arquitectura: ESP32 DevKit + PCA9685 I2C (0x40) + 5V 5A + 3x MG996R + 1x MG90S
 *
 *  Conecta tu celular a Wi-Fi: "RoboForge-MarsMini" (clave: roboforge123)
 *  Abre en navegador: http://192.168.4.1
 *
 *  Canales PCA9685 (I2C SDA=GPIO21, SCL=GPIO22, 50 Hz):
 *    CH0 -> Base Yaw J0 (MG996R)
 *    CH1 -> Hombro J1 (MG996R)
 *    CH2 -> Codo 4-Barras J2 (MG996R)
 *    CH3 -> Pinza / Gripper (MG90S)
 * ============================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* AP_SSID = "RoboForge-MarsMini";
const char* AP_PASS = "roboforge123";

WebServer server(80);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const uint16_t SERVO_MIN_PULSE = 115;
const uint16_t SERVO_MAX_PULSE = 505;
const int SPEED_MS = {step_delay};

struct Pose {{
  int j0;
  int j1;
  int j2;
  int pinza;
}};

const Pose SECUENCIA[] = {{
{poses_block}
}};
const int NUM_POSES = sizeof(SECUENCIA) / sizeof(SECUENCIA[0]);
// Pose inicial alineada 1:1 con los sliders HTML (90/120/60/80°) para evitar saltos al encender
const Pose POSE_INICIAL = {{90, 120, 60, 80}};
Pose poseActual = POSE_INICIAL;

uint16_t degToPulse(int ang) {{
  int c = constrain(ang, 0, 180);
  return (uint16_t)map(c, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}}

void aplicarPose(const Pose& p) {{
  pwm.setPWM(0, 0, degToPulse(p.j0));
  pwm.setPWM(1, 0, degToPulse(p.j1));
  pwm.setPWM(2, 0, degToPulse(p.j2));
  pwm.setPWM(3, 0, degToPulse(p.pinza));
}}

void moverSuave(const Pose& obj, int pasoMs) {{
  int maxD = max(max(abs(obj.j0 - poseActual.j0), abs(obj.j1 - poseActual.j1)),
                 max(abs(obj.j2 - poseActual.j2), abs(obj.pinza - poseActual.pinza)));
  if (maxD <= 0) {{ aplicarPose(obj); return; }}
  Pose ini = poseActual;
  for (int s = 1; s <= maxD; s++) {{
    float t = (float)s / (float)maxD;
    float k = t * t * (3.0f - 2.0f * t);
    poseActual.j0    = ini.j0    + (int)round((obj.j0    - ini.j0)    * k);
    poseActual.j1    = ini.j1    + (int)round((obj.j1    - ini.j1)    * k);
    poseActual.j2    = ini.j2    + (int)round((obj.j2    - ini.j2)    * k);
    poseActual.pinza = ini.pinza + (int)round((obj.pinza - ini.pinza) * k);
    aplicarPose(poseActual);
    delay(pasoMs);
  }}
  poseActual = obj;
  aplicarPose(poseActual);
}}

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboForge Mars-Mini Wi-Fi</title>
<style>body{{background:#0b1120;color:#e2e8f0;font-family:system-ui,sans-serif;padding:16px;max-width:420px;margin:auto}}
h2{{color:#38bdf8;margin-bottom:4px}}label{{display:flex;justify-content:space-between;margin-top:12px;font-size:14px}}
input[type=range]{{width:100%;accent-color:#38bdf8}}button{{margin-top:18px;width:100%;padding:12px;border:0;border-radius:8px;
background:#0284c7;color:#fff;font-weight:700;font-size:15px;cursor:pointer}}</style></head><body>
<h2>RoboForge Mars-Mini</h2><p style="color:#94a3b8;font-size:12px">ESP32 + PCA9685 · Control Wi-Fi en Tiempo Real</p>
<label>J0 Base Yaw <span id="v0">90°</span></label><input id="s0" type="range" min="0" max="180" value="90" oninput="upd()">
<label>J1 Hombro <span id="v1">120°</span></label><input id="s1" type="range" min="20" max="165" value="120" oninput="upd()">
<label>J2 Codo 4-Barras <span id="v2">60°</span></label><input id="s2" type="range" min="20" max="165" value="60" oninput="upd()">
<label>Pinza (MG90S) <span id="v3">80°</span></label><input id="s3" type="range" min="25" max="140" value="80" oninput="upd()">
<button onclick="fetch('/play')">▶ Play Secuencia ({len(effective_poses)} Poses)</button>
<script>function upd(){{let a=s0.value,b=s1.value,c=s2.value,d=s3.value;v0.innerText=a+'°';v1.innerText=b+'°';v2.innerText=c+'°';v3.innerText=d+'°';
fetch(`/set?j0=${{a}}&j1=${{b}}&j2=${{c}}&p=${{d}}`);}}</script></body></html>
)rawliteral";

void handleRoot() {{ server.send_P(200, "text/html", HTML_PAGE); }}

void handleSet() {{
  Pose p = poseActual;
  if (server.hasArg("j0")) p.j0 = server.arg("j0").toInt();
  if (server.hasArg("j1")) p.j1 = server.arg("j1").toInt();
  if (server.hasArg("j2")) p.j2 = server.arg("j2").toInt();
  if (server.hasArg("p"))  p.pinza = server.arg("p").toInt();
  poseActual = p;
  aplicarPose(p);
  server.send(200, "text/plain", "OK");
}}

void handlePlay() {{
  for (int i = 0; i < NUM_POSES; i++) {{
    moverSuave(SECUENCIA[i], SPEED_MS);
    delay(450);
  }}
  server.send(200, "text/plain", "DONE");
}}

void setup() {{
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  poseActual = POSE_INICIAL;
  aplicarPose(poseActual);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("RoboForge Mars-Mini AP IP: "));
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/play", handlePlay);
  server.begin();
}}

void loop() {{
  server.handleClient();
}}
"""

