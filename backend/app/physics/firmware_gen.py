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
