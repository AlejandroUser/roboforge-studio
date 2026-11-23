import React, { useCallback, useEffect, useRef, useState } from "react";
import { api } from "../api";
import type { HardwarePortInfo, LiveArmPose } from "../types";

export interface LiveHardwarePanelProps {
  pose: LiveArmPose;
}

interface TxLogEntry {
  id: number;
  time: string;
  frame: string;
  target: string;
  ok: boolean;
}

const DEFAULT_VIRTUAL_PORT: HardwarePortInfo = {
  port: "VIRTUAL_SIM",
  description: "Simulador USB RoboForge (Sin placa física)",
};

function formatPoseFrame(pose: LiveArmPose): string {
  const j0 = Math.round(pose.j0_base);
  const j1 = Math.round(pose.j1_shoulder);
  const j2 = Math.round(pose.j2_elbow);
  const j3 = Math.round(pose.j3_wrist);
  const g = Math.round(pose.gripper_pct);
  return `P:${j0},${j1},${j2},${j3},${g}`;
}

export const LiveHardwarePanel: React.FC<LiveHardwarePanelProps> = ({ pose }) => {
  const hasWebSerial = typeof navigator !== "undefined" && "serial" in navigator;
  const [transport, setTransport] = useState<"backend" | "webserial">("backend");
  const [ports, setPorts] = useState<HardwarePortInfo[]>([DEFAULT_VIRTUAL_PORT]);
  const [selectedPort, setSelectedPort] = useState<string>("VIRTUAL_SIM");
  const [baudrate, setBaudrate] = useState<number>(115200);
  const [liveStream, setLiveStream] = useState<boolean>(false);
  const [sending, setSending] = useState<boolean>(false);
  const [statusMsg, setStatusMsg] = useState<string>("Listo (Simulador Virtual USB)");
  const [txLogs, setTxLogs] = useState<TxLogEntry[]>([]);

  // WebSerial refs
  const serialPortRef = useRef<any>(null);
  const serialWriterRef = useRef<WritableStreamDefaultWriter<Uint8Array> | null>(null);
  const [webSerialConnected, setWebSerialConnected] = useState<boolean>(false);

  const lastSentFrameRef = useRef<string>("");
  const streamTimerRef = useRef<number | null>(null);

  const appendLog = useCallback((frame: string, target: string, ok: boolean) => {
    const now = new Date();
    const time = now.toTimeString().slice(0, 8);
    setTxLogs((prev) => [{ id: Date.now() + Math.random(), time, frame, target, ok }, ...prev].slice(0, 8));
  }, []);

  const refreshPorts = useCallback(async () => {
    try {
      const res = await api.getHardwarePorts();
      const list = Array.isArray(res) ? res : res?.ports || [];
      const merged = list.length > 0 ? list : [DEFAULT_VIRTUAL_PORT];
      setPorts(merged);
      if (!merged.some((p) => p.port === selectedPort)) {
        setSelectedPort(merged[0].port);
      }
    } catch {
      setPorts([DEFAULT_VIRTUAL_PORT]);
      setSelectedPort("VIRTUAL_SIM");
    }
  }, [selectedPort]);

  useEffect(() => {
    refreshPorts();
  }, [refreshPorts]);

  const disconnectWebSerial = useCallback(async () => {
    try {
      if (serialWriterRef.current) {
        await serialWriterRef.current.close().catch(() => {});
        serialWriterRef.current.releaseLock();
        serialWriterRef.current = null;
      }
      if (serialPortRef.current) {
        await serialPortRef.current.close().catch(() => {});
        serialPortRef.current = null;
      }
    } finally {
      setWebSerialConnected(false);
      setLiveStream(false);
      setStatusMsg("WebSerial desconectado.");
    }
  }, []);

  const connectWebSerial = useCallback(async () => {
    if (!hasWebSerial) {
      setStatusMsg("Este navegador no soporta WebSerial API (usa Chrome/Edge o Puente Backend).");
      return;
    }
    try {
      const port = await (navigator as any).serial.requestPort();
      await port.open({ baudRate: baudrate });
      const writer = port.writable?.getWriter();
      serialPortRef.current = port;
      serialWriterRef.current = writer || null;
      setWebSerialConnected(true);
      setStatusMsg(`WebSerial conectado a ${baudrate} baudios.`);
    } catch (err: any) {
      setStatusMsg(`Conexión WebSerial cancelada o fallida: ${err?.message || "error"}`);
    }
  }, [hasWebSerial, baudrate]);

  useEffect(() => {
    return () => {
      if (serialWriterRef.current) {
        serialWriterRef.current.releaseLock();
      }
    };
  }, []);

  const sendCurrentPose = useCallback(async () => {
    const frame = formatPoseFrame(pose);
    setSending(true);
    try {
      if (transport === "webserial") {
        if (!webSerialConnected || !serialWriterRef.current) {
          setStatusMsg("Conecta primero un puerto WebSerial.");
          return;
        }
        const encoded = new TextEncoder().encode(`${frame}\n`);
        await serialWriterRef.current.write(encoded);
        lastSentFrameRef.current = frame;
        appendLog(frame, `WebSerial @${baudrate}`, true);
        setStatusMsg(`Enviado vía WebSerial (${frame})`);
      } else {
        const res = await api.sendHardwarePose({
          port: selectedPort,
          baudrate,
          j0_base: Math.round(pose.j0_base),
          j1_shoulder: Math.round(pose.j1_shoulder),
          j2_elbow: Math.round(pose.j2_elbow),
          j3_wrist: Math.round(pose.j3_wrist),
          gripper_pct: Math.round(pose.gripper_pct),
        });
        const sentFrame = (res.frame || frame).trim();
        lastSentFrameRef.current = frame;
        appendLog(sentFrame, `${res.port || selectedPort} (${res.mode || "sim"})`, Boolean(res.ok));
        setStatusMsg(res.ok ? `TX OK → ${res.port} [${res.mode}]` : `Error: ${res.error || "fallo TX"}`);
      }
    } catch (err: any) {
      appendLog(frame, selectedPort, false);
      setStatusMsg(`Error enviando pose: ${err?.message || "sin conexión"}`);
    } finally {
      setSending(false);
    }
  }, [pose, transport, webSerialConnected, baudrate, selectedPort, appendLog]);

  // Throttled live stream when FK/IK sliders move
  useEffect(() => {
    if (!liveStream) return;
    const frame = formatPoseFrame(pose);
    if (frame === lastSentFrameRef.current) return;

    if (streamTimerRef.current) {
      window.clearTimeout(streamTimerRef.current);
    }
    streamTimerRef.current = window.setTimeout(() => {
      sendCurrentPose();
    }, 110);

    return () => {
      if (streamTimerRef.current) {
        window.clearTimeout(streamTimerRef.current);
      }
    };
  }, [liveStream, pose, sendCurrentPose]);

  return (
    <div className="mech-pane">
      {/* Transport selector */}
      <div className="two-col">
        <button
          type="button"
          className={`btn-tool ${transport === "backend" ? "active" : ""}`}
          onClick={() => setTransport("backend")}
        >
          Puente Backend (USB/Sim)
        </button>
        <button
          type="button"
          className={`btn-tool ${transport === "webserial" ? "active" : ""}`}
          onClick={() => setTransport("webserial")}
          title={hasWebSerial ? "Conexión directa desde Chrome/Edge" : "Requiere Chrome/Edge con WebSerial"}
        >
          WebSerial {hasWebSerial ? "✓" : "(N/D)"}
        </button>
      </div>

      {/* Port + Baudrate */}
      {transport === "backend" ? (
        <div className="field-row">
          <div className="field-head">
            <span>Puerto Serial / Simulador</span>
            <button
              type="button"
              className="btn-tool"
              style={{ padding: "2px 6px", fontSize: 10 }}
              onClick={refreshPorts}
              title="Actualizar lista de puertos COM/USB"
            >
              ↻ Escanear
            </button>
          </div>
          <select value={selectedPort} onChange={(e) => setSelectedPort(e.target.value)}>
            {ports.map((p) => (
              <option key={p.port} value={p.port}>
                {p.port} — {p.description}
              </option>
            ))}
          </select>
        </div>
      ) : (
        <div className="two-col">
          <button
            type="button"
            className="btn-primary"
            disabled={!hasWebSerial || webSerialConnected}
            onClick={connectWebSerial}
          >
            {webSerialConnected ? "✓ Conectado" : "Conectar USB"}
          </button>
          <button
            type="button"
            className="btn-tool"
            disabled={!webSerialConnected}
            onClick={disconnectWebSerial}
          >
            Desconectar
          </button>
        </div>
      )}

      <div className="two-col">
        <div className="field-row">
          <div className="field-head"><span>Baudrate</span></div>
          <select value={baudrate} onChange={(e) => setBaudrate(Number(e.target.value))}>
            <option value={9600}>9600 bps</option>
            <option value={57600}>57600 bps</option>
            <option value={115200}>115200 bps (ESP32/UNO)</option>
            <option value={250000}>250000 bps</option>
          </select>
        </div>
        <div className="field-row">
          <div className="field-head"><span>Trama Actual</span></div>
          <div
            style={{
              padding: "6px 8px",
              borderRadius: 6,
              background: "rgba(15, 23, 42, 0.7)",
              border: "1px solid rgba(56, 189, 248, 0.28)",
              fontFamily: "monospace",
              fontSize: 11,
              color: "#38bdf8",
              textAlign: "center",
            }}
          >
            {formatPoseFrame(pose)}
          </div>
        </div>
      </div>

      {/* Actions */}
      <div className="two-col">
        <button
          type="button"
          className="btn-primary"
          disabled={sending || (transport === "webserial" && !webSerialConnected)}
          onClick={sendCurrentPose}
        >
          {sending ? "Enviando..." : "Enviar Pose Actual"}
        </button>
        <button
          type="button"
          className={`btn-tool ${liveStream ? "active" : ""}`}
          disabled={transport === "webserial" && !webSerialConnected}
          onClick={() => setLiveStream((v) => !v)}
          title="Envía la trama automáticamente al mover los sliders FK o el objetivo IK 3D"
        >
          {liveStream ? "● Live Stream ON" : "○ Live Stream"}
        </button>
      </div>

      <div className="psu-banner" style={{ fontSize: 11 }}>
        <span>Estado Enlace</span>
        <strong>{statusMsg}</strong>
      </div>

      {/* Mini TX Console */}
      <div
        style={{
          padding: "7px 8px",
          borderRadius: 7,
          background: "#070b12",
          border: "1px solid rgba(148, 163, 184, 0.2)",
          fontFamily: "monospace",
          fontSize: 10.5,
        }}
      >
        <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 4, color: "#94a3b8" }}>
          <span>Consola Serial TX →</span>
          {txLogs.length > 0 && (
            <button
              type="button"
              onClick={() => setTxLogs([])}
              style={{
                background: "transparent",
                border: "none",
                color: "#64748b",
                cursor: "pointer",
                fontSize: 10,
                padding: 0,
              }}
            >
              Limpiar
            </button>
          )}
        </div>
        {txLogs.length === 0 ? (
          <div style={{ color: "#475569" }}>Sin tramas enviadas aún. Pulsa «Enviar Pose Actual» o activa Live Stream.</div>
        ) : (
          <div style={{ display: "flex", flexDirection: "column", gap: 3, maxHeight: 115, overflowY: "auto" }}>
            {txLogs.map((log) => (
              <div
                key={log.id}
                style={{
                  display: "flex",
                  justifyContent: "space-between",
                  gap: 6,
                  color: log.ok ? "#34d399" : "#f87171",
                }}
              >
                <span>
                  [{log.time}] TX → {log.frame}
                </span>
                <span style={{ color: "#64748b" }}>{log.target}</span>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};
