"""RoboForge Studio — Native Windows Desktop Application (pywebview + embedded FastAPI)."""

from __future__ import annotations

import socket
import sys
import threading
import time
import urllib.request
import webbrowser
from pathlib import Path

import uvicorn
import webview

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Direct import ensures PyInstaller traces the entire FastAPI + CAD + Physics backend
from backend.app.main import app as roboforge_app  # noqa: E402


def _find_free_port(preferred: int = 8765) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        if s.connect_ex(("127.0.0.1", preferred)) != 0:
            return preferred
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _wait_for_server(url: str, timeout_s: float = 25.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(f"{url}/api/health", timeout=1.0) as resp:
                if resp.status == 200:
                    return True
        except Exception:
            time.sleep(0.15)
    return False


def main() -> None:
    web_mode = "--web" in sys.argv
    port = _find_free_port(8000 if web_mode else 8765)
    base_url = f"http://127.0.0.1:{port}"

    # log_config=None prevents uvicorn from calling sys.stdout.isatty() in PyInstaller --windowed mode
    config = uvicorn.Config(
        roboforge_app,
        host="127.0.0.1",
        port=port,
        log_level="warning",
        log_config=None,
    )
    server = uvicorn.Server(config)

    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()

    if not _wait_for_server(base_url, timeout_s=30.0):
        sys.exit(1)

    if web_mode:
        print(f"[RoboForge] Servidor listo en {base_url} (Ctrl+C para cerrar)")
        webbrowser.open(base_url)
        try:
            while server_thread.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            server.should_exit = True
            server_thread.join(timeout=3.0)
        return

    # Enable file downloads (.STL, .3MF, .STEP, .ZIP, .INO, .MD) inside pywebview/WebView2
    webview.settings["ALLOW_DOWNLOADS"] = True

    webview.create_window(
        title="RoboForge Studio Pro v10.0 — Ingeniería Computacional CAD & Robótica",
        url=base_url,
        width=1480,
        height=920,
        min_size=(1120, 720),
        background_color="#090d14",
        resizable=True,
        confirm_close=False,
    )

    try:
        webview.start(gui="edgechromium", debug=False)
    finally:
        server.should_exit = True
        server_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
