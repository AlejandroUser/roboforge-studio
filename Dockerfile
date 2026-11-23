# ==============================================================================
# Stage 1: Build React + Three.js (R3F) Frontend
# ==============================================================================
FROM node:22-slim AS frontend-builder

WORKDIR /build/frontend-react

COPY frontend-react/package.json frontend-react/package-lock.json ./
RUN npm ci

COPY frontend-react/ ./
RUN npx tsc --noEmit && npm run build

# ==============================================================================
# Stage 2: Python 3.12 Runtime (FastAPI + build123d / OCCT + Gmsh + FEA)
# Compatible with Docker local and HuggingFace Spaces (port 7860, UID 1000)
# ==============================================================================
FROM python:3.12-slim AS runtime

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app:/app/backend \
    PORT=7860

WORKDIR /app

# Runtime OpenGL / X11 libraries required by OpenCASCADE (build123d) & Gmsh
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglu1-mesa \
    libglib2.0-0 \
    libxrender1 \
    libxft2 \
    libxcursor1 \
    libxinerama1 \
    && rm -rf /var/lib/apt/lists/*

COPY backend/requirements.txt ./backend/requirements.txt
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r backend/requirements.txt

COPY backend/ ./backend/
COPY --from=frontend-builder /build/frontend-react/dist ./frontend-react/dist

# Ensure exports directory is writable (including non-root UID 1000 on HuggingFace Spaces)
RUN mkdir -p /app/exports && chmod -R 777 /app/exports

EXPOSE 7860

CMD ["uvicorn", "backend.app.main:app", "--host", "0.0.0.0", "--port", "7860"]
