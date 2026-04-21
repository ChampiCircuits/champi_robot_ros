#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$SCRIPT_DIR/backend"
FRONTEND_DIR="$SCRIPT_DIR/frontend"

if ! command -v python3 >/dev/null 2>&1; then
  echo "Error: python3 is required." >&2
  exit 1
fi

install_npm_if_missing() {
  if command -v npm >/dev/null 2>&1; then
    return 0
  fi

  echo "npm not found. Attempting automatic installation..."

  if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update && sudo apt-get install -y npm
  fi

  if ! command -v npm >/dev/null 2>&1; then
    echo "Error: npm installation failed. Please install npm manually and rerun this script." >&2
    return 1
  fi

  echo "npm installation successful."
}

install_npm_if_missing

if [ ! -d "$BACKEND_DIR" ] || [ ! -d "$FRONTEND_DIR" ]; then
  echo "Error: backend or frontend directory not found next to this script." >&2
  exit 1
fi

BACKEND_PID=""
FRONTEND_PID=""
CLEANED_UP=0

cleanup() {
  if [ "$CLEANED_UP" -eq 1 ]; then
    return
  fi
  CLEANED_UP=1

  echo
  echo "Stopping services..."

  if [ -n "$BACKEND_PID" ] && kill -0 "$BACKEND_PID" 2>/dev/null; then
    kill "$BACKEND_PID" 2>/dev/null || true
  fi

  if [ -n "$FRONTEND_PID" ] && kill -0 "$FRONTEND_PID" 2>/dev/null; then
    kill "$FRONTEND_PID" 2>/dev/null || true
  fi

  wait "$BACKEND_PID" 2>/dev/null || true
  wait "$FRONTEND_PID" 2>/dev/null || true
}

trap cleanup INT TERM EXIT

echo "Starting backend on http://localhost:8000 ..."
(
  cd "$BACKEND_DIR"
  python3 -m uvicorn main:app --reload --host 0.0.0.0 --port 8000 2>&1 | sed -u 's/^/[backend] /'
) &
BACKEND_PID=$!

echo "Starting frontend on http://localhost:5173 ..."
(
  cd "$FRONTEND_DIR"
  npm run dev 2>&1 | sed -u 's/^/[frontend] /'
) &
FRONTEND_PID=$!

echo "Backend PID: $BACKEND_PID"
echo "Frontend PID: $FRONTEND_PID"
echo "Press Ctrl+C to stop both services."

while true; do
  if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
    echo "Backend exited. Stopping frontend..." >&2
    exit 1
  fi

  if ! kill -0 "$FRONTEND_PID" 2>/dev/null; then
    echo "Frontend exited. Stopping backend..." >&2
    exit 1
  fi

  sleep 1
done


