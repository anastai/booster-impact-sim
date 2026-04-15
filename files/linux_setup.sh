#!/bin/bash
set -e

# ── 1. System packages ─────────────────────────────────────────────────────────
sudo apt update && sudo apt install -y \
    python3 python3-pip python3-venv git

# ── 2. Clone repo ──────────────────────────────────────────────────────────────
git clone https://github.com/anastai/booster-impact-sim.git
cd booster-impact-sim

# ── 3. Virtual environment ─────────────────────────────────────────────────────
python3 -m venv .venv
source .venv/bin/activate

# ── 4. Python dependencies ─────────────────────────────────────────────────────
pip install --upgrade pip
pip install matplotlib scipy numpy pytest casadi jupyter python-docx

# ── 5. Verify ──────────────────────────────────────────────────────────────────
python -m pytest tests/check_vectors.py -q
echo ""
echo "Setup complete. To start:"
echo "  cd booster-impact-sim"
echo "  source .venv/bin/activate"
echo "  jupyter notebook simulation.ipynb"
