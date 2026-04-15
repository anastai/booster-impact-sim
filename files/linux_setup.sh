#!/bin/bash
set -e

# ── 1. System packages ─────────────────────────────────────────────────────────
sudo apt update && sudo apt install -y \
    python3 python3-pip python3-venv git

# ── 2. VS Code ────────────────────────────────────────────────────────────────
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
echo "deb [arch=amd64] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update && sudo apt install -y code
rm packages.microsoft.gpg

# ── 3. Clone repo ──────────────────────────────────────────────────────────────
git clone https://github.com/anastai/booster-impact-sim.git
cd booster-impact-sim

# ── 4. Virtual environment ─────────────────────────────────────────────────────
python3 -m venv .venv
source .venv/bin/activate

# ── 5. Python dependencies ─────────────────────────────────────────────────────
pip install --upgrade pip
pip install matplotlib scipy numpy pytest casadi jupyter python-docx

# ── 6. Verify ──────────────────────────────────────────────────────────────────
python -m pytest tests/check_vectors.py -q
echo ""
echo "Setup complete. To start:"
echo "  cd booster-impact-sim"
echo "  source .venv/bin/activate"
echo "  jupyter notebook simulation.ipynb"
