#!/bin/bash
set -e

# ── 1. System packages ─────────────────────────────────────────────────────────
sudo apt update && sudo apt install -y \
    python3 python3-pip python3-venv git curl

# ── 2. VS Code ────────────────────────────────────────────────────────────────
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
echo "deb [arch=amd64] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update && sudo apt install -y code
rm packages.microsoft.gpg

# ── 3. Node.js + Claude Code ──────────────────────────────────────────────────
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt install -y nodejs
sudo npm install -g @anthropic-ai/claude-code

# ── 4. Clone repo ──────────────────────────────────────────────────────────────
git clone https://github.com/anastai/booster-impact-sim.git
cd booster-impact-sim

# ── 5. Virtual environment ─────────────────────────────────────────────────────
python3 -m venv .venv
source .venv/bin/activate

# ── 6. Python dependencies ─────────────────────────────────────────────────────
pip install --upgrade pip
pip install matplotlib scipy numpy pytest casadi jupyter python-docx ipykernel

# ── 7. Register venv as Jupyter kernel ────────────────────────────────────────
python -m ipykernel install --user --name=booster-venv --display-name "Booster Sim (.venv)"

# ── 8. Verify ──────────────────────────────────────────────────────────────────
python -m pytest tests/check_vectors.py -q
echo ""
echo "Setup complete. To start:"
echo "  cd booster-impact-sim"
echo "  source .venv/bin/activate"
echo "  jupyter notebook simulation.ipynb"
echo "  claude   (to open Claude Code)"
