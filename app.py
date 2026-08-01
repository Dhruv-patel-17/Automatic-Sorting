"""
VisionSort SCADA Dashboard - Flask Backend
================================================================
Serves a live, auto-refreshing SCADA-style dashboard for the VisionSort
sorting line: classification history/graphs from the CSV log, plus
(best-effort) live PLC tag values read directly from OpenPLC over
Modbus TCP, the same way a real SCADA screen would poll a PLC.

Run:
  pip install flask pymodbus pandas
  python app.py
Then open http://127.0.0.1:5000 in a browser.

The dashboard keeps working even if OpenPLC isn't running - the PLC
status panel simply shows "PLC: NOT CONNECTED" and the log/graphs still
work independently, since they only depend on the CSV file.
"""

import os
import csv
from flask import Flask, render_template, jsonify

try:
    from pymodbus.client import ModbusTcpClient
    PYMODBUS_AVAILABLE = True
except ImportError:
    PYMODBUS_AVAILABLE = False

app = Flask(__name__)

# =================================================================
# CONFIGURATION - match these to your actual pipeline setup
# =================================================================
LOG_FILE = os.path.join(os.path.dirname(__file__), "classification_log.csv")

PLC_IP = "127.0.0.1"
PLC_PORT = 503

# Modbus points to poll for the live SCADA status panel - matches the
# I/O tag table in the project README
PLC_TAGS = {
    "ENTRY_SEN":          {"type": "discrete_input", "address": 3},
    "CONV_MOTOR":         {"type": "coil",            "address": 0},
    "REJECT_SOL":         {"type": "coil",            "address": 1},
    "VISION_REJECT_FLAG": {"type": "coil",            "address": 3},
    "VISION_HEARTBEAT":   {"type": "coil",            "address": 4},
    "VISION_FAULT_LAMP":  {"type": "coil",            "address": 5},
}


def read_log():
    """Reads the classification log CSV and returns a list of dict rows."""
    if not os.path.exists(LOG_FILE):
        return []
    with open(LOG_FILE, newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def outcome_of(row):
    result = (row.get("result") or "").upper()
    if "UNRECOGNIZED" in result:
        return "unknown"
    if "DIVERT" in result:
        return "reject"
    if "PASS" in result:
        return "pass"
    classification = (row.get("classification") or "").upper()
    if classification == "UNKNOWN":
        return "unknown"
    return "pass"


@app.route("/")
def dashboard():
    return render_template("dashboard.html")


@app.route("/api/log")
def api_log():
    """Returns the full classification log plus computed summary stats."""
    rows = read_log()
    for r in rows:
        r["outcome"] = outcome_of(r)

    total = len(rows)
    pass_count = sum(1 for r in rows if r["outcome"] == "pass")
    reject_count = sum(1 for r in rows if r["outcome"] == "reject")
    unknown_count = sum(1 for r in rows if r["outcome"] == "unknown")
    lid_count = sum(1 for r in rows if (r.get("classification") or "").upper() == "LID")
    base_count = sum(1 for r in rows if (r.get("classification") or "").upper() == "BASE")

    return jsonify({
        "rows": rows[-500:],   # cap payload size for very long-running sessions
        "summary": {
            "total": total,
            "pass": pass_count,
            "reject": reject_count,
            "unknown": unknown_count,
            "lid": lid_count,
            "base": base_count,
            "pass_rate": round(pass_count / total * 100, 1) if total else 0,
        }
    })


@app.route("/api/plc_status")
def api_plc_status():
    """
    Best-effort live read of PLC tag values over Modbus TCP - if OpenPLC
    isn't reachable, returns connected: false rather than erroring, so
    the dashboard's log/graphs keep working regardless.
    """
    if not PYMODBUS_AVAILABLE:
        return jsonify({"connected": False, "reason": "pymodbus not installed"})

    client = ModbusTcpClient(PLC_IP, port=PLC_PORT)
    if not client.connect():
        return jsonify({"connected": False})

    values = {}
    try:
        for name, cfg in PLC_TAGS.items():
            if cfg["type"] == "coil":
                result = client.read_coils(cfg["address"], count=1)
            else:
                result = client.read_discrete_inputs(cfg["address"], count=1)
            values[name] = bool(result.bits[0]) if not result.isError() else None
    except Exception as e:
        client.close()
        return jsonify({"connected": False, "error": str(e)})

    client.close()
    return jsonify({"connected": True, "tags": values})


if __name__ == "__main__":
    app.run(debug=True, port=5000)
