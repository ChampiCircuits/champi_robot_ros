from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, List
import json
import os
from generator import generate_cpp_code
from builder import build_and_flash

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class Waypoint(BaseModel):
    x: float
    y: float
    waitS: float = 0.0

class ConfigData(BaseModel):
    trajectories: Dict[str, List[Waypoint]]
    globalSpeed: float
    angularSpeedDegS: float
    delayAfterPullCordS: float

DATA_FILE = os.path.join(os.path.dirname(__file__), "data", "config.json")

@app.get("/api/config")
def load_config():
    default_config = {
        "trajectories": {str(i): [] for i in range(1, 7)},
        "globalSpeed": 10,
        "angularSpeedDegS": 70,
        "delayAfterPullCordS": 3,
    }

    if os.path.exists(DATA_FILE):
        with open(DATA_FILE, "r") as f:
            loaded = json.load(f)
            return {**default_config, **loaded}

    return default_config

@app.post("/api/config")
def save_config(config: ConfigData):
    with open(DATA_FILE, "w") as f:
         json.dump(config.model_dump(), f)
    return {"status": "ok"}

@app.post("/api/flash/{pami_id}")
def flash_pami_endpoint(pami_id: int):
    if not os.path.exists(DATA_FILE):
        raise HTTPException(status_code=400, detail="Veuillez d'abord sauvegarder la configuration.")
        
    with open(DATA_FILE, "r") as f:
        config_data = json.load(f)
        
    generate_cpp_code(pami_id, config_data)
    success, logs = build_and_flash()
    
    if not success:
        raise HTTPException(status_code=500, detail=f"Erreur de compilation/flash:\n{logs}")
    
    return {"status": "ok", "logs": logs}

