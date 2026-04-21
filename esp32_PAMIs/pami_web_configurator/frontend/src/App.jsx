import React, { useState, useEffect, useRef } from 'react';
import TableCanvas from './components/TableCanvas.jsx';
import Timeline from './components/Timeline.jsx';
import ConfigPanel from './components/ConfigPanel.jsx';

const API_URL = 'http://localhost:8000/api';

const calculateDistance = (pts) => {
  let d = 0;
  if (!pts) return d;
  for (let i = 1; i < pts.length; i++) {
    d += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  return d;
};

function App() {
  const [selectedPami, setSelectedPami] = useState(1);
  const [trajectories, setTrajectories] = useState({
    1: [], 2: [], 3: [], 4: [], 5: [], 6: []
  });
  const [elapsedTime, setElapsedTime] = useState(0); // in seconds
  const [isPlaying, setIsPlaying] = useState(false);
  const [globalSpeed, setGlobalSpeed] = useState(10); // in cm/s
  const [delayAfterPullCordS, setDelayAfterPullCordS] = useState(3); // in seconds
  const [isSaving, setIsSaving] = useState(false);
  const tableCanvasRef = useRef(null);
  const [tableControlsState, setTableControlsState] = useState({
    canUndo: false,
    canRedo: false,
    canClear: false,
    isEditLocked: false,
  });
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 });

  useEffect(() => {
    fetch(`${API_URL}/config`)
      .then(res => res.json())
      .then(data => {
        if (data.trajectories) setTrajectories(data.trajectories);
        if (data.globalSpeed !== undefined) setGlobalSpeed(data.globalSpeed);
        if (data.delayAfterPullCordS !== undefined) setDelayAfterPullCordS(data.delayAfterPullCordS);
      })
      .catch(err => console.error("Could not load backend config", err));
  }, []);

  const handleSaveConfig = () => {
    setIsSaving(true);
    fetch(`${API_URL}/config`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ trajectories, globalSpeed, delayAfterPullCordS })
    })
    .then(res => res.json())
    .then(() => {
        alert("Configuration sauvegardée avec succès !");
        setIsSaving(false);
    })
    .catch(err => {
        alert("Erreur de sauvegarde");
        setIsSaving(false);
        console.error(err);
    });
  };

  const handleCompileFlash = (pamiId) => {
    alert(`Lancement de la compilation pour le PAMI ${pamiId}...\nMerci de patienter (ne fermez pas la page).`);
    fetch(`${API_URL}/flash/${pamiId}`, { method: "POST" })
      .then(async res => {
          const data = await res.json();
          if (!res.ok) throw new Error(data.detail || "Erreur de flash");
          return data;
      })
      .then(() => alert(`PAMI ${pamiId} flashé avec succès !`))
      .catch(err => {
          console.error(err);
          alert(`Erreur:\n${err.message}`);
      });
  };

  const maxGlobalDistanceMm = React.useMemo(() => {
    let maxD = 0;
    Object.values(trajectories).forEach(pts => {
      maxD = Math.max(maxD, calculateDistance(pts));
    });
    return maxD;
  }, [trajectories]);

  const speedMmPerS = globalSpeed * 10;
  const movementTime = speedMmPerS > 0 ? maxGlobalDistanceMm / speedMmPerS : 0;
  const maxTime = movementTime > 0 ? delayAfterPullCordS + movementTime : 0;

  const handleUpdateTrajectory = (pamiId, waypoints) => {
    setTrajectories(prev => ({...prev, [pamiId]: waypoints}));
  };

  const currentMovementCount = trajectories[selectedPami]?.length || 0;

  return (
    <div className="App" style={{ margin: '0 auto', padding: '20px' }}>
      <div style={{ display: 'flex', flexDirection: 'column', gap: '20px' }}>
        <div style={{ width: '100%', display: 'flex', gap: '16px', alignItems: 'flex-start' }}>
          <div style={{ width: '15%', minWidth: '130px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
            <div>
              <strong>Edition de la trajectoire pour le PAMI {selectedPami}</strong>
              {tableControlsState.isEditLocked && (
                <div style={{ color: 'red', fontSize: '0.9em' }}>
                  (Mode Visualisation - Edition bloquee)
                </div>
              )}
            </div>
            <button
              onClick={() => tableCanvasRef.current?.undo()}
              disabled={!tableControlsState.canUndo || tableControlsState.isEditLocked}
            >
              Undo
            </button>
            <button
              onClick={() => tableCanvasRef.current?.redo()}
              disabled={!tableControlsState.canRedo || tableControlsState.isEditLocked}
            >
              Redo
            </button>
            <button
              onClick={() => tableCanvasRef.current?.clear()}
              disabled={!tableControlsState.canClear || tableControlsState.isEditLocked}
            >
              Clear Trajectory
            </button>
            <div style={{ marginTop: '4px' }}>
              <label style={{ display: 'block', marginBottom: '6px' }}><strong>PAMI :</strong></label>
              <select
                value={selectedPami}
                onChange={(e) => setSelectedPami(Number(e.target.value))}
                style={{ width: '100%', padding: '6px', fontSize: '14px' }}
              >
                {[1, 2, 3, 4, 5, 6].map(i => (
                  <option key={i} value={i}>N° {i}</option>
                ))}
              </select>
            </div>
            <div style={{ marginTop: '4px' }}>
              <label style={{ display: 'block', marginBottom: '6px' }}><strong>Vitesse globale :</strong></label>
              <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <input
                  type="number"
                  value={globalSpeed}
                  onChange={(e) => setGlobalSpeed(Number(e.target.value))}
                  min="1"
                  style={{ width: '100%', padding: '6px', fontSize: '14px' }}
                />
                <span style={{ fontSize: '12px' }}>cm/s</span>
              </div>
            </div>
            <div style={{ marginTop: '4px' }}>
              <label style={{ display: 'block', marginBottom: '6px' }}><strong>Délai après tirette :</strong></label>
              <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <input
                  type="number"
                  value={delayAfterPullCordS}
                  onChange={(e) => setDelayAfterPullCordS(Number(e.target.value))}
                  min="0"
                  step="0.1"
                  style={{ width: '100%', padding: '6px', fontSize: '14px' }}
                />
                <span style={{ fontSize: '12px' }}>s</span>
              </div>
            </div>
            <div style={{ marginTop: '6px', fontFamily: 'monospace', fontSize: '13px', backgroundColor: '#eef', padding: '6px 8px', borderRadius: '5px', border: '1px solid #ccd' }}>
              <strong>X:</strong> {mousePos.x} mm<br />
              <strong>Y:</strong> {mousePos.y} mm
            </div>
            <div style={{ marginTop: '4px', fontFamily: 'monospace', fontSize: '13px' }}>
              Mouvements PAMI {selectedPami} : {currentMovementCount}
            </div>
          </div>

          <div style={{ width: '85%' }}>
            <TableCanvas 
              ref={tableCanvasRef}
              selectedPami={selectedPami}
              waypoints={trajectories[selectedPami]}
              setWaypoints={(wps) => handleUpdateTrajectory(selectedPami, wps)}
              allTrajectories={trajectories}
              elapsedTime={elapsedTime}
              isPlaying={isPlaying}
              speedMmPerS={speedMmPerS}
              delayAfterPullCordS={delayAfterPullCordS}
              onControlsStateChange={setTableControlsState}
              onMousePositionChange={setMousePos}
            />
          </div>
        </div>

        <div style={{ width: '85%', marginLeft: 'calc(15% + 16px)' }}>
          <Timeline 
            elapsedTime={elapsedTime} 
            setElapsedTime={setElapsedTime} 
            maxTime={maxTime}
            isPlaying={isPlaying} 
            setIsPlaying={setIsPlaying}
          />
        </div>
        <div style={{ width: '100%' }}>
          <ConfigPanel 
            selectedPami={selectedPami} 
            onSave={handleSaveConfig}
            onCompileFlash={() => handleCompileFlash(selectedPami)}
            isSaving={isSaving}
          />
        </div>
      </div>
    </div>
  );
}

export default App;
