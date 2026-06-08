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

const calculateWaitTime = (pts) => {
  if (!pts) return 0;
  return pts.reduce((sum, pt) => sum + Math.max(0, Number(pt.waitS || 0)), 0);
};

const normalizeAngle = (a) => {
  let angle = a;
  while (angle > Math.PI) angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
};

const calculateTrajectoryDuration = (pts, speedMmPerS, angularSpeedRadS, isHolonomic = false) => {
  if (!pts || pts.length === 0) return 0;

  const waitTime = calculateWaitTime(pts);
  const moveTime = speedMmPerS > 0 ? calculateDistance(pts) / speedMmPerS : 0;
  if (isHolonomic || pts.length < 2 || angularSpeedRadS <= 0) {
    return waitTime + moveTime;
  }

  let turnTime = 0;
  let currentHeading = Math.atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
  for (let i = 1; i < pts.length - 1; i++) {
    const nextHeading = Math.atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
    const delta = Math.abs(normalizeAngle(nextHeading - currentHeading));
    if (delta >= 0.02) {
      turnTime += delta / angularSpeedRadS;
    }
    currentHeading = nextHeading;
  }

  return waitTime + moveTime + turnTime;
};

function App() {
  const [selectedPami, setSelectedPami] = useState('1');
  const [trajectories, setTrajectories] = useState({
    '1': [], '2': [], '3': [], '4': [], '5': [], '6': [], bigRobot: []
  });
  const [elapsedTime, setElapsedTime] = useState(0); // in seconds
  const [isPlaying, setIsPlaying] = useState(false);
  const [globalSpeed, setGlobalSpeed] = useState(10); // in cm/s
  const [angularSpeedDegS, setAngularSpeedDegS] = useState(70); // in deg/s
  const [delayAfterPullCordS, setDelayAfterPullCordS] = useState(3); // in seconds
  const [isSaving, setIsSaving] = useState(false);
  const [isFlashing, setIsFlashing] = useState(false);
  const [logs, setLogs] = useState([]);
  const tableCanvasRef = useRef(null);
  const logsContainerRef = useRef(null);
  const [tableControlsState, setTableControlsState] = useState({
    canUndo: false,
    canRedo: false,
    canClear: false,
    isEditLocked: false,
  });
  const [mousePos, setMousePos] = useState({ x: 0, y: 0 });
  const [selectedWaypointIndex, setSelectedWaypointIndex] = useState(null);

  const addLog = (type, text) => {
    const time = new Date().toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    setLogs(prev => [...prev, { type, text, time }]);
  };

  useEffect(() => {
    if (logsContainerRef.current) {
      logsContainerRef.current.scrollTop = logsContainerRef.current.scrollHeight;
    }
  }, [logs]);

  useEffect(() => {
    fetch(`${API_URL}/config`)
      .then(res => res.json())
      .then(data => {
        if (data.trajectories) {
          setTrajectories({
            '1': [], '2': [], '3': [], '4': [], '5': [], '6': [], bigRobot: [],
            ...data.trajectories,
          });
        }
        if (data.globalSpeed !== undefined) setGlobalSpeed(data.globalSpeed);
        if (data.angularSpeedDegS !== undefined) setAngularSpeedDegS(data.angularSpeedDegS);
        if (data.delayAfterPullCordS !== undefined) setDelayAfterPullCordS(data.delayAfterPullCordS);
      })
      .catch(err => console.error("Could not load backend config", err));
  }, []);

  const handleSaveConfig = () => {
    setIsSaving(true);
    addLog('info', 'Sauvegarde de la configuration...');
    fetch(`${API_URL}/config`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ trajectories, globalSpeed, angularSpeedDegS, delayAfterPullCordS })
    })
    .then(res => res.json())
    .then(() => {
        addLog('success', 'Configuration sauvegardée avec succès !');
        setIsSaving(false);
    })
    .catch(err => {
        addLog('error', `Erreur de sauvegarde: ${err.message}`);
        setIsSaving(false);
    });
  };

  const handleCompileFlash = (pamiId) => {
    setIsFlashing(true);
    addLog('info', `[PAMI ${pamiId}] Génération du code C++...`);
    addLog('info', `[PAMI ${pamiId}] Lancement de la compilation PlatformIO (patience)...`);
    fetch(`${API_URL}/flash/${pamiId}`, { method: "POST" })
      .then(async res => {
          const data = await res.json();
          if (!res.ok) throw new Error(data.detail || "Erreur de flash");
          return data;
      })
      .then(data => {
          addLog('success', `[PAMI ${pamiId}] Flashé avec succès !`);
          if (data.logs) {
            data.logs.split('\n').filter(l => l.trim()).forEach(line => addLog('info', line));
          }
          setIsFlashing(false);
      })
      .catch(err => {
          err.message.split('\n').filter(l => l.trim()).forEach(line => addLog('error', line));
          setIsFlashing(false);
      });
  };

  const speedMmPerS = globalSpeed * 10;
  const angularSpeedRadS = (angularSpeedDegS * Math.PI) / 180;
  const maxTrajectoryDuration = React.useMemo(() => {
    let maxDuration = 0;
    Object.entries(trajectories).forEach(([trajId, pts]) => {
      maxDuration = Math.max(
        maxDuration,
        calculateTrajectoryDuration(pts, speedMmPerS, angularSpeedRadS, trajId === 'bigRobot')
      );
    });
    return maxDuration;
  }, [trajectories, speedMmPerS, angularSpeedRadS]);
  const maxTime = maxTrajectoryDuration > 0 ? delayAfterPullCordS + maxTrajectoryDuration : 0;

  const handleUpdateTrajectory = (pamiId, waypoints) => {
    setTrajectories(prev => ({...prev, [pamiId]: waypoints}));
  };

  const isBigRobotSelected = selectedPami === 'bigRobot';
  const selectedLabel = isBigRobotSelected ? 'Gros robot' : `PAMI ${selectedPami}`;

  const currentMovementCount = trajectories[selectedPami]?.length || 0;
  const selectedWaypoints = trajectories[selectedPami] || [];
  const selectedWaypoint = selectedWaypointIndex !== null ? selectedWaypoints[selectedWaypointIndex] : null;

  useEffect(() => {
    setSelectedWaypointIndex(null);
  }, [selectedPami]);

  useEffect(() => {
    if (selectedWaypointIndex === null) return;
    if (selectedWaypointIndex >= selectedWaypoints.length) {
      setSelectedWaypointIndex(selectedWaypoints.length > 0 ? selectedWaypoints.length - 1 : null);
    }
  }, [selectedWaypointIndex, selectedWaypoints]);

  const updateSelectedWaypointWait = (nextWaitS) => {
    if (selectedWaypointIndex === null) return;
    const safeWaitS = Math.max(0, Number.isFinite(nextWaitS) ? nextWaitS : 0);
    setTrajectories((prev) => {
      const current = [...(prev[selectedPami] || [])];
      if (!current[selectedWaypointIndex]) return prev;
      current[selectedWaypointIndex] = {
        ...current[selectedWaypointIndex],
        waitS: safeWaitS,
      };
      return { ...prev, [selectedPami]: current };
    });
  };

  const removeSelectedWaypoint = () => {
    if (selectedWaypointIndex === null || tableControlsState.isEditLocked) return;
    tableCanvasRef.current?.deleteSelectedWaypoint(selectedWaypointIndex);
  };

  useEffect(() => {
    const onKeyDown = (event) => {
      if (!(event.key === 'Delete' || event.key === 'Del')) return;
      const tag = (event.target?.tagName || '').toLowerCase();
      if (tag === 'input' || tag === 'textarea' || tag === 'select') return;
      if (selectedWaypointIndex === null || tableControlsState.isEditLocked) return;

      event.preventDefault();
      removeSelectedWaypoint();
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [selectedWaypointIndex, tableControlsState.isEditLocked]);

  useEffect(() => {
    const onKeyDown = (event) => {
      const tag = (event.target?.tagName || '').toLowerCase();
      if (tag === 'input' || tag === 'textarea' || tag === 'select') return;

      const isCtrlOrMeta = event.ctrlKey || event.metaKey;
      if (!isCtrlOrMeta) return;

      const key = event.key.toLowerCase();
      const isUndo = key === 'z' && !event.shiftKey;
      const isRedo = key === 'y' || (key === 'z' && event.shiftKey);

      if (isUndo) {
        if (tableControlsState.isEditLocked || !tableControlsState.canUndo) return;
        event.preventDefault();
        tableCanvasRef.current?.undo();
        return;
      }

      if (isRedo) {
        if (tableControlsState.isEditLocked || !tableControlsState.canRedo) return;
        event.preventDefault();
        tableCanvasRef.current?.redo();
      }
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [tableControlsState.isEditLocked, tableControlsState.canUndo, tableControlsState.canRedo]);

  return (
    <div className="App" style={{ margin: '0 auto', padding: '20px' }}>
      <div style={{ display: 'flex', flexDirection: 'column', gap: '20px' }}>
        <div style={{ width: '100%', display: 'flex', gap: '16px', alignItems: 'flex-start' }}>
          <div style={{ width: '15%', minWidth: '130px', display: 'flex', flexDirection: 'column', gap: '10px' }}>
            <div>
              <strong>Edition de la trajectoire pour {selectedLabel}</strong>
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
                onChange={(e) => setSelectedPami(e.target.value)}
                style={{ width: '100%', padding: '6px', fontSize: '14px' }}
              >
                {[1, 2, 3, 4, 5, 6].map(i => (
                  <option key={i} value={String(i)}>N° {i}</option>
                ))}
                <option value="bigRobot">Gros robot</option>
              </select>
            </div>
            <div style={{ marginTop: '4px' }}>
              <label style={{ display: 'block', marginBottom: '6px' }}><strong>Vitesse linéaire globale :</strong></label>
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
              <label style={{ display: 'block', marginBottom: '6px' }}><strong>Vitesse angulaire globale :</strong></label>
              <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                <input
                  type="number"
                  value={angularSpeedDegS}
                  onChange={(e) => setAngularSpeedDegS(Number(e.target.value))}
                  min="1"
                  step="1"
                  style={{ width: '100%', padding: '6px', fontSize: '14px' }}
                />
                <span style={{ fontSize: '12px' }}>deg/s</span>
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
              Mouvements {selectedLabel} : {currentMovementCount}
            </div>
            <div style={{ marginTop: '8px', padding: '8px', border: '1px solid #ccd', borderRadius: '6px', backgroundColor: '#f7f9ff' }}>
              <div style={{ marginBottom: '6px' }}><strong>Point sélectionné</strong></div>
              {selectedWaypoint ? (
                <>
                  <div style={{ fontSize: '12px', marginBottom: '6px' }}>
                    Point #{selectedWaypointIndex + 1} ({Math.round(selectedWaypoint.x)}, {Math.round(selectedWaypoint.y)})
                  </div>
                  <label style={{ display: 'block', marginBottom: '6px' }}><strong>Attente (s) :</strong></label>
                  <input
                    type="number"
                    min="0"
                    step="0.1"
                    value={Number(selectedWaypoint.waitS || 0)}
                    onChange={(e) => updateSelectedWaypointWait(Number(e.target.value))}
                    style={{ width: '100%', padding: '6px', fontSize: '14px', marginBottom: '6px' }}
                  />
                  <div style={{ display: 'flex', gap: '6px' }}>
                    <button onClick={() => updateSelectedWaypointWait((selectedWaypoint.waitS || 0) + 0.5)}>+0.5s</button>
                    <button onClick={() => updateSelectedWaypointWait((selectedWaypoint.waitS || 0) + 1)}>+1s</button>
                    <button onClick={() => updateSelectedWaypointWait((selectedWaypoint.waitS || 0) + 2)}>+2s</button>
                    <button onClick={() => updateSelectedWaypointWait(0)}>=0s</button>
                  </div>
                  <div style={{ marginTop: '6px' }}>
                    <button
                      onClick={removeSelectedWaypoint}
                      disabled={tableControlsState.isEditLocked}
                      style={{ padding: '3px 8px', fontSize: '12px' }}
                    >
                      Supprimer ce point
                    </button>
                  </div>
                </>
              ) : (
                <div style={{ fontSize: '12px' }}>Clique sur un point de trajectoire pour régler son attente.</div>
              )}
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
              angularSpeedDegS={angularSpeedDegS}
              delayAfterPullCordS={delayAfterPullCordS}
              selectedWaypointIndex={selectedWaypointIndex}
              onSelectWaypoint={setSelectedWaypointIndex}
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
            onCompileFlash={() => handleCompileFlash(Number(selectedPami))}
            canCompile={!isBigRobotSelected}
            selectedLabel={selectedLabel}
            isSaving={isSaving}
            isFlashing={isFlashing}
          />
        </div>
        <div style={{ width: '100%' }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '6px' }}>
            <strong style={{ fontSize: '14px' }}>Logs</strong>
            <button onClick={() => setLogs([])} style={{ fontSize: '12px', padding: '2px 8px' }}>Effacer</button>
          </div>
          <div
            ref={logsContainerRef}
            style={{
              backgroundColor: '#1e1e1e',
              color: '#ccc',
              fontFamily: 'monospace',
              fontSize: '12px',
              padding: '10px',
              borderRadius: '4px',
              height: '200px',
              overflowY: 'auto',
              border: '1px solid #444',
            }}
          >
            {logs.length === 0 ? (
              <span style={{ color: '#666' }}>Aucun log pour le moment...</span>
            ) : (
              logs.map((log, i) => (
                <div key={i} style={{ color: log.type === 'success' ? '#4caf50' : log.type === 'error' ? '#f44336' : '#ccc', marginBottom: '2px' }}>
                  <span style={{ color: '#888', marginRight: '8px' }}>[{log.time}]</span>
                  {log.text}
                </div>
              ))
            )}
          </div>
        </div>
      </div>
    </div>
  );
}

export default App;
