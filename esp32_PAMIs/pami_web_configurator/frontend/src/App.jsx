import React, { useState, useEffect } from 'react';
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
  const [isSaving, setIsSaving] = useState(false);

  useEffect(() => {
    fetch(`${API_URL}/config`)
      .then(res => res.json())
      .then(data => {
        if (data.trajectories) setTrajectories(data.trajectories);
        if (data.globalSpeed) setGlobalSpeed(data.globalSpeed);
      })
      .catch(err => console.error("Could not load backend config", err));
  }, []);

  const handleSaveConfig = () => {
    setIsSaving(true);
    fetch(`${API_URL}/config`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ trajectories, globalSpeed })
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
  const maxTime = speedMmPerS > 0 ? maxGlobalDistanceMm / speedMmPerS : 0;

  const handleUpdateTrajectory = (pamiId, waypoints) => {
    setTrajectories(prev => ({...prev, [pamiId]: waypoints}));
  };

  return (
    <div className="App">
      <h1>PAMI Web Configurator</h1>
      <div style={{ display: 'flex', gap: '20px' }}>
        <div>
          <TableCanvas 
            selectedPami={selectedPami}
            waypoints={trajectories[selectedPami]}
            setWaypoints={(wps) => handleUpdateTrajectory(selectedPami, wps)}
            allTrajectories={trajectories}
            elapsedTime={elapsedTime}
            isPlaying={isPlaying}
            speedMmPerS={speedMmPerS}
          />
          <Timeline 
            elapsedTime={elapsedTime} 
            setElapsedTime={setElapsedTime} 
            maxTime={maxTime}
            isPlaying={isPlaying} 
            setIsPlaying={setIsPlaying}
          />
        </div>
        <ConfigPanel 
          selectedPami={selectedPami} 
          setSelectedPami={setSelectedPami}
          globalSpeed={globalSpeed}
          setGlobalSpeed={setGlobalSpeed}
          onSave={handleSaveConfig}
          onCompileFlash={() => handleCompileFlash(selectedPami)}
          isSaving={isSaving}
        />
      </div>
    </div>
  );
}

export default App;
