import React, { useState } from 'react';
import TableCanvas from './components/TableCanvas.jsx';
import Timeline from './components/Timeline.jsx';
import ConfigPanel from './components/ConfigPanel.jsx';

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
        />
      </div>
    </div>
  );
}

export default App;
