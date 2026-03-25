import React, { useEffect } from 'react';

const Timeline = ({ elapsedTime, setElapsedTime, maxTime, isPlaying, setIsPlaying }) => {
    
    useEffect(() => {
        let interval;
        if (isPlaying) {
            let lastTime = Date.now();
            interval = setInterval(() => {
                const now = Date.now();
                const dt = (now - lastTime) / 1000; // temps écoulé en secondes
                lastTime = now;

                setElapsedTime(prev => {
                    const nextTime = prev + dt;
                    if (nextTime >= maxTime && maxTime > 0) {
                        setIsPlaying(false);
                        return maxTime;
                    }
                    return nextTime;
                });
            }, 30); // Rafraichissement toutes les 30ms pour une anim fluide
        } else if (!isPlaying && elapsedTime !== 0) {
            clearInterval(interval);
        }
        return () => clearInterval(interval);
    }, [isPlaying, maxTime, elapsedTime, setElapsedTime, setIsPlaying]);

    const handlePlayPause = () => {
        if (elapsedTime >= maxTime && !isPlaying && maxTime > 0) {
            setElapsedTime(0);
        }
        setIsPlaying(!isPlaying);
    };

    const handleStop = () => {
        setIsPlaying(false);
        setElapsedTime(0);
    };

    return (
        <div style={{ marginTop: '20px', padding: '15px', border: '1px solid #ccc', borderRadius: '5px', backgroundColor: '#f9f9f9' }}>
            <h3 style={{ marginTop: 0 }}>Visualisation & Multijoueur</h3>
            <div style={{ display: 'flex', alignItems: 'center', gap: '15px' }}>
                <button onClick={handlePlayPause} style={{ padding: '8px 15px', cursor: 'pointer', minWidth: '80px' }}>
                    {isPlaying ? '⏸ Pause' : '▶ Play'}
                </button>
                <button onClick={handleStop} style={{ padding: '8px 15px', cursor: 'pointer' }}>
                    ⏹ Stop
                </button>
                <input 
                    type="range" 
                    min="0" 
                    max={maxTime || 100} 
                    step="0.01" 
                    value={elapsedTime} 
                    onChange={(e) => {
                        setElapsedTime(Number(e.target.value));
                        setIsPlaying(false); // Pause auto si on déplace le slider
                    }}
                    style={{ flexGrow: 1 }}
                />
                <span style={{ minWidth: '100px', fontFamily: 'monospace', fontSize: '16px' }}>
                    {elapsedTime.toFixed(2)}s {maxTime > 0 ? `/ ${maxTime.toFixed(2)}s` : ''}
                </span>
            </div>
        </div>
    );
};

export default Timeline;
