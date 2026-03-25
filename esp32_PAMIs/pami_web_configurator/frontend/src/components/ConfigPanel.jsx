import React from 'react';

const ConfigPanel = ({ selectedPami, setSelectedPami, globalSpeed, setGlobalSpeed }) => {
    return (
        <div style={{ padding: '20px', border: '1px solid #ccc', minWidth: '300px' }}>
            <h2>Configuration</h2>
            
            <div style={{ marginBottom: '20px' }}>
                <label style={{ marginRight: '10px' }}><strong>Sélection du PAMI :</strong></label>
                <select 
                    value={selectedPami} 
                    onChange={(e) => setSelectedPami(Number(e.target.value))}
                    style={{ fontSize: '16px', padding: '5px' }}
                >
                    {[1, 2, 3, 4, 5, 6].map(i => (
                        <option key={i} value={i}>PAMI {i}</option>
                    ))}
                </select>
            </div>

            <div style={{ marginBottom: '20px' }}>
                <label style={{ marginRight: '10px' }}><strong>Vitesse globale (cm/s) :</strong></label>
                <input 
                    type="number" 
                    value={globalSpeed} 
                    onChange={(e) => setGlobalSpeed(Number(e.target.value))}
                    min="1"
                    style={{ width: '80px', padding: '5px', fontSize: '16px' }}
                />
            </div>

            <p>Paramètres du <strong>PAMI {selectedPami}</strong> (compilation/flash) à venir...</p>
        </div>
    );
};

export default ConfigPanel;
