import React from 'react';

const ConfigPanel = ({
    selectedPami,
    setSelectedPami,
    globalSpeed,
    setGlobalSpeed,
    startAfterDelayS,
    setStartAfterDelayS,
    onSave,
    onCompileFlash,
    isSaving,
}) => {
    return (
        <div style={{ padding: '20px', border: '1px solid #ccc', minWidth: '300px', backgroundColor: '#fafafa' }}>
            <h2 style={{marginTop: '0'}}>Configuration</h2>
            
            <div style={{ marginBottom: '20px', padding: '15px', backgroundColor: '#fff', border: '1px solid #eee', borderRadius: '5px' }}>
                <label style={{ marginRight: '10px' }}><strong>Vitesse globale :</strong></label>
                <input 
                    type="number" 
                    value={globalSpeed} 
                    onChange={(e) => setGlobalSpeed(Number(e.target.value))}
                    min="1"
                    style={{ width: '80px', padding: '5px', fontSize: '16px' }}
                /> <span style={{ marginLeft: '5px' }}>cm/s</span>
            </div>

            <div style={{ marginBottom: '20px', padding: '15px', backgroundColor: '#fff', border: '1px solid #eee', borderRadius: '5px' }}>
                <label style={{ marginRight: '10px' }}><strong>Delai avant depart :</strong></label>
                <input
                    type="number"
                    value={startAfterDelayS}
                    onChange={(e) => setStartAfterDelayS(Number(e.target.value))}
                    min="0"
                    step="0.1"
                    style={{ width: '80px', padding: '5px', fontSize: '16px' }}
                /> <span style={{ marginLeft: '5px' }}>s</span>
            </div>
            
            <div style={{ marginBottom: '30px' }}>
                <button 
                    onClick={onSave} 
                    disabled={isSaving}
                    style={{ width: '100%', padding: '10px', fontSize: '16px', backgroundColor: '#28a745', color: '#fff', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                >
                    {isSaving ? 'Sauvegarde...' : '💾 Sauvegarder Projet'}
                </button>
            </div>

            <div style={{ paddingTop: '20px', borderTop: '2px dashed #ddd' }}>
                 <h3>Export & Flash PAMI</h3>
                
                <div style={{ marginBottom: '15px' }}>
                    <label style={{ marginRight: '10px' }}><strong>Brancher le PAMI :</strong></label>
                    <select 
                        value={selectedPami} 
                        onChange={(e) => setSelectedPami(Number(e.target.value))}
                        style={{ fontSize: '16px', padding: '5px', width: '100px' }}
                    >
                        {[1, 2, 3, 4, 5, 6].map(i => (
                            <option key={i} value={i}>N° {i}</option>
                        ))}
                    </select>
                </div>

                <div style={{ color: '#555', fontSize: '0.9em', marginBottom: '15px' }}>
                    <i>Connectez le PAMI <b>{selectedPami}</b> en USB. Le script génèrera le code C++ avec sa trajectoire spécifique puis lancera la compilation PlatformIO.</i>
                </div>

                <button 
                    onClick={onCompileFlash}
                    style={{ width: '100%', padding: '15px', fontSize: '16px', fontWeight: 'bold', backgroundColor: '#007bff', color: '#fff', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                >
                    🚀 Compiler & Flasher PAMI {selectedPami}
                </button>
            </div>
        </div>
    );
};

export default ConfigPanel;
