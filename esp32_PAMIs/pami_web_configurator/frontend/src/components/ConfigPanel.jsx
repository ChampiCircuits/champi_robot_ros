import React from 'react';

const ConfigPanel = ({
    selectedPami,
    selectedLabel,
    onSave,
    onCompileFlash,
    canCompile,
    isSaving,
    isFlashing,
}) => {
    return (
        <div style={{ padding: '20px', border: '1px solid #ccc', width: '100%', boxSizing: 'border-box', backgroundColor: '#fafafa' }}>
            <h2 style={{marginTop: '0'}}>Configuration</h2>
            
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

                <div style={{ color: '#555', fontSize: '0.9em', marginBottom: '15px' }}>
                    {canCompile ? (
                        <i>Connectez le PAMI <b>{selectedPami}</b> en USB. Le script génèrera le code C++ avec sa trajectoire spécifique puis lancera la compilation PlatformIO.</i>
                    ) : (
                        <i>Le gros robot est uniquement utilise pour simulation/collision. Aucune generation CPP n'est lancee pour cette trajectoire.</i>
                    )}
                </div>

                <button 
                    onClick={onCompileFlash}
                    disabled={!canCompile || isFlashing}
                    style={{ width: '100%', padding: '15px', fontSize: '16px', fontWeight: 'bold', backgroundColor: canCompile && !isFlashing ? '#007bff' : '#8aa0bf', color: '#fff', border: 'none', borderRadius: '4px', cursor: canCompile && !isFlashing ? 'pointer' : 'not-allowed' }}
                >
                    {!canCompile ? '🚫 Pas de flash pour le gros robot' : isFlashing ? '⏳ Compilation en cours...' : `🚀 Compiler & Flasher ${selectedLabel}`}
                </button>
            </div>
        </div>
    );
};

export default ConfigPanel;
