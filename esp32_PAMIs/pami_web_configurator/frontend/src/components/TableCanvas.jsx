import React, { useRef, useEffect, useState, forwardRef, useImperativeHandle } from 'react';

const PAMI_COLORS = {
    1: '#FF0000', // Rouge
    2: '#0000FF', // Bleu
    3: '#008000', // Vert
    4: '#FFA500', // Orange
    5: '#800080', // Violet
    6: '#FF00FF'  // Magenta
};

const PAMI_LENGTH_MM = 140;
const PAMI_WIDTH_MM = 102;

const TABLE_WIDTH_MM = 3000;
const TABLE_HEIGHT_MM = 2000;
const CANVAS_WIDTH = 900;
const CANVAS_HEIGHT = 600;
const GRID_SPACING_MM = 50;
const GRID_OFFSET_X_MM = 25;

// Fonctions de conversions Pixels <-> Millimètres avec repère en bas à gauche
const mmToPxX = (x_mm) => (x_mm / TABLE_WIDTH_MM) * CANVAS_WIDTH;
const mmToPxY = (y_mm) => CANVAS_HEIGHT - ((y_mm / TABLE_HEIGHT_MM) * CANVAS_HEIGHT); // Inversion Y
const pxToMmX = (x_px) => Math.round((x_px / CANVAS_WIDTH) * TABLE_WIDTH_MM);
const pxToMmY = (y_px) => Math.round(((CANVAS_HEIGHT - y_px) / CANVAS_HEIGHT) * TABLE_HEIGHT_MM);
const snapToGridY = (valueMm) => Math.round(valueMm / GRID_SPACING_MM) * GRID_SPACING_MM;
const snapToGridX = (valueMm) => Math.round((valueMm - GRID_OFFSET_X_MM) / GRID_SPACING_MM) * GRID_SPACING_MM + GRID_OFFSET_X_MM;
const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const normalizeHeadingDeg = (deg) => {
    let d = Number.isFinite(deg) ? deg : 0;
    d = ((d % 360) + 360) % 360;
    return d;
};
const degToRad = (deg) => (deg * Math.PI) / 180;

// Helpers pour la détection de collision via le théorème des axes séparateurs (SAT)
const getOBBCorners = (x, y, w, h, angle) => {
    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);
    const hw = w / 2, hh = h / 2;
    // Les 4 coins d'un rectangle centré en 0,0 avant rotation
    return [
        { lx: -hw, ly: -hh },
        { lx: hw, ly: -hh },
        { lx: hw, ly: hh },
        { lx: -hw, ly: hh }
    ].map(c => ({
        x: x + c.lx * cosA - c.ly * sinA,
        y: y + c.lx * sinA + c.ly * cosA
    }));
};

const projectOBB = (corners, axis) => {
    let min = corners[0].x * axis.x + corners[0].y * axis.y;
    let max = min;
    for (let i = 1; i < corners.length; i++) {
        const projected = corners[i].x * axis.x + corners[i].y * axis.y;
        if (projected < min) min = projected;
        if (projected > max) max = projected;
    }
    return { min, max };
};

const checkOBBCollision = (rect1, rect2) => {
    const c1 = getOBBCorners(rect1.x, rect1.y, rect1.w, rect1.h, rect1.angle);
    const c2 = getOBBCorners(rect2.x, rect2.y, rect2.w, rect2.h, rect2.angle);

    // Récupérer les axes normaux à explorer (les arrêtes de chaque rectangle)
    const axes = [
        { x: c1[1].x - c1[0].x, y: c1[1].y - c1[0].y },
        { x: c1[2].x - c1[1].x, y: c1[2].y - c1[1].y },
        { x: c2[1].x - c2[0].x, y: c2[1].y - c2[0].y },
        { x: c2[2].x - c2[1].x, y: c2[2].y - c2[1].y }
    ];

    for (let axis of axes) {
        const len = Math.hypot(axis.x, axis.y);
        if (len === 0) continue;
        const normAxis = { x: axis.x / len, y: axis.y / len };
        
        const p1 = projectOBB(c1, normAxis);
        const p2 = projectOBB(c2, normAxis);

        // Si intervalles de projection séparés -> pas de collision
        if (p1.max < p2.min || p2.max < p1.min) return false;
    }
    return true; // Tous les axes se superposent -> on touche !
};

// Helper : trouve la position exacte à une certaine distance parcourue et calcule l'angle (orientation)
const getPositionAtDistance = (pts, targetDist) => {
    if (!pts || pts.length === 0) return null;
    if (pts.length === 1) return { ...pts[0], angle: 0 };
    
    let currentD = 0;
    for (let i = 1; i < pts.length; i++) {
        const dx = pts[i].x - pts[i - 1].x;
        const dy = pts[i].y - pts[i - 1].y;
        const segDist = Math.hypot(dx, dy);
        
        if (currentD + segDist >= targetDist) {
            const ratio = (targetDist - currentD) / segDist;
            return {
                x: pts[i - 1].x + dx * ratio,
                y: pts[i - 1].y + dy * ratio,
                angle: Math.atan2(dy, dx)
            };
        }
        currentD += segDist;
    }
    // Si on dépasse, on reste sur le dernier point et on garde l'angle du dernier segment
    const dx = pts[pts.length - 1].x - pts[pts.length - 2].x;
    const dy = pts[pts.length - 1].y - pts[pts.length - 2].y;
    return { ...pts[pts.length - 1], angle: Math.atan2(dy, dx) };
};

const normalizeAngle = (angle) => {
    let a = angle;
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
};

const getPoseAtTimeWithWaitsAndTurns = (pts, elapsedS, speedMmPerS, angularSpeedRadS) => {
    if (!pts || pts.length === 0) return null;
    if (pts.length === 1) {
        const startHeadingDeg = normalizeHeadingDeg(Number(pts[0].headingDeg || 0));
        return { x: pts[0].x, y: pts[0].y, angle: degToRad(startHeadingDeg) };
    }

    let t = Math.max(0, elapsedS);
    const firstSegHeading = Math.atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
    const hasStartHeading = Number.isFinite(Number(pts[0].headingDeg));
    let currentHeading = hasStartHeading ? degToRad(normalizeHeadingDeg(Number(pts[0].headingDeg))) : firstSegHeading;
    const minTurnRad = 0.02;

    const waitAtPoint0 = Math.max(0, Number(pts[0].waitS || 0));
    if (t <= waitAtPoint0) {
        return { x: pts[0].x, y: pts[0].y, angle: currentHeading };
    }
    t -= waitAtPoint0;

    for (let i = 0; i < pts.length - 1; i++) {
        const from = pts[i];
        const to = pts[i + 1];

        const targetHeading = Math.atan2(to.y - from.y, to.x - from.x);
        const deltaHeading = normalizeAngle(targetHeading - currentHeading);
        const absDelta = Math.abs(deltaHeading);

        if (absDelta >= minTurnRad && angularSpeedRadS > 0) {
            const turnTime = absDelta / angularSpeedRadS;
            if (t <= turnTime) {
                const sign = deltaHeading >= 0 ? 1 : -1;
                const partialAngle = currentHeading + sign * angularSpeedRadS * t;
                return { x: from.x, y: from.y, angle: partialAngle };
            }
            t -= turnTime;
        }

        currentHeading = targetHeading;

        const dx = to.x - from.x;
        const dy = to.y - from.y;
        const segDist = Math.hypot(dx, dy);
        const segTime = speedMmPerS > 0 ? segDist / speedMmPerS : Number.POSITIVE_INFINITY;

        if (t <= segTime) {
            const ratio = segTime > 0 && Number.isFinite(segTime) ? t / segTime : 0;
            return {
                x: from.x + dx * ratio,
                y: from.y + dy * ratio,
                angle: currentHeading,
            };
        }

        t -= segTime;

        const waitAtPoint = Math.max(0, Number(to.waitS || 0));
        if (t <= waitAtPoint) {
            return { x: to.x, y: to.y, angle: currentHeading };
        }
        t -= waitAtPoint;
    }

    const n = pts.length;
    return { x: pts[n - 1].x, y: pts[n - 1].y, angle: currentHeading };
};

const TableCanvas = forwardRef(({ selectedPami, waypoints, setWaypoints, allTrajectories, elapsedTime, isPlaying, speedMmPerS, angularSpeedDegS, delayAfterPullCordS, selectedWaypointIndex, onSelectWaypoint, onControlsStateChange, onMousePositionChange }, ref) => {
    const canvasRef = useRef(null);
    const [undoStack, setUndoStack] = useState([]);
    const [redoStack, setRedoStack] = useState([]);
    const [mousePos, setMousePos] = useState({ x: 0, y: 0 }); // En millimètres réels
    
    // Clear redo history when changing PAMI context
    useEffect(() => {
        setUndoStack([]);
        setRedoStack([]);
    }, [selectedPami]);

    const deleteWaypointAt = (index) => {
        if (isPlaying || elapsedTime > 0) return;
        const currentWaypoints = [...(waypoints || [])];
        if (index < 0 || index >= currentWaypoints.length) return;

        const [removedPoint] = currentWaypoints.splice(index, 1);
        setWaypoints(currentWaypoints);
        setUndoStack((prev) => [...prev, { type: 'remove', index, point: removedPoint }]);
        setRedoStack([]);

        if (currentWaypoints.length === 0) {
            onSelectWaypoint?.(null);
        } else {
            onSelectWaypoint?.(Math.max(0, index - 1));
        }
    };

    useEffect(() => {
        const canvas = canvasRef.current;
        const ctx = canvas.getContext('2d');
        const img = new Image();
        img.src = '/table_map.png';
        
        img.onload = () => {
            // Config de l'animation pour les re-renders
            const render = () => {
                ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
                ctx.drawImage(img, 0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

                // Grille de points tous les 5 cm, discrète pour garder la lisibilite des trajectoires.
                ctx.fillStyle = 'rgba(0, 0, 0, 0.4)';
                for (let x = GRID_OFFSET_X_MM; x <= TABLE_WIDTH_MM; x += GRID_SPACING_MM) {
                    for (let y = 0; y <= TABLE_HEIGHT_MM; y += GRID_SPACING_MM) {
                        ctx.beginPath();
                        ctx.arc(mmToPxX(x), mmToPxY(y), 1.2, 0, 2 * Math.PI);
                        ctx.fill();
                    }
                }

                const botPositions = {};

                // 1. Dessiner les trajectoires
                Object.keys(allTrajectories).forEach(pamiId => {
                    const pts = allTrajectories[pamiId];
                    if (!pts || pts.length === 0) return;
                    
                    const isSelected = parseInt(pamiId) === selectedPami;
                    
                    // Si on est en mode édition pure, on grise les autres trajectoires
                    const isAnimActive = isPlaying || elapsedTime > 0;
                    ctx.globalAlpha = (!isAnimActive && !isSelected) ? 0.2 : 0.7;

                    ctx.beginPath();
                    ctx.moveTo(mmToPxX(pts[0].x), mmToPxY(pts[0].y));
                    for (let i = 1; i < pts.length; i++) {
                        ctx.lineTo(mmToPxX(pts[i].x), mmToPxY(pts[i].y));
                    }
                    ctx.strokeStyle = PAMI_COLORS[pamiId];
                    ctx.lineWidth = isSelected ? 3 : 2;
                    ctx.stroke();

                    // Points de passage
                    pts.forEach((wp, index) => {
                        ctx.beginPath();
                        ctx.arc(mmToPxX(wp.x), mmToPxY(wp.y), isSelected ? 6 : 4, 0, 2 * Math.PI);
                        ctx.fillStyle = index === 0 ? '#ffffff' : PAMI_COLORS[pamiId];
                        ctx.fill();
                        ctx.lineWidth = 1;
                        ctx.stroke();

                        if (isSelected && selectedWaypointIndex === index) {
                            ctx.beginPath();
                            ctx.arc(mmToPxX(wp.x), mmToPxY(wp.y), 10, 0, 2 * Math.PI);
                            ctx.strokeStyle = '#111';
                            ctx.lineWidth = 2;
                            ctx.stroke();
                        }

                        const waitS = Math.max(0, Number(wp.waitS || 0));
                        if (waitS > 0) {
                            const tx = mmToPxX(wp.x) + 8;
                            const ty = mmToPxY(wp.y) - 8;
                            ctx.font = '12px Arial';
                            ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
                            const label = `⏱ ${waitS.toFixed(1)}s`;
                            const labelW = ctx.measureText(label).width;
                            ctx.fillRect(tx - 4, ty - 10, labelW + 8, 14);
                            ctx.fillStyle = '#222';
                            ctx.fillText(label, tx, ty);
                        }

                        if (index === 0) {
                            const headingDeg = normalizeHeadingDeg(Number(wp.headingDeg || 0));
                            const headingRad = degToRad(headingDeg);
                            const startX = mmToPxX(wp.x);
                            const startY = mmToPxY(wp.y);
                            const arrowLen = 20;
                            const endX = startX + Math.cos(headingRad) * arrowLen;
                            const endY = startY - Math.sin(headingRad) * arrowLen;

                            ctx.beginPath();
                            ctx.moveTo(startX, startY);
                            ctx.lineTo(endX, endY);
                            ctx.strokeStyle = '#111';
                            ctx.lineWidth = 2;
                            ctx.stroke();

                            const ah = 6;
                            const left = headingRad + Math.PI - 0.4;
                            const right = headingRad + Math.PI + 0.4;
                            ctx.beginPath();
                            ctx.moveTo(endX, endY);
                            ctx.lineTo(endX + Math.cos(left) * ah, endY - Math.sin(left) * ah);
                            ctx.lineTo(endX + Math.cos(right) * ah, endY - Math.sin(right) * ah);
                            ctx.closePath();
                            ctx.fillStyle = '#111';
                            ctx.fill();
                        }
                    });
                    
                    ctx.globalAlpha = 1.0;

                    // 2. Calcul des positions courantes pour la simulation temporelle
                    if (isAnimActive) {
                        const effectiveElapsedTime = Math.max(0, elapsedTime - delayAfterPullCordS);
                        const pos = getPoseAtTimeWithWaitsAndTurns(
                            pts,
                            effectiveElapsedTime,
                            speedMmPerS,
                            (angularSpeedDegS * Math.PI) / 180
                        );
                        if (pos) {
                            botPositions[pamiId] = pos;
                        }
                    }
                });

                // 3. Dessiner les robots animés et vérifier les collisions
                if (isPlaying || elapsedTime > 0) {
                    const idsKey = Object.keys(botPositions);
                    const collisions = new Set();

                    // Détection quadratique précise avec OBB (Oriented Bounding Boxes)
                    for (let i = 0; i < idsKey.length; i++) {
                        for (let j = i + 1; j < idsKey.length; j++) {
                            const p1 = botPositions[idsKey[i]];
                            const p2 = botPositions[idsKey[j]];
                            
                            const rect1 = { x: p1.x, y: p1.y, w: PAMI_LENGTH_MM, h: PAMI_WIDTH_MM, angle: p1.angle };
                            const rect2 = { x: p2.x, y: p2.y, w: PAMI_LENGTH_MM, h: PAMI_WIDTH_MM, angle: p2.angle };

                            if (checkOBBCollision(rect1, rect2)) {
                                collisions.add(idsKey[i]);
                                collisions.add(idsKey[j]);
                                
                                // Indicateur visuel du choc (un lien rouge épais entre les centres impliqués)
                                ctx.beginPath();
                                ctx.moveTo(mmToPxX(p1.x), mmToPxY(p1.y));
                                ctx.lineTo(mmToPxX(p2.x), mmToPxY(p2.y));
                                ctx.strokeStyle = 'rgba(255, 0, 0, 0.8)';
                                ctx.lineWidth = 4;
                                ctx.stroke();
                            }
                        }
                    }

                    // Dessin des robots (Rectangles)
                    idsKey.forEach(id => {
                        const pos = botPositions[id];
                        const pxX = mmToPxX(pos.x);
                        const pxY = mmToPxY(pos.y);
                        
                        const wPx = (PAMI_LENGTH_MM / TABLE_WIDTH_MM) * CANVAS_WIDTH;
                        const hPx = (PAMI_WIDTH_MM / TABLE_HEIGHT_MM) * CANVAS_HEIGHT;

                        ctx.save();
                        ctx.translate(pxX, pxY);
                        // Attention: angle mathématique = Y vers le haut. Notre Canvas = Y vers le bas.
                        ctx.rotate(-pos.angle);

                        // Rectangle du PAMI centré
                        ctx.fillStyle = collisions.has(id) ? '#FF0000' : PAMI_COLORS[id];
                        ctx.fillRect(-wPx/2, -hPx/2, wPx, hPx);
                        ctx.strokeStyle = '#000';
                        ctx.lineWidth = 2;
                        ctx.strokeRect(-wPx/2, -hPx/2, wPx, hPx);

                        // Indicateur de la face avant (Un petit bloc à l'avant du rectangle)
                        ctx.fillStyle = '#000';
                        ctx.fillRect(wPx/2 - 6, -hPx/4, 6, hPx/2);
                        
                        ctx.restore();
                        
                        // Numéro du PAMI
                        ctx.fillStyle = '#FFF';
                        ctx.font = '14px Arial';
                        ctx.textAlign = 'center';
                        ctx.textBaseline = 'middle';
                        ctx.fillText(id, pxX, pxY);
                    });
                }
            };
            
            render();
        };
    }, [waypoints, allTrajectories, selectedPami, selectedWaypointIndex, elapsedTime, isPlaying, speedMmPerS, angularSpeedDegS, delayAfterPullCordS]);

    const handleMouseMove = (e) => {
        const rect = canvasRef.current.getBoundingClientRect();
        const scaleX = CANVAS_WIDTH / rect.width;
        const scaleY = CANVAS_HEIGHT / rect.height;
        const x_px = (e.clientX - rect.left) * scaleX;
        const y_px = (e.clientY - rect.top) * scaleY;
        
        setMousePos({
            x: pxToMmX(x_px),
            y: pxToMmY(y_px)
        });
    };

    const rotateStartHeading = () => {
        const currentWaypoints = [...(waypoints || [])];
        if (!currentWaypoints[0]) return;

        const beforePoint = currentWaypoints[0];
        const beforeHeading = normalizeHeadingDeg(Number(beforePoint.headingDeg || 0));
        const afterHeading = (beforeHeading + 90) % 360;
        const afterPoint = { ...beforePoint, headingDeg: afterHeading };

        currentWaypoints[0] = afterPoint;
        setWaypoints(currentWaypoints);
        setUndoStack((prev) => [...prev, { type: 'update', index: 0, beforePoint, afterPoint }]);
        setRedoStack([]);
        onSelectWaypoint?.(0);
    };

    const handleCanvasClick = (e) => {
        // Interdire le dessin pendant la lecture de l'animation
        if (isPlaying || elapsedTime > 0) return;

        const rect = canvasRef.current.getBoundingClientRect();
        const scaleX = CANVAS_WIDTH / rect.width;
        const scaleY = CANVAS_HEIGHT / rect.height;
        const x_px = (e.clientX - rect.left) * scaleX;
        const y_px = (e.clientY - rect.top) * scaleY;
        
        const currentWaypoints = waypoints || [];
        const hitThresholdPx = 10;
        let hitIndex = -1;
        for (let i = 0; i < currentWaypoints.length; i++) {
            const wp = currentWaypoints[i];
            const dx = mmToPxX(wp.x) - x_px;
            const dy = mmToPxY(wp.y) - y_px;
            if (Math.hypot(dx, dy) <= hitThresholdPx) {
                hitIndex = i;
                break;
            }
        }

        if (hitIndex >= 0) {
            if (hitIndex === 0) {
                rotateStartHeading();
                return;
            }
            onSelectWaypoint?.(hitIndex);
            return;
        }

        const x_mm = clamp(snapToGridX(pxToMmX(x_px)), 0, TABLE_WIDTH_MM);
        const y_mm = clamp(snapToGridY(pxToMmY(y_px)), 0, TABLE_HEIGHT_MM);

        const insertIndex = selectedWaypointIndex !== null
            ? Math.min(selectedWaypointIndex + 1, currentWaypoints.length)
            : currentWaypoints.length;
        const nextWaypoints = [...currentWaypoints];
        const insertedPoint = { x: x_mm, y: y_mm, waitS: 0, headingDeg: 0 };
        nextWaypoints.splice(insertIndex, 0, insertedPoint);

        setWaypoints(nextWaypoints);
        setUndoStack((prev) => [...prev, { type: 'add', index: insertIndex, point: insertedPoint }]);
        setRedoStack([]);
        onSelectWaypoint?.(insertIndex);
    };

    const handleUndo = () => {
        if (isPlaying || elapsedTime > 0) return;
        if (undoStack.length === 0) return;

        const action = undoStack[undoStack.length - 1];
        const currentWaypoints = [...(waypoints || [])];
        let nextWaypoints = currentWaypoints;

        if (action.type === 'add') {
            if (action.index >= 0 && action.index < nextWaypoints.length) {
                nextWaypoints = [...nextWaypoints];
                nextWaypoints.splice(action.index, 1);
                onSelectWaypoint?.(nextWaypoints.length > 0 ? Math.max(0, action.index - 1) : null);
            }
        } else if (action.type === 'remove') {
            nextWaypoints = [...nextWaypoints];
            nextWaypoints.splice(action.index, 0, action.point);
            onSelectWaypoint?.(action.index);
        } else if (action.type === 'update') {
            if (action.index >= 0 && action.index < nextWaypoints.length) {
                nextWaypoints = [...nextWaypoints];
                nextWaypoints[action.index] = action.beforePoint;
                onSelectWaypoint?.(action.index);
            }
        } else if (action.type === 'clear') {
            nextWaypoints = [...action.points];
            onSelectWaypoint?.(null);
        }

        setWaypoints(nextWaypoints);
        setUndoStack((prev) => prev.slice(0, -1));
        setRedoStack((prev) => [...prev, action]);
    };

    const handleRedo = () => {
        if (isPlaying || elapsedTime > 0) return;
        if (redoStack.length === 0) return;

        const action = redoStack[redoStack.length - 1];
        const currentWaypoints = [...(waypoints || [])];
        let nextWaypoints = currentWaypoints;

        if (action.type === 'add') {
            nextWaypoints = [...nextWaypoints];
            nextWaypoints.splice(action.index, 0, action.point);
            onSelectWaypoint?.(action.index);
        } else if (action.type === 'remove') {
            if (action.index >= 0 && action.index < nextWaypoints.length) {
                nextWaypoints = [...nextWaypoints];
                nextWaypoints.splice(action.index, 1);
                onSelectWaypoint?.(nextWaypoints.length > 0 ? Math.max(0, action.index - 1) : null);
            }
        } else if (action.type === 'update') {
            if (action.index >= 0 && action.index < nextWaypoints.length) {
                nextWaypoints = [...nextWaypoints];
                nextWaypoints[action.index] = action.afterPoint;
                onSelectWaypoint?.(action.index);
            }
        } else if (action.type === 'clear') {
            nextWaypoints = [];
            onSelectWaypoint?.(null);
        }

        setWaypoints(nextWaypoints);
        setRedoStack((prev) => prev.slice(0, -1));
        setUndoStack((prev) => [...prev, action]);
    };

    const handleClear = () => {
        if (isPlaying || elapsedTime > 0) return;
        const currentWaypoints = [...(waypoints || [])];
        if (currentWaypoints.length === 0) return;

        setWaypoints([]);
        setUndoStack((prev) => [...prev, { type: 'clear', points: currentWaypoints }]);
        setRedoStack([]);
        onSelectWaypoint?.(null);
    };

    useImperativeHandle(ref, () => ({
        undo: handleUndo,
        redo: handleRedo,
        clear: handleClear,
        deleteSelectedWaypoint: deleteWaypointAt,
    }), [handleUndo, handleRedo, handleClear, deleteWaypointAt]);

    useEffect(() => {
        if (!onControlsStateChange) return;
        onControlsStateChange({
            canUndo: undoStack.length > 0,
            canRedo: redoStack.length > 0,
            canClear: !!waypoints && waypoints.length > 0,
            isEditLocked: isPlaying || elapsedTime > 0,
        });
    }, [onControlsStateChange, waypoints, undoStack, redoStack, isPlaying, elapsedTime]);

    useEffect(() => {
        if (!onMousePositionChange) return;
        onMousePositionChange(mousePos);
    }, [onMousePositionChange, mousePos]);

    return (
        <div style={{ width: '100%' }}>
            <canvas 
                ref={canvasRef} 
                width={CANVAS_WIDTH} 
                height={CANVAS_HEIGHT} 
                style={{ 
                    border: '1px solid black', 
                    cursor: (isPlaying || elapsedTime > 0) ? 'not-allowed' : 'crosshair',
                    display: 'block',
                    width: '100%',
                    height: 'auto'
                }}
                onClick={handleCanvasClick}
                onMouseMove={handleMouseMove}
            />
        </div>
    );
});

export default TableCanvas;

