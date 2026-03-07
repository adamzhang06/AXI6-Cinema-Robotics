/**
 * curve-editor.js  —  Minimal interactive waypoint editor for timeline tracks.
 *
 * Each track (Slide, Pan, Tilt) gets its own SVG overlay.
 * Waypoints are stored as {frame, y} logical locations, where 'frame' is resilient to zoom.
 * We use SVG because it handles 6000px+ widths performantly and inherently
 * provides CSS hover/cursor native functionality without continuous mouse hit-testing.
 *
 * Interactions:
 *   Double-click container → add waypoint
 *   Left-drag point        → move waypoint (Y clamped to track bounds)
 *   Right-click point      → delete waypoint
 *   Shift + Click point    → delete waypoint (alternative)
 */

document.addEventListener('DOMContentLoaded', () => {

    // ---------------------------------------------------------------
    // DATA — one plain array of {frame, y} per track.
    // ---------------------------------------------------------------
    const trackData = {
        slide: [],
        pan:   [],
        tilt:  [],
    };

    const trackColors = {
        slide: '#FFD500',
        pan:   '#ff4444',
        tilt:  '#44ff44',
    };

    const DIAMOND_RADIUS = 6;

    // ---------------------------------------------------------------
    // SELECTION STATE — tracks which waypoint is currently selected
    // ---------------------------------------------------------------
    let selectedWaypoint = null; // { trackKey, index, element }

    function selectWaypoint(trackKey, index, element) {
        // Deselect previous
        if (selectedWaypoint && selectedWaypoint.element) {
            selectedWaypoint.element.setAttribute('stroke', '#000');
            selectedWaypoint.element.setAttribute('stroke-width', '1');
            selectedWaypoint.element.removeAttribute('filter');
            selectedWaypoint.element.style.removeProperty('filter');
        }

        // Apply selection styling using the track's own color
        const color = trackColors[trackKey];
        element.setAttribute('stroke', '#ffffff');
        element.setAttribute('stroke-width', '2.5');
        // Add a glow filter using the track color
        element.style.filter = `drop-shadow(0 0 4px ${color})`;

        selectedWaypoint = { trackKey, index, element };

        // Link with the rest of the UI selection logic in app.js
        if (window.selectTrack) {
            window.selectTrack(`track-${trackKey}`);
        }
    }

    function deselectAllWaypoints() {
        if (selectedWaypoint && selectedWaypoint.element) {
            selectedWaypoint.element.setAttribute('stroke', '#000');
            selectedWaypoint.element.setAttribute('stroke-width', '1');
            selectedWaypoint.element.removeAttribute('filter');
            selectedWaypoint.element.style.removeProperty('filter');
        }
        selectedWaypoint = null;
    }

    // Expose globally so app.js can call it when tracks are deselected
    window.deselectAllWaypoints = deselectAllWaypoints;

    // ---------------------------------------------------------------
    // INIT – set up each SVG once per track.
    // ---------------------------------------------------------------
    function initTrackSVG(laneId, svgId, trackKey) {
        const lane = document.getElementById(laneId);
        const svg = document.getElementById(svgId);
        if (!lane || !svg) return;

        const waypoints = trackData[trackKey];
        const color = trackColors[trackKey];

        // --- SVG Setup ---
        const ns = "http://www.w3.org/2000/svg";
        
        // Polyline for connecting paths (pointer-events-none so we can click through it)
        const polyline = document.createElementNS(ns, "polyline");
        polyline.setAttribute("fill", "none");
        polyline.setAttribute("stroke", color);
        polyline.setAttribute("stroke-width", "1.5");
        polyline.setAttribute("stroke-opacity", "0.6");
        polyline.setAttribute("pointer-events", "none");
        svg.appendChild(polyline);

        // Group to hold coordinate circle nodes
        const pointsGroup = document.createElementNS(ns, "g");
        svg.appendChild(pointsGroup);

        // ---------------------------------------------------------------
        // UPDATE — Rebuild the path and circle nodes
        // ---------------------------------------------------------------
        function draw() {
            // Sort left-to-right temporally
            waypoints.sort((a, b) => a.frame - b.frame);

            if (waypoints.length === 0) {
                polyline.setAttribute('points', '');
                pointsGroup.innerHTML = '';
                return;
            }

            const isHidden = window.hiddenTracks && window.hiddenTracks.has(trackKey);
            const drawColor = isHidden ? 'rgba(100, 100, 100, 0.3)' : color;
            
            // Update line
            // TODO: Swap out polyline points generation for your Bezier/spline path generator
            const pointsStr = waypoints.map(pt => `${window.TimelineAPI.frameToX(pt.frame)},${pt.y}`).join(' ');
            polyline.setAttribute('points', pointsStr);
            polyline.setAttribute('stroke', drawColor);

            // Brutal but simple: clear all point nodes and rebuild
            pointsGroup.innerHTML = '';
            
            waypoints.forEach((pt, i) => {
                const cx = window.TimelineAPI.frameToX(pt.frame);
                const cy = pt.y;
                const r = DIAMOND_RADIUS;
                
                const diamond = document.createElementNS(ns, "polygon");
                const points = `${cx},${cy - r} ${cx + r},${cy} ${cx},${cy + r} ${cx - r},${cy}`;
                
                diamond.setAttribute("points", points);
                diamond.setAttribute("fill", drawColor);
                diamond.setAttribute("stroke", "#000");
                diamond.setAttribute("stroke-width", "1");
                diamond.classList.add('waypoint-node');
                diamond.dataset.trackKey = trackKey;
                
                // Keep the points clickable and make them look clickable if active
                if (!isHidden) {
                    diamond.setAttribute("pointer-events", "auto"); 
                    diamond.style.cursor = 'grab';
                } else {
                    diamond.style.pointerEvents = 'none'; // fully disable clicks on the point itself
                }
                
                // Data attribute to map node back to data array
                diamond.dataset.index = i;
                
                // Re-apply selection styling if this is the currently selected waypoint
                if (!isHidden && selectedWaypoint && selectedWaypoint.trackKey === trackKey && selectedWaypoint.index === i) {
                    diamond.setAttribute('stroke', '#ffffff');
                    diamond.setAttribute('stroke-width', '2.5');
                    diamond.style.filter = `drop-shadow(0 0 4px ${color})`;
                    selectedWaypoint.element = diamond; // Update DOM reference
                }
                
                pointsGroup.appendChild(diamond);
            });
        }
        
        // Register to redraw when the timeline semantic zoom scale changes
        if (window.TimelineAPI && window.TimelineAPI.redrawHooks) {
            window.TimelineAPI.redrawHooks.push(draw);
        }

        // Helper: get mouse position relative to the lane's CSS layout
        function getLocalPos(e) {
            const rect = lane.getBoundingClientRect();
            return {
                x: e.clientX - rect.left,
                y: e.clientY - rect.top,
            };
        }

        // ---------------------------------------------------------------
        // INTERACTION STATE
        // ---------------------------------------------------------------
        let dragIndex = -1;
        
        // Cache lane bounds during mousedown so we don't cause layout thrashing while dragging
        let laneHeightCache = 0; 
        let isDraggingPointDOM = null; // Store the actual DOM element being dragged for style updates
        let isDraggingPointData = null; // Store the actual data ref to update it cleanly during bounds check

        // --- ADD WAYPOINT (double-click on the lane background) ---
        lane.addEventListener('dblclick', (e) => {
            if (window.hiddenTracks && window.hiddenTracks.has(trackKey)) return;
            // If they clicked an existing point, ignore
            if (e.target.classList && e.target.classList.contains('waypoint-node')) return;

            const pos = getLocalPos(e);
            const frameToInsert = window.TimelineAPI.xToFrame(pos.x);
            
            // Limit: Can only place point within timeline bounds
            if (frameToInsert < 0 || frameToInsert > window.TimelineAPI.durationSeconds * window.TimelineAPI.fps) {
                return;
            }

            // Limit: One point per frame
            if (waypoints.some(p => p.frame === frameToInsert)) {
                return; // point already exists on this frame
            }

            // TODO: Map raw pixel pos.y to motor values (x is mapped to frame)
            waypoints.push({ frame: frameToInsert, y: pos.y });
            draw();
        });

        // --- MOUSE DOWN (on a point) ---
        // We listen on the svg directly since the points are inside it
        svg.addEventListener('mousedown', (e) => {
            if (window.hiddenTracks && window.hiddenTracks.has(trackKey)) return;
            if (e.button !== 0) return; // Only left-click
            if (!e.target.classList || !e.target.classList.contains('waypoint-node')) return;

            const idx = parseInt(e.target.dataset.index, 10);
            
            // Shift+Click = Delete
            if (e.shiftKey) {
                // If we're deleting the selected waypoint, clear selection
                if (selectedWaypoint && selectedWaypoint.trackKey === trackKey && selectedWaypoint.index === idx) {
                    deselectAllWaypoints();
                }
                waypoints.splice(idx, 1);
                draw();
                return;
            }

            // Select this waypoint
            selectWaypoint(trackKey, idx, e.target);

            // Start drag
            dragIndex = idx;
            isDraggingPointData = waypoints[idx];
            laneHeightCache = lane.clientHeight; // Cache so we don't thrash layout
            isDraggingPointDOM = e.target;
            
            isDraggingPointDOM.style.cursor = 'grabbing';
            document.body.style.cursor = 'grabbing';
            
            // Prevent the timeline's main scrolling drag from firing
            e.stopPropagation();
        });

        // --- MOUSE MOVE (drag globally so we don't lose focus) ---
        document.addEventListener('mousemove', (e) => {
            if (dragIndex === -1 || !isDraggingPointData) return;

            const pos = getLocalPos(e);
            let targetFrame = window.TimelineAPI.xToFrame(pos.x);
            
            // We lock the drag boundaries dynamically between the previous and next points (if any)
            let minFrame = 0;
            let maxFrame = window.TimelineAPI.durationSeconds * window.TimelineAPI.fps;
            
            // Ensure bounds
            const sortedIdx = waypoints.indexOf(isDraggingPointData);
            if (sortedIdx > 0) {
                minFrame = waypoints[sortedIdx - 1].frame + 1;
            }
            if (sortedIdx < waypoints.length - 1) {
                maxFrame = waypoints[sortedIdx + 1].frame - 1;
            }

            // Clamp frame and Y bounds
            targetFrame = Math.max(minFrame, Math.min(maxFrame, targetFrame));

            // Update data model
            isDraggingPointData.frame = targetFrame;
            isDraggingPointData.y = Math.max(0, Math.min(laneHeightCache, pos.y));

            // Actually, because we are using SVG, instead of rebuilding the whole DOM tree on every frame 
            // inside draw(), we can just mutate the specific point and polyline attributes directly for buttery 60fps
            // performance. But for "minimum viable code" simplicity, `draw()` creates the exact same visual.
            draw(); 
        });

        // --- MOUSE UP (end drag) ---
        document.addEventListener('mouseup', () => {
            if (dragIndex !== -1) {
                dragIndex = -1;
                document.body.style.cursor = '';
                if (isDraggingPointDOM) {
                    isDraggingPointDOM.style.cursor = 'grab';
                }
                isDraggingPointDOM = null;
                isDraggingPointData = null;
            }
        });

        // --- RIGHT CLICK (delete point) ---
        svg.addEventListener('contextmenu', (e) => {
            if (e.target.classList && e.target.classList.contains('waypoint-node')) {
                e.preventDefault(); // Stop browser context menu
                const idx = parseInt(e.target.dataset.index, 10);
                waypoints.splice(idx, 1);
                draw();
            }
        });

    }

    // ---------------------------------------------------------------
    // BOOTSTRAP
    // ---------------------------------------------------------------
    // Note: We use the `lane-` wrapper div for background clicks (dblclick),
    // and the `svg-` for the point interaction.
    initTrackSVG('lane-slide', 'svg-slide', 'slide');
    initTrackSVG('lane-pan',   'svg-pan',   'pan');
    initTrackSVG('lane-tilt',  'svg-tilt',  'tilt');

    // ---------------------------------------------------------------
    // API: TRIM WAYPOINTS
    // ---------------------------------------------------------------
    if (window.TimelineAPI) {
        window.TimelineAPI.trimWaypoints = function(maxFrame) {
            let changed = false;
            ['slide', 'pan', 'tilt'].forEach(key => {
                const arr = trackData[key];
                const oldLen = arr.length;
                for (let i = arr.length - 1; i >= 0; i--) {
                    if (arr[i].frame > maxFrame) {
                        arr.splice(i, 1);
                    }
                }
                if (arr.length !== oldLen) changed = true;
            });
            if (changed && window.TimelineAPI.redrawHooks) {
                window.TimelineAPI.redrawHooks.forEach(hook => hook());
            }
        };
    }

});
