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

            // Update line
            // TODO: Swap out polyline points generation for your Bezier/spline path generator
            const pointsStr = waypoints.map(pt => `${window.TimelineAPI.frameToX(pt.frame)},${pt.y}`).join(' ');
            polyline.setAttribute('points', pointsStr);

            // Brutal but simple: clear all point nodes and rebuild
            pointsGroup.innerHTML = '';
            
            waypoints.forEach((pt, i) => {
                const cx = window.TimelineAPI.frameToX(pt.frame);
                const cy = pt.y;
                const r = DIAMOND_RADIUS;
                
                const diamond = document.createElementNS(ns, "polygon");
                const points = `${cx},${cy - r} ${cx + r},${cy} ${cx},${cy + r} ${cx - r},${cy}`;
                
                diamond.setAttribute("points", points);
                diamond.setAttribute("fill", color);
                diamond.setAttribute("stroke", "#000");
                diamond.setAttribute("stroke-width", "1");
                diamond.classList.add('waypoint-node');
                
                // Keep the points clickable and make them look clickable
                // We set auto because the parent SVG has pointer-events: none
                diamond.setAttribute("pointer-events", "auto"); 
                diamond.style.cursor = 'grab';
                
                // Data attribute to map node back to data array
                diamond.dataset.index = i;
                
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
            if (e.button !== 0) return; // Only left-click
            if (!e.target.classList || !e.target.classList.contains('waypoint-node')) return;

            const idx = parseInt(e.target.dataset.index, 10);
            
            // Shift+Click = Delete
            if (e.shiftKey) {
                waypoints.splice(idx, 1);
                draw();
                return;
            }

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

});
