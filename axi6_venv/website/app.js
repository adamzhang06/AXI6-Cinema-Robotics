document.addEventListener('DOMContentLoaded', () => {
    const playhead = document.getElementById('playhead');
    const curveArea = document.getElementById('curve-area');
    const timerDisplay = document.getElementById('timeline-timer');
    const frameCounter = document.getElementById('frame-counter');
    const dragIndicator = document.getElementById('drag-indicator');
    const timelineContent = document.getElementById('timeline-content');
    const zoomSlider = document.getElementById('zoom-slider');
    const zoomInBtn = document.getElementById('zoom-in-btn');
    const zoomOutBtn = document.getElementById('zoom-out-btn');
    
    // Playback buttons
    const btnSkipLeft = document.getElementById('btn-skip-left');
    const btnPlayLeft = document.getElementById('btn-play-left');
    const btnPause = document.getElementById('btn-pause');
    const btnPlayRight = document.getElementById('btn-play-right');
    const btnSkipRight = document.getElementById('btn-skip-right');
    
    const btnToggleCamera = document.getElementById('btn-toggle-camera');
    
    let isDragging = false;
    let currentTime = 0; // State variable to preserve time across zooms
    let preciseTime = 0; // Unsnapped accumulator for smooth playback
    
    // Playback state
    let isPlaying = false;
    let playDirection = 1;
    let lastFrameTime = 0;
    let playAnimationId = null;
    
    // Configuration: Total timeline duration in seconds and framerate
    let durationSeconds = 120; // 2 minutes max default
    const fps = 30;
    
    // Zoom configurations
    let pixelsPerSecond = 100; // default
    
    // --- EXPOSE TIMELINE API FOR CURVE EDITOR ---
    window.TimelineAPI = {
        get fps() { return fps; },
        get durationSeconds() { return durationSeconds; },
        get pixelsPerSecond() { return pixelsPerSecond; },
        
        // Hooks for curve-editor.js to redraw shapes when zoom level changes
        redrawHooks: [],
        
        xToFrame(x) {
            const time = x / pixelsPerSecond;
            return Math.round(time * fps);
        },
        frameToX(frame) {
            const time = frame / fps;
            return time * pixelsPerSecond;
        }
    };
    
    function updateZoomRange() {
        if (!zoomSlider || !curveArea) return;
        
        // Calculate min and max zoom limits
        // Min zoom: fits entirely in the curve area
        const minPixelsPerSecond = curveArea.clientWidth / durationSeconds;
        // Max zoom: 1 second takes up the whole area
        const maxPixelsPerSecond = curveArea.clientWidth;
        
        // Read slider 0-100
        const zoomPercent = parseInt(zoomSlider.value, 10);
        
        // Logarithmic scale for smoother zooming ux
        // But for simplicity let's stick to linear interpolation between min and max right now
        pixelsPerSecond = minPixelsPerSecond + (maxPixelsPerSecond - minPixelsPerSecond) * (zoomPercent / 100);
        
        updateTimelineWidth();
        
        // Re-calculate the playhead position to stick to the same time
        const newX = (currentTime / durationSeconds) * (durationSeconds * pixelsPerSecond);
        playhead.style.left = `${newX}px`;
        
        // Ensure playhead stays in view
        // Calculate where the playhead is relative to the scroll view
        const targetScroll = newX - (curveArea.clientWidth / 2);
        curveArea.scrollLeft = Math.max(0, targetScroll);
        
        drawRuler();
    }
    
    // Set the physical width of the scrollable inner content
    function updateTimelineWidth() {
        if (!timelineContent) return;
        const totalWidth = durationSeconds * pixelsPerSecond;
        timelineContent.style.width = `${totalWidth}px`;
        timelineContent.style.minWidth = `${totalWidth}px`;
        
        // Dispatch redraw event to any registered overlays (e.g. curve editor SVG)
        if (window.TimelineAPI && window.TimelineAPI.redrawHooks) {
            window.TimelineAPI.redrawHooks.forEach(hook => hook());
        }
    }
    
    // Initialize
    updateTimelineWidth();
    
    // Hook up zoom controls
    if (zoomSlider) {
        // Run once on load to establish bounds
        setTimeout(updateZoomRange, 0); 
        
        zoomSlider.addEventListener('input', updateZoomRange);
        
        zoomInBtn.addEventListener('click', () => {
            zoomSlider.value = Math.min(100, parseInt(zoomSlider.value) + 10);
            updateZoomRange();
        });
        
        
        zoomOutBtn.addEventListener('click', () => {
            zoomSlider.value = Math.max(0, parseInt(zoomSlider.value) - 10);
            updateZoomRange();
        });
        
        const zoomContainer = document.getElementById('zoom-controls-container');
        if (zoomContainer) {
            zoomContainer.addEventListener('wheel', (e) => {
                e.preventDefault();
                const step = 2; // Sensitivity for wheel ticks
                
                if (e.deltaY < 0) {
                    // Scrolling up -> Zoom In
                    zoomSlider.value = Math.min(100, parseInt(zoomSlider.value) + step);
                } else if (e.deltaY > 0) {
                    // Scrolling down -> Zoom Out
                    zoomSlider.value = Math.max(0, parseInt(zoomSlider.value) - step);
                }
                
                updateZoomRange();
            }, { passive: false });
        }
    }

    function formatTimecode(time) {
        const safeTime = Math.round(time * 1000) / 1000;
        const hours = Math.floor(safeTime / 3600);
        const minutes = Math.floor((safeTime % 3600) / 60);
        const seconds = Math.floor(safeTime % 60);
        const frames = Math.round((safeTime % 1) * fps);
        
        return String(hours).padStart(2, '0') + ':' +
               String(minutes).padStart(2, '0') + ':' +
               String(seconds).padStart(2, '0') + ':' +
               String(frames).padStart(2, '0');
    }

    // Set up Max Timeline Input
    const maxTimeInput = document.getElementById('timeline-max-time');
    const totalFramesDisplay = document.getElementById('total-frames');

    function updateMaxTimeUI() {
        if (maxTimeInput) maxTimeInput.value = formatTimecode(durationSeconds);
        if (totalFramesDisplay) totalFramesDisplay.textContent = `/ ${Math.round(durationSeconds * fps)}`;
    }
    
    if (maxTimeInput) {
        updateMaxTimeUI();

        maxTimeInput.addEventListener('keydown', (e) => {
            const el = e.target;
            let val = el.value;
            let pos = el.selectionStart;
            let end = el.selectionEnd;
            let hasSelection = end > pos;
            
            // Safety snap: if they clicked somehow into a forbidden spot
            if (pos === 3 || pos === 6 || pos === 9) pos--;

            // Handle multi-character selection
            if (hasSelection) {
                if (e.key === 'Backspace' || e.key === 'Delete') {
                    e.preventDefault();
                    let newStr = val;
                    for (let i = pos; i < end; i++) {
                        if (newStr[i] !== ':') {
                            newStr = newStr.substring(0, i) + '0' + newStr.substring(i + 1);
                        }
                    }
                    el.value = newStr;
                    el.setSelectionRange(pos, pos);
                    triggerDurationChange(newStr);
                    return;
                } else if (/^\d$/.test(e.key)) {
                    e.preventDefault();
                    let newStr = val;
                    for (let i = pos; i < end; i++) {
                        if (newStr[i] !== ':') {
                            newStr = newStr.substring(0, i) + '0' + newStr.substring(i + 1);
                        }
                    }
                    let repPos = pos;
                    if (newStr[repPos] === ':') repPos++;
                    
                    if (repPos < end) {
                        newStr = newStr.substring(0, repPos) + e.key + newStr.substring(repPos + 1);
                        let newPos = repPos + 1;
                        if (newPos === 3 || newPos === 6 || newPos === 9) newPos--;
                        el.value = newStr;
                        el.setSelectionRange(newPos, newPos);
                        triggerDurationChange(newStr);
                    }
                    return;
                } else if (e.key === 'ArrowLeft') {
                    e.preventDefault();
                    el.setSelectionRange(pos, pos);
                    return;
                } else if (e.key === 'ArrowRight') {
                    e.preventDefault();
                    if (end === 3 || end === 6 || end === 9) end++;
                    el.setSelectionRange(end, end);
                    return;
                }
            }

            // Only handle Backspace, Delete, Arrow keys, and Digits manually
            if (e.key === 'Backspace') {
                e.preventDefault();
                if (pos > 0) {
                    pos--;
                    if (val[pos] === ':') pos--;
                    
                    val = val.substring(0, pos) + '0' + val.substring(pos + 1);
                    el.value = val;
                    
                    let newPos = pos;
                    if (newPos === 3 || newPos === 6 || newPos === 9) newPos--;
                    
                    el.setSelectionRange(newPos, newPos);
                    triggerDurationChange(val);
                }
            } else if (e.key === 'Delete') {
                e.preventDefault();
                let delPos = pos;
                if (val[delPos] === ':') delPos++;
                
                if (delPos < val.length) {
                    val = val.substring(0, delPos) + '0' + val.substring(delPos + 1);
                    el.value = val;
                    el.setSelectionRange(pos, pos); // Cursor strictly stays where it was
                    triggerDurationChange(val);
                }
            } else if (e.key === 'ArrowLeft') {
                e.preventDefault();
                if (pos > 0) {
                    pos--;
                    if (pos === 3 || pos === 6 || pos === 9) pos--; // skip the forbidden index natively
                    el.setSelectionRange(pos, pos);
                }
            } else if (e.key === 'ArrowRight') {
                e.preventDefault();
                if (pos < val.length) {
                    pos++;
                    if (pos === 3 || pos === 6 || pos === 9) pos++; // skip the forbidden index natively
                    el.setSelectionRange(pos, pos);
                }
            } else if (/^\d$/.test(e.key)) {
                e.preventDefault();
                let repPos = pos;
                if (val[repPos] === ':') repPos++;
                
                if (repPos < val.length) {
                    val = val.substring(0, repPos) + e.key + val.substring(repPos + 1);
                    el.value = val;
                    
                    let newPos = repPos + 1;
                    if (newPos === 3 || newPos === 6 || newPos === 9) newPos--; // safety snap
                    
                    el.setSelectionRange(newPos, newPos);
                    triggerDurationChange(val);
                }
            } else if (e.key === 'Tab' || e.key === 'Enter') {
                // let it pass
            } else if (e.metaKey || e.ctrlKey) {
                // let shortcuts pass
            } else {
                e.preventDefault();
            }
        });

        // Keep normal mouse clicking, but if they click right of a colon, it'll snap perfectly on the first keypress. This avoids visual thrashing.
        maxTimeInput.addEventListener('paste', e => e.preventDefault());

        function triggerDurationChange(valStr) {
            const parts = valStr.split(':').map(Number);
            if (parts.length === 4 && !parts.some(isNaN)) {
                const newDuration = (parts[0] * 3600) + (parts[1] * 60) + parts[2] + (parts[3] / fps);
                if (newDuration > 0 && newDuration <= 3600 * 2) { 
                    durationSeconds = newDuration;
                    
                    if (window.TimelineAPI && window.TimelineAPI.trimWaypoints) {
                        window.TimelineAPI.trimWaypoints(durationSeconds * fps);
                    }
                    
                    if (totalFramesDisplay) totalFramesDisplay.textContent = `/ ${Math.round(durationSeconds * fps)}`;
                    
                    updateZoomRange(); // Recalculates limits & calls drawRuler
                    
                    if (currentTime > durationSeconds) {
                        setTime(durationSeconds);
                    }
                }
            }
        }
    }

    // Helper to jump to a specific time, update UI strictly, and keep playhead anchored during play
    function setTime(time, isSkip = false) {
        preciseTime = Math.max(0, Math.min(durationSeconds, time));
        
        // Snap time to the nearest frame (30fps)
        const frameDuration = 1 / fps;
        const snappedTime = Math.round(preciseTime / frameDuration) * frameDuration;
        
        currentTime = Math.max(0, Math.min(durationSeconds, snappedTime));
        
        const newX = currentTime * pixelsPerSecond;
        playhead.style.left = `${newX}px`;
        
        // Format time (HH:MM:SS:FF) avoiding floating point precision errors
        const safeTime = Math.round(currentTime * 1000) / 1000;
            
        // Update the display text
        timerDisplay.textContent = formatTimecode(safeTime);
        
        // Update absolute frame counter UI
        if (frameCounter) {
            const absoluteFrame = Math.round(safeTime * fps);
            frameCounter.textContent = absoluteFrame;
        }
        
        // If we are explicitly skipping to a point (Skip buttons, playhead jump), strictly center the camera on it
        if (isSkip && curveArea) {
            const rect = curveArea.getBoundingClientRect();
            curveArea.scrollLeft = newX - (rect.width / 2);
            return;
        }

        // If we are playing automatically, we want to smoothly scroll exactly with the playhead to keep it visible.
        if (isPlaying && curveArea) {
            const rect = curveArea.getBoundingClientRect();
            const relativeX = newX - curveArea.scrollLeft;
            const margin = 80; // Same visual edge distance as dragging threshold
            
            // If the playhead tries to pass the right margin, smoothly push the scroll window right exactly with it
            if (relativeX > rect.width - margin && playDirection === 1) {
                curveArea.scrollLeft = newX - (rect.width - margin);
            // If the playhead tries to pass the left margin playing backwards, smoothly push left
            } else if (relativeX < margin && playDirection === -1) {
                curveArea.scrollLeft = newX - margin;
            }
        }
    }

    // Function to calculate and update playhead position and timer text from mouse coordinate
    function updatePlayheadPosition(clientX) {
        const rect = timelineContent.getBoundingClientRect();
        let x = clientX - rect.left;
        
        // Constrain playhead within the curve area bounds
        if (x < 0) x = 0;
        if (x > rect.width) x = rect.width;
        
        const progress = x / rect.width;
        const mappedTime = durationSeconds * progress;
        
        setTime(mappedTime);
    }
    
    // Start dragging when clicking on the playhead tab specifically
    const playheadTab = document.getElementById('playhead-tab');
    if (playheadTab) {
        playheadTab.addEventListener('mousedown', (e) => {
            stopPlayback(); // Stop any auto-play
            isDragging = true;
            // Prevent highlighting text while dragging
            document.body.style.userSelect = 'none';
            e.preventDefault();
            e.stopPropagation();
        });
    }
    
    // Auto-scrolling variables
    let scrollAnimationFrame = null;
    let scrollVelocity = 0;
    let lastClientX = 0; // Keep track of JS mouse position
    
    // Update playhead on mouse move across the whole document
    document.addEventListener('mousemove', (e) => {
        if (!isDragging) return;
        // Auto-scrolling logic based on viewport proximity
        const rect = curveArea.getBoundingClientRect();
        
        // Prevent the playhead from visually sliding out of the box while dragging
        let constrainedClientX = e.clientX;
        if (constrainedClientX < rect.left) constrainedClientX = rect.left;
        if (constrainedClientX > rect.right) constrainedClientX = rect.right;
        
        lastClientX = constrainedClientX;
        
        // Calculate the playhead position internally using the constrained X
        updatePlayheadPosition(constrainedClientX);
        
        // Configuration for the auto-scroll speed and trigger zone
        const edgeZone = 80; // Start scrolling 80px before the edge
        const maxScrollSpeed = 25; // Maximum scroll speed
        
        // Distance from edges (negative means mouse is PAST the edge)
        const distFromRight = rect.right - e.clientX;
        const distFromLeft = e.clientX - rect.left;
        
        if (distFromRight < edgeZone) {
            // Mouse is near or past the right edge
            let intensity = 1 - Math.max(0, distFromRight) / edgeZone;
            // Cap intensity at 1 (max speed) if we are way past the edge
            if (intensity > 1) intensity = 1; 
            scrollVelocity = intensity * maxScrollSpeed;
            startAutoScroll();
        } else if (distFromLeft < edgeZone) {
            // Mouse is near or past the left edge
            let intensity = 1 - Math.max(0, distFromLeft) / edgeZone; 
            if (intensity > 1) intensity = 1;
            scrollVelocity = -intensity * maxScrollSpeed;
            startAutoScroll();
        } else {
            // Mouse is strictly inside the safe zone
            stopAutoScroll();
        }
    });
    
    // Continuously scroll the curve area horizontally
    function autoScrollLoop() {
        if (!isDragging || scrollVelocity === 0) {
            stopAutoScroll();
            return;
        }
        
        curveArea.scrollLeft += scrollVelocity;
        
        // Since the window scrolled out from under us, we need to recalculate the playhead position based on that new scroll offset
        updatePlayheadPosition(lastClientX);
        
        scrollAnimationFrame = requestAnimationFrame(autoScrollLoop);
    }
    
    function startAutoScroll() {
        if (!scrollAnimationFrame) {
            autoScrollLoop();
        }
    }
    
    function stopAutoScroll() {
        if (scrollAnimationFrame) {
            cancelAnimationFrame(scrollAnimationFrame);
            scrollAnimationFrame = null;
        }
        scrollVelocity = 0;
    }
    
    // Stop dragging when mouse goes up anywhere
    document.addEventListener('mouseup', () => {
        if (isDragging) {
            isDragging = false;
            stopAutoScroll();
            document.body.style.userSelect = '';
        }
    });

    // Clicking anywhere on the ruler jumps the playhead to that spot
    const timelineRuler = document.getElementById('timeline-ruler');
    if (timelineRuler) {
        timelineRuler.addEventListener('mousedown', (e) => {
            stopPlayback();
            updatePlayheadPosition(e.clientX);
            isDragging = true;
            document.body.style.userSelect = 'none';
        });
    }

    // --- Playback Controls Logic --- //
    
    function playLoop(timestamp) {
        if (!isPlaying) return;
        
        if (!lastFrameTime) lastFrameTime = timestamp;
        
        const deltaSeconds = (timestamp - lastFrameTime) / 1000;
        lastFrameTime = timestamp;
        
        const nextTime = preciseTime + (deltaSeconds * playDirection);
        setTime(nextTime);
        
        // Stop playback if we hit the actual boundary while moving in that direction
        if ((preciseTime <= 0 && playDirection === -1) || (preciseTime >= durationSeconds && playDirection === 1)) {
            stopPlayback();
        } else {
            playAnimationId = requestAnimationFrame(playLoop);
        }
    }

    function startPlayback(direction) {
        if (isPlaying && playDirection === direction) return; // already playing
        
        playDirection = direction;
        
        // If at the end and trying to play forward, restart from 0
        if (playDirection === 1 && preciseTime >= durationSeconds - 0.001) {
            setTime(0, true);
        }
        // If at the start and trying to play backwards, restart from end
        if (playDirection === -1 && preciseTime <= 0.001) {
            setTime(durationSeconds, true);
        }
        
        if (!isPlaying) {
            isPlaying = true;
            lastFrameTime = performance.now();
            playAnimationId = requestAnimationFrame(playLoop);
        }
    }
    
    function stopPlayback() {
        isPlaying = false;
        if (playAnimationId) {
            cancelAnimationFrame(playAnimationId);
            playAnimationId = null;
        }
    }

    // Connect playback buttons
    if (btnSkipLeft) btnSkipLeft.addEventListener('click', () => { stopPlayback(); setTime(0, true); });
    if (btnSkipRight) btnSkipRight.addEventListener('click', () => { stopPlayback(); setTime(durationSeconds, true); });
    if (btnPlayLeft) btnPlayLeft.addEventListener('click', () => startPlayback(-1));
    if (btnPlayRight) btnPlayRight.addEventListener('click', () => startPlayback(1));
    if (btnPause) btnPause.addEventListener('click', stopPlayback);

    // Helper to format time for ruler labels
    function formatRulerTime(totalTime) {
        const hours = Math.floor(totalTime / 3600);
        const minutes = Math.floor((totalTime % 3600) / 60);
        const seconds = Math.floor(totalTime % 60);
        const frames = Math.floor((totalTime % 1) * fps);
        return String(hours).padStart(2, '0') + ':' +
               String(minutes).padStart(2, '0') + ':' +
               String(seconds).padStart(2, '0') + ':' +
               String(frames).padStart(2, '0');
    }

    // Function to draw the timeline ruler tick marks and text
    function drawRuler() {
        const ruler = document.getElementById('timeline-ruler');
        if (!ruler) return;
        ruler.innerHTML = ''; // Clear existing
        
        const width = timelineContent.getBoundingClientRect().width;
        if (width === 0) return; // Not visible yet
        
        // Configuration for ticks (Dynamic based on zoom scale to prevent text overlap)
        let majorTickInterval;
        let minorTickInterval;
        
        if (pixelsPerSecond > 800) {
            majorTickInterval = 0.25;
            minorTickInterval = 1/fps; // Frame level ticks
        } else if (pixelsPerSecond > 300) {
            majorTickInterval = 0.5;
            minorTickInterval = 0.1;
        } else if (pixelsPerSecond > 120) {
            majorTickInterval = 1;
            minorTickInterval = 0.25;
        } else if (pixelsPerSecond > 60) {
            majorTickInterval = 2;
            minorTickInterval = 0.5;
        } else if (pixelsPerSecond > 30) {
            majorTickInterval = 5;
            minorTickInterval = 1;
        } else if (pixelsPerSecond > 15) {
            majorTickInterval = 10;
            minorTickInterval = 2;
        } else if (pixelsPerSecond > 5) {
            majorTickInterval = 30;
            minorTickInterval = 5;
        } else {
            majorTickInterval = 60;
            minorTickInterval = 10;
        }
        
        // Loop through the total duration and generate ticks
        for (let t = 0; t <= durationSeconds; t += minorTickInterval) {
            // Round t to 3 decimal places to prevent floating point drift when accumulating (e.g. 5.00000000001)
            const tRounded = Math.round(t * 1000) / 1000;
            
            const percentage = (tRounded / durationSeconds) * 100;
            
            // Add a tiny epsilon and use the firmly rounded t to check for major intervals
            const isMajor = Math.abs((tRounded % majorTickInterval)) < 0.001 || Math.abs((tRounded % majorTickInterval) - majorTickInterval) < 0.001;
            
            const tick = document.createElement('div');
            tick.className = `absolute bottom-0 w-[1px] bg-white opacity-${isMajor ? '30' : '10'}`;
            tick.style.height = isMajor ? '12px' : '6px';
            tick.style.left = `${percentage}%`;
            
            ruler.appendChild(tick);
            
            // Draw label only for major ticks, and explicitly hide them for the very start and very end
            if (isMajor && t > 0.01 && t < durationSeconds - 0.01) {
                const label = document.createElement('div');
                label.className = 'absolute top-1 text-[9px] font-mono text-white/40 transform -translate-x-1/2';
                label.style.left = `${percentage}%`;
                label.textContent = formatRulerTime(t);
                
                ruler.appendChild(label);
            }
        }
    }
    
    // Draw ruler on load and on window resize
    drawRuler();
    window.addEventListener('resize', drawRuler);
    
    // --- Track Selection --- //
    const trackBlocks = document.querySelectorAll('.track-block');
    const selectedTrackInfo = document.getElementById('selected-track-info');
    const selectedTrackName = document.getElementById('selected-track-name');
    const selectedTrackColor = document.getElementById('selected-track-color');

    window.deselectAllTracks = () => {
        trackBlocks.forEach(b => {
            b.classList.remove('selected');
            b.style.borderLeftColor = '';
        });
        if (selectedTrackInfo) selectedTrackInfo.classList.add('hidden');
        
        if (window.deselectAllWaypoints) window.deselectAllWaypoints();
    };

    window.selectTrack = (trackId) => {
        if (window.operationMode === 'tracking') return;
        
        // Remove highlight from all tracks
        trackBlocks.forEach(b => {
             b.classList.remove('selected');
             b.style.borderLeftColor = '';
        });
        
        const block = document.getElementById(trackId);
        if (!block) return;
        
        block.classList.add('selected');
        const color = block.getAttribute('data-color');
        block.style.borderLeftColor = color;
        
        const name = block.getAttribute('data-name');
        if (selectedTrackName) selectedTrackName.textContent = name + ' Axis';
        if (selectedTrackColor) {
            selectedTrackColor.style.backgroundColor = color;
            selectedTrackColor.style.boxShadow = `0 0 8px ${color}`;
        }
        
        if (selectedTrackInfo) selectedTrackInfo.classList.remove('hidden');
        const trackingInfo = document.getElementById('selected-tracking-info');
        if (trackingInfo) trackingInfo.classList.add('hidden');
    };

    trackBlocks.forEach(block => {
        block.addEventListener('click', () => {
            if (window.operationMode === 'tracking') return;
            
            const trackId = block.id.replace('track-', '');
            if (window.hiddenTracks && window.hiddenTracks.has(trackId)) return;
            
            window.deselectAllTracks();
            window.selectTrack(block.id);
        });
    });
    
    // --- Track Visibility Toggle --- //
    const visibilityButtons = document.querySelectorAll('.visibility-toggle-btn');
    const eyeOpenSVG = `<path d="M2 12s3-7 10-7 10 7 10 7-3 7-10 7-10-7-10-7Z" /><circle cx="12" cy="12" r="3" />`;
    const eyeClosedSVG = `<path d="M9.88 9.88a3 3 0 1 0 4.24 4.24" /><path d="M10.73 5.08A10.43 10.43 0 0 1 12 5c7 0 10 7 10 7a13.16 13.16 0 0 1-1.67 2.68" /><path d="M6.61 6.61A13.526 13.526 0 0 0 2 12s3 7 10 7a9.74 9.74 0 0 0 5.39-1.61" /><line x1="2" x2="22" y1="2" y2="22" />`;

    window.hiddenTracks = new Set();

    visibilityButtons.forEach(btn => {
        btn.addEventListener('click', (e) => {
            e.stopPropagation();
            const svg = btn.querySelector('svg');
            const trackBlock = btn.closest('.track-block');
            const indicator = trackBlock.querySelector('.track-indicator');
            const trackId = trackBlock.id.replace('track-', '');
            
            if (svg.classList.contains('eye-open')) {
                // Switch to closed state
                svg.classList.remove('eye-open');
                svg.classList.add('eye-closed');
                svg.innerHTML = eyeClosedSVG;
                
                // Style button red
                btn.classList.remove('text-white/40', 'hover:text-white');
                btn.classList.add('text-red-500', 'hover:text-red-400');
                
                // Dim indicator
                indicator.style.opacity = '0.2';
                indicator.style.boxShadow = 'none';
                
                const overlay = document.getElementById(`overlay-${trackId}`);
                if (overlay) overlay.classList.add('opacity-100');
                
                window.hiddenTracks.add(trackId);
                
                if (trackBlock.classList.contains('selected')) {
                    window.deselectAllTracks();
                }
            } else {
                // Switch to open state
                svg.classList.remove('eye-closed');
                svg.classList.add('eye-open');
                svg.innerHTML = eyeOpenSVG;
                
                // Revert button styling
                btn.classList.remove('text-red-500', 'hover:text-red-400');
                btn.classList.add('text-white/40', 'hover:text-white');
                
                // Restore indicator
                indicator.style.opacity = '1';
                indicator.style.boxShadow = ''; // Inherits from original CSS class
                
                const overlay = document.getElementById(`overlay-${trackId}`);
                if (overlay) overlay.classList.remove('opacity-100');
                
                window.hiddenTracks.delete(trackId);
            }
            
            // Re-draw curves to reflect new disabled styling
            if (window.TimelineAPI && window.TimelineAPI.redrawHooks) {
                window.TimelineAPI.redrawHooks.forEach(hook => hook());
            }
        });
    });
    
    // --- WebCam Initialization --- //
    let isCameraOn = false;
    let webcamStream = null;

    if (btnToggleCamera) {
        btnToggleCamera.classList.add('text-white/50');
    }

    async function initWebcam() {
        const videoElement = document.getElementById('webcam-preview');
        if (!videoElement) return;

        try {
            // Request permission and open the highest resolution video stream available up to 4K
            webcamStream = await navigator.mediaDevices.getUserMedia({ 
                video: { 
                    facingMode: "user", // Use front camera by default if available
                    width: { ideal: 1920 },
                    height: { ideal: 1080 }
                }, 
                audio: false 
            });
            
            videoElement.srcObject = webcamStream;
        } catch (err) {
            console.error("Error accessing webcam:", err);
            isCameraOn = false;
            if (btnToggleCamera) btnToggleCamera.classList.add('opacity-50');
            // Optionally, we could show a fallback UI or error message on the screen here
        }
    }
    
    // Toggle camera visibility
    if (btnToggleCamera) {
        btnToggleCamera.addEventListener('click', async () => {
            const videoElement = document.getElementById('webcam-preview');
            const indicator = document.getElementById('camera-indicator');
            if (!videoElement) return;

            isCameraOn = !isCameraOn;
            
            if (isCameraOn) {
                // Determine if we need to request the stream again
                if (!webcamStream || !webcamStream.active) {
                    await initWebcam();
                } else {
                    videoElement.play();
                }
                videoElement.classList.remove('hidden');
                
                // Toggle Labels
                const viewportLogo = document.getElementById('viewport-logo');
                const viewportLogoOutline = document.getElementById('viewport-logo-outline');
                const cameraLabelContainer = document.getElementById('camera-label-container');
                if (viewportLogo) viewportLogo.classList.add('hidden');
                if (viewportLogoOutline) viewportLogoOutline.classList.add('hidden');
                if (cameraLabelContainer) cameraLabelContainer.classList.remove('hidden');
                
                if (indicator) indicator.classList.remove('hidden');
                btnToggleCamera.classList.add('text-white');
                btnToggleCamera.classList.remove('text-white/50');
            } else {
                videoElement.pause();
                
                // Actively stop the hardware stream to save power and turn off the green light on Mac
                if (webcamStream) {
                    webcamStream.getTracks().forEach(track => track.stop());
                }
                videoElement.srcObject = null;
                webcamStream = null;

                videoElement.classList.add('hidden');
                
                // Toggle Labels
                const viewportLogo = document.getElementById('viewport-logo');
                const viewportLogoOutline = document.getElementById('viewport-logo-outline');
                const cameraLabelContainer = document.getElementById('camera-label-container');
                if (viewportLogo) viewportLogo.classList.remove('hidden');
                if (viewportLogoOutline) viewportLogoOutline.classList.remove('hidden');
                if (cameraLabelContainer) cameraLabelContainer.classList.add('hidden');
                
                if (indicator) indicator.classList.add('hidden');
                btnToggleCamera.classList.remove('text-white');
                btnToggleCamera.classList.add('text-white/50');
            }
        });
    }
});
