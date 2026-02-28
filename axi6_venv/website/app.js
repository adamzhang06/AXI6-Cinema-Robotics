document.addEventListener('DOMContentLoaded', () => {
    const playhead = document.getElementById('playhead');
    const curveArea = document.getElementById('curve-area');
    const timerDisplay = document.getElementById('timeline-timer');
    const dragIndicator = document.getElementById('drag-indicator');
    const timelineContent = document.getElementById('timeline-content');
    const zoomSlider = document.getElementById('zoom-slider');
    const zoomInBtn = document.getElementById('zoom-in-btn');
    const zoomOutBtn = document.getElementById('zoom-out-btn');
    
    let isDragging = false;
    let currentTime = 0; // State variable to preserve time across zooms
    
    // Configuration: Total timeline duration in seconds and framerate
    const durationSeconds = 60; // 1 minute max for now
    const fps = 30;
    
    // Zoom configurations
    let pixelsPerSecond = 100; // default
    
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
        timelineContent.style.minWidth = `${totalWidth}px`;
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
    }

    // Function to calculate and update playhead position and timer text
    function updatePlayheadPosition(clientX) {
        const rect = timelineContent.getBoundingClientRect();
        let x = clientX - rect.left;
        
        // Constrain playhead within the curve area bounds
        if (x < 0) x = 0;
        if (x > rect.width) x = rect.width;
        
        // Move the playhead
        playhead.style.left = `${x}px`;
        
        // Calculate the time based on progress
        const progress = x / rect.width;
        currentTime = Math.max(0, Math.min(durationSeconds, durationSeconds * progress));
        
        // Format time (HH:MM:SS:FF)
        const hours = Math.floor(currentTime / 3600);
        const minutes = Math.floor((currentTime % 3600) / 60);
        const seconds = Math.floor(currentTime % 60);
        const frames = Math.floor((currentTime % 1) * fps);
        
        const formattedTime = 
            String(hours).padStart(2, '0') + ':' +
            String(minutes).padStart(2, '0') + ':' +
            String(seconds).padStart(2, '0') + ':' +
            String(frames).padStart(2, '0');
            
        // Update the display text
        timerDisplay.textContent = formattedTime;
    }
    
    // Start dragging when clicking on the playhead
    playhead.addEventListener('mousedown', (e) => {
        isDragging = true;
        // Prevent highlighting text while dragging
        document.body.style.userSelect = 'none';
        e.preventDefault();
    });
    
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

    // Clicking anywhere on the background jumps the playhead to that spot
    timelineContent.addEventListener('mousedown', (e) => {
        if (e.target !== playhead && !playhead.contains(e.target)) {
            updatePlayheadPosition(e.clientX);
            isDragging = true;
            document.body.style.userSelect = 'none';
        }
    });

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
        
        // Configuration for ticks (Dynamic based on zoom scale)
        let majorTickInterval = 1; // Default 1 second
        let minorTickInterval = 0.25; // Default 0.25 seconds
        
        if (pixelsPerSecond > 800) {
            // Very zoomed in
            majorTickInterval = 0.25;
            minorTickInterval = 1/fps; // Frame level ticks
        } else if (pixelsPerSecond > 300) {
            // Zoomed in
            majorTickInterval = 0.5;
            minorTickInterval = 0.1;
        } else if (pixelsPerSecond < 30) {
            // Very zoomed out
            majorTickInterval = 5;
            minorTickInterval = 1;
        } else if (pixelsPerSecond < 60) {
            // Zoomed out
            majorTickInterval = 2;
            minorTickInterval = 0.5;
        }
        
        // Loop through the total duration and generate ticks
        for (let t = 0; t <= durationSeconds; t += minorTickInterval) {
            const percentage = (t / durationSeconds) * 100;
            // Add a tiny epsilon to handle floating point imprecision for module operator
            const isMajor = Math.abs((t % majorTickInterval)) < 0.001 || Math.abs((t % majorTickInterval) - majorTickInterval) < 0.001;
            
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
});
